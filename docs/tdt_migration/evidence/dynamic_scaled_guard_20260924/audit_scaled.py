#!/usr/bin/env python3
"""Independent raw-data audit for the single scaled-guard QP simulation."""
import collections
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRIAL = ROOT / "build/tdt_p2b/runs/dynamic_scaled_guard_pilot_v1/tdt_qp_1"
SERIES = TRIAL.parent
sys.path.insert(0, str(HERE.parent / "dynamic_guard_pilot_20260923"))
sys.path.insert(0, str(HERE.parent / "sim_stop_probe_20260923"))
from guard_core import certificate
from sweep_shadow import fixture_sweep


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    for line in path.open():
        yield json.loads(line)


def cell_at(grid, x, y):
    ox, oy = grid["origin"]
    r = grid["resolution"]
    ix, iy = math.floor((x - ox) / r), math.floor((y - oy) / r)
    if not (0 <= ix < grid["width"] and 0 <= iy < grid["height"]):
        return None
    return grid["data"][iy * grid["width"] + ix]


def main():
    inputs = json.loads((SERIES / "inputs.json").read_text())
    old_profile = ROOT / "build/tdt_p2b/runs/dynamic_sweep_routing_pilot_v2/profile.yaml"
    assert sha(SERIES / "profile.yaml") == sha(old_profile) == inputs["profile_sha256"]
    for name, digest in inputs["files"].items():
        assert sha(ROOT / name) == digest, name
    summary = json.loads((TRIAL / "observation/summary.json").read_text())
    dynamic = json.loads((TRIAL / "dynamic_summary.json").read_text())
    assert summary["evidence_valid"] and dynamic["evidence_valid"]
    assert dynamic["original_static_summary_matches"]
    decisions = list(rows(TRIAL / "guard_decisions.jsonl"))
    assert decisions[0]["kind"] == "configuration"
    body = decisions[0]["body"]
    sweep, fixture = fixture_sweep()
    actual = [row for row in decisions if row["kind"] == "decision"]
    counts = collections.Counter(row["reason"] for row in actual)
    uncertified = []
    for row in actual:
        emitted = row["emitted"]
        recomputed = certificate(row["pose"], row["odom_speed"], emitted, body, sweep)
        assert recomputed["safe_under_model"] == row["certificate"]["safe_under_model"]
        assert abs(recomputed["body_lower_m"] - row["certificate"]["body_lower_m"]) < 1e-9
        assert abs(recomputed["padded_lower_m"] - row["certificate"]["padded_lower_m"]) < 1e-9
        if row["reason"] == "admit":
            assert row["scale"] == 1 and emitted == row["proposed"]
            assert recomputed["safe_under_model"]
        elif row["reason"] == "scale_sweep":
            assert 0 < row["scale"] < 1 and recomputed["safe_under_model"]
            assert not row["full_command_certificate"]["safe_under_model"]
            assert all(abs(a - row["scale"] * b) < 1e-12
                       for a, b in zip(emitted, row["proposed"]))
        elif row["reason"] == "reject_unstoppable":
            assert row["scale"] == 0 and emitted == [0, 0, 0]
            assert not recomputed["safe_under_model"]
            uncertified.append(row)
        else:
            raise AssertionError(row["reason"])
    first_scaled = next(row for row in actual if row["reason"] == "scale_sweep")
    chain = list(rows(TRIAL / "observation/command_chain.jsonl"))
    chain_counts = collections.Counter(row["topic"] for row in chain)
    near_first = [row for row in chain if row["topic"] in
                  ("/cmd_vel", "/cmd_vel_guarded", "/simulation/chassis/cmd_vel")
                  and abs(row["t"] - first_scaled["sim_s"]) < 0.015]
    maps = {}
    for name, filename, lethal in (("global", "costmap.jsonl", 100),
                                   ("local", "raw_local_maps.jsonl", 254)):
        all_maps = list(rows(TRIAL / "observation" / filename))
        in_window = [g for g in all_maps if g["t"] >= 24 and cell_at(g, 4.9, 0) is not None]
        maps[name] = {"recorded": len(all_maps),
                      "sweep_center_covered": len(in_window),
                      "sweep_center_lethal": sum(cell_at(g, 4.9, 0) == lethal for g in in_window)}
        assert in_window and maps[name]["sweep_center_lethal"] == len(in_window)
    geometry = dynamic["geometry"]["new_car_reference"]
    result = {
        "schema": "tdt_dynamic_scaled_guard_audit/v1",
        "profile_sha256": inputs["profile_sha256"],
        "same_profile_as_sweep_routing_v2": True,
        "fixture_source": fixture,
        "docker_exit": int((TRIAL / "docker_exit.txt").read_text()),
        "runtime_geometry_capture_exit": int((TRIAL / "geometry_capture_exit.txt").read_text()),
        "action_status": summary["action_status"], "preflight_status": summary["preflight"]["status"],
        "timed_out": summary["timed_out"], "recoveries": summary["recoveries"],
        "final_xy_error_m": summary["final_xy_error_m"],
        "final_yaw_error_rad": summary["final_yaw_error_rad"],
        "cross_track_rms_m": summary["cross_track_rms_m"],
        "checks": dynamic["checks"],
        "limited_dynamic_geometry_and_goal_pass": dynamic["limited_dynamic_geometry_and_goal_pass"],
        "body_moving_interpolation_bound_m": geometry["body"]["moving_interpolation_bound_m"],
        "padded_moving_interpolation_bound_m": geometry["padded"]["moving_interpolation_bound_m"],
        "max_gazebo_gap_s": dynamic["continuity"]["max_gazebo_gap_s"],
        "max_gazebo_ros_position_difference_m": dynamic["continuity"]["max_gazebo_ros_position_difference_m"],
        "guard_decisions_by_reason": dict(counts),
        "guard_decisions_recomputed": len(actual),
        "model_unstoppable_zero_commands": len(uncertified),
        "model_unstoppable_time_range_s": [min(x["sim_s"] for x in uncertified),
                                             max(x["sim_s"] for x in uncertified)] if uncertified else None,
        "first_scaled_command": {k: first_scaled[k] for k in
                                 ("sim_s", "pose", "proposed", "emitted", "scale", "certificate")},
        "near_first_scale_command_chain": [
            {k: row[k] for k in ("topic", "t", "vx", "vy", "wz")} for row in near_first],
        "command_chain_counts": dict(chain_counts),
        "costmap_marking": maps,
        "qp_adopted": dynamic["qp_adopted"],
        "validated_astar_fallbacks": dynamic["validated_astar_fallbacks"],
        "yaw_metrics": dynamic["yaw_metrics"],
        "accepted_for_deployment": False,
    }
    with (HERE / "scaled_audit.json").open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")


if __name__ == "__main__":
    main()
