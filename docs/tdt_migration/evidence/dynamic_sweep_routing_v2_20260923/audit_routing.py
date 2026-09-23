#!/usr/bin/env python3
"""Audit the two immutable point-cloud routing pilots and their published maps."""
import collections
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
RUNS = ROOT / "build/tdt_p2b/runs"
V1 = RUNS / "dynamic_sweep_routing_pilot_v1"
V2 = RUNS / "dynamic_sweep_routing_pilot_v2"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    for line in path.open():
        yield json.loads(line)


def cell_at(grid, x, y):
    ox, oy = grid["origin"]
    res = grid["resolution"]
    ix = math.floor((x - ox) / res)
    iy = math.floor((y - oy) / res)
    if not 0 <= ix < grid["width"] or not 0 <= iy < grid["height"]:
        return None
    return grid["data"][iy * grid["width"] + ix]


def audit_trial(series):
    trial = series / "tdt_qp_1"
    s = json.loads((trial / "observation/summary.json").read_text())
    d = json.loads((trial / "dynamic_summary.json").read_text())
    inputs = json.loads((series / "inputs.json").read_text())
    guard = collections.Counter(row.get("reason", row["kind"])
                                for row in rows(trial / "guard_decisions.jsonl"))
    out = {
        "profile_sha256": sha(series / "profile.yaml"),
        "profile_matches_input": sha(series / "profile.yaml") == inputs["profile_sha256"],
        "docker_exit": int((trial / "docker_exit.txt").read_text()),
        "preflight": s["preflight"], "action_status": s["action_status"],
        "timed_out": s["timed_out"], "recoveries": s["recoveries"],
        "final_pose": s["final_pose"], "final_xy_error_m": s["final_xy_error_m"],
        "final_yaw_error_rad": s["final_yaw_error_rad"],
        "evidence_valid": d["evidence_valid"],
        "dynamic_gate_pass": d["limited_dynamic_geometry_and_goal_pass"],
        "body_moving_interpolation_bound_m": d["geometry"]["new_car_reference"]["body"]["moving_interpolation_bound_m"],
        "padded_moving_interpolation_bound_m": d["geometry"]["new_car_reference"]["padded"]["moving_interpolation_bound_m"],
        "guard_decisions": dict(guard),
        "plan_messages": s["plan_messages"],
    }
    for name, relative in (("global", "costmap.jsonl"),
                           ("local", "raw_local_maps.jsonl")):
        maps = list(rows(trial / "observation" / relative))
        in_window = [g for g in maps if g["t"] >= 24 and cell_at(g, 4.9, 0) is not None]
        out[name + "_maps"] = {
            "recorded": len(maps), "with_sweep_center_in_window_after_release": len(in_window),
            "sweep_center_lethal_count": sum(cell_at(g, 4.9, 0) in (100, 254)
                                              for g in in_window),
            "center_values": dict(collections.Counter(str(cell_at(g, 4.9, 0)) for g in in_window)),
        }
    return out


def main():
    first = audit_trial(V1)
    second = audit_trial(V2)
    assert first["profile_sha256"] == second["profile_sha256"]
    assert first["profile_matches_input"] and second["profile_matches_input"]
    assert first["global_maps"]["sweep_center_lethal_count"] > 0
    assert second["global_maps"]["sweep_center_lethal_count"] > 0
    result = {
        "schema": "tdt_dynamic_sweep_routing_audit/v1",
        "v1_extra_raster_margin_m": 0.05,
        "v2_extra_raster_margin_m": 0.0,
        "byte_identical_profile": True,
        "v1": first, "v2": second,
        "accepted_for_deployment": False,
    }
    with Path(__file__).with_name("routing_audit.json").open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")


if __name__ == "__main__":
    main()
