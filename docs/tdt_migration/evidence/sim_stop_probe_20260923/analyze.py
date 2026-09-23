#!/usr/bin/env python3
"""Audit the new no-navigation command-to-odom step response."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
RUN = ROOT / "build/tdt_p2b/runs/sim_stop_probe_v2"
AXES = {"lateral_stop": 1, "longitudinal_stop": 0, "yaw_stop": 2}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(rows, name, axis):
    starts = [r for r in rows if r["kind"] == "phase" and r["name"] == name]
    assert len(starts) == 1
    phase_t = starts[0]["callback_sim_s"]
    sent = next(r for r in rows if r["kind"] == "sent" and r["name"] == name)
    relay = next(r for r in rows if r["kind"] == "relay" and
                 r["callback_sim_s"] >= sent["callback_sim_s"] and
                 abs(r["velocity"][axis]) < 1e-9)
    old = [r for r in rows if r["kind"] == "odom" and r["stamp_s"] <= relay["callback_sim_s"]]
    assert old
    initial_speed = abs(old[-1]["velocity"][axis])
    assert initial_speed >= 0.35, (name, initial_speed)
    response = [r for r in rows if r["kind"] == "odom" and
                relay["callback_sim_s"] <= r["stamp_s"] <= phase_t + 1.0]
    assert response
    first_drop = next((r for r in response if
                       abs(r["velocity"][axis]) < initial_speed - 0.02), None)
    zero = next((r for r in response if
                 abs(r["velocity"][axis]) <= 0.01), None)
    assert first_drop and zero
    assert response[0]["stamp_s"] <= first_drop["stamp_s"] <= zero["stamp_s"]
    around = [old[-1], *[r for r in response if r["stamp_s"] <= zero["stamp_s"]]]
    distance = sum((abs(a["velocity"][axis]) + abs(b["velocity"][axis])) / 2 *
                   (b["stamp_s"] - a["stamp_s"]) for a, b in zip(around, around[1:]))
    active = [r for r in around if
              0.05 < abs(r["velocity"][axis]) < initial_speed - 0.02]
    slopes = [(abs(a["velocity"][axis]) - abs(b["velocity"][axis])) /
              (b["stamp_s"] - a["stamp_s"]) for a, b in zip(active, active[1:])]
    return {"name": name, "axis": axis, "initial_speed": initial_speed,
            "first_zero_sent_sim_s": sent["callback_sim_s"],
            "first_zero_relay_sim_s": relay["callback_sim_s"],
            "first_drop_odom_stamp_s": first_drop["stamp_s"],
            "first_at_or_below_001_odom_stamp_s": zero["stamp_s"],
            "relay_to_first_drop_s": first_drop["stamp_s"] - relay["callback_sim_s"],
            "relay_to_near_zero_s": zero["stamp_s"] - relay["callback_sim_s"],
            "sampled_axis_motion_after_relay": distance,
            "intermediate_deceleration_per_s": slopes,
            "sample_count": len(response)}


def main():
    rows = [json.loads(line) for line in (RUN / "records.jsonl").open()]
    assert [r["name"] for r in rows if r["kind"] == "phase"] == [
        "settle", "lateral_drive", "lateral_stop", "longitudinal_drive",
        "longitudinal_stop", "yaw_drive", "yaw_stop"]
    assert sum(r["kind"] == "complete" for r in rows) == 1
    assert all(math.isfinite(x) for r in rows if "velocity" in r for x in r["velocity"])
    assert (RUN / "container_exit.txt").read_text() == "0\n"
    result = {"schema": "tdt_direct_sim_stop_probe/v1",
              "scope": "one no-navigation direct /cmd_vel step per axis; neither velocity_smoother nor Nav2, not a certified braking lower bound",
              "records_sha256": sha(RUN / "records.jsonl"),
              "robot_sdf_sha256": sha(ROOT / "src/rm_simulation/worlds/phase1_omni.sdf"),
              "chassis_stub_sha256": sha(ROOT / "src/rm_chassis_interface/src/chassis_interface_stub.cpp"),
              "trials": [audit(rows, name, axis) for name, axis in AXES.items()]}
    with (HERE / "result.json").open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    for trial in result["trials"]:
        print(trial["name"], "speed", round(trial["initial_speed"], 4),
              "relay-to-near-zero", round(trial["relay_to_near_zero_s"], 4),
              "motion", round(trial["sampled_axis_motion_after_relay"], 4))


if __name__ == "__main__":
    main()
