#!/usr/bin/env python3
"""Audit the separate no-navigation smoother-to-Gazebo response."""
import hashlib
import json
from pathlib import Path

from analyze import AXES, HERE, ROOT, sha

RUN = ROOT / "build/tdt_p2b/runs/sim_stop_probe_v3"


def main():
    rows = [json.loads(line) for line in (RUN / "records.jsonl").open()]
    assert sum(r["kind"] == "complete" for r in rows) == 1
    assert (RUN / "container_exit.txt").read_text() == "0\n"
    assert "Transitioning successful" in (RUN / "lifecycle_activate.txt").read_text()
    trials = []
    for name, axis in AXES.items():
        sent = next(r for r in rows if r["kind"] == "sent" and r["name"] == name)
        sent_t = sent["callback_sim_s"]
        old = [r for r in rows if r["kind"] == "odom" and r["stamp_s"] <= sent_t]
        assert old
        initial_speed = abs(old[-1]["velocity"][axis])
        assert initial_speed >= 0.35, (name, initial_speed)
        smoothed = [r for r in rows if r["kind"] == "smoothed" and r["callback_sim_s"] >= sent_t]
        first_smoothed_drop = next(r for r in smoothed if abs(r["velocity"][axis]) < initial_speed - 0.02)
        first_smoothed_zero = next(r for r in smoothed if abs(r["velocity"][axis]) <= 0.01)
        odom = [r for r in rows if r["kind"] == "odom" and r["stamp_s"] >= sent_t]
        first_odom_drop = next(r for r in odom if abs(r["velocity"][axis]) < initial_speed - 0.02)
        first_odom_zero = next(r for r in odom if abs(r["velocity"][axis]) <= 0.01)
        path = [old[-1], *[r for r in odom if r["stamp_s"] <= first_odom_zero["stamp_s"]]]
        motion = sum((abs(a["velocity"][axis]) + abs(b["velocity"][axis])) / 2 *
                     (b["stamp_s"] - a["stamp_s"]) for a, b in zip(path, path[1:]))
        relay = next(r for r in rows if r["kind"] == "relay" and
                     r["callback_sim_s"] >= first_smoothed_zero["callback_sim_s"] and
                     abs(r["velocity"][axis]) <= 0.01)
        trials.append({"name": name, "axis": axis, "initial_speed": initial_speed,
                       "first_zero_sent_sim_s": sent_t,
                       "first_smoothed_drop_sim_s": first_smoothed_drop["callback_sim_s"],
                       "first_smoothed_near_zero_sim_s": first_smoothed_zero["callback_sim_s"],
                       "first_relay_near_zero_sim_s": relay["callback_sim_s"],
                       "first_odom_drop_stamp_s": first_odom_drop["stamp_s"],
                       "first_odom_near_zero_stamp_s": first_odom_zero["stamp_s"],
                       "sent_to_smoothed_drop_s": first_smoothed_drop["callback_sim_s"] - sent_t,
                       "sent_to_smoothed_near_zero_s": first_smoothed_zero["callback_sim_s"] - sent_t,
                       "sent_to_odom_near_zero_s": first_odom_zero["stamp_s"] - sent_t,
                       "sampled_axis_motion_after_zero_sent": motion})
    result = {"schema": "tdt_smoother_sim_stop_probe/v1",
              "scope": "one no-navigation /cmd_vel_nav step per axis through OPEN_LOOP velocity_smoother and simulator; not a certified worst-case response",
              "records_sha256": sha(RUN / "records.jsonl"),
              "profile_sha256": sha(ROOT / "build/tdt_p2b/runs/dynamic_map_age_pilot_v1/tdt_astar_1/profile.yaml"),
              "robot_sdf_sha256": sha(ROOT / "src/rm_simulation/worlds/phase1_omni.sdf"),
              "trials": trials}
    with (HERE / "smoother_result.json").open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    for row in trials:
        print(row["name"], "sent-to-odom-near-zero",
              round(row["sent_to_odom_near_zero_s"], 4), "s",
              "sampled-axis-motion-after-zero-sent",
              round(row["sampled_axis_motion_after_zero_sent"], 4))


if __name__ == "__main__":
    main()
