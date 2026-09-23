#!/usr/bin/env python3
"""Summarize preserved pilot records without relaxing the planar geometry oracle."""
import collections
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
RUNS = ROOT / "build/tdt_p2b/runs/dynamic_guard_pilot_v1"


def rows(path):
    return [json.loads(line) for line in path.open()]


def vector(row):
    return (row["vx"], row["vy"], row["wz"])


def audit():
    result = {}
    for mode in ("baseline", "guard"):
        trial = RUNS / mode / "tdt_qp_1"
        summary = json.loads((trial / "observation/summary.json").read_text())
        case = {
            "docker_exit": int((trial / "docker_exit.txt").read_text()),
            "action_status": summary["action_status"],
            "timed_out": summary["timed_out"],
            "recoveries": summary["recoveries"],
            "final_pose": summary["final_pose"],
            "final_xy_error_m": summary["final_xy_error_m"],
            "final_yaw_error_rad": summary["final_yaw_error_rad"],
            "preflight_status": summary["preflight"]["status"],
        }
        if mode == "baseline":
            first = None
            maximum = (0.0, None)
            count = 0
            for row in rows(trial / "gazebo_poses.jsonl"):
                by = {p["name"]: p for p in row["pose"]}
                if "rm_sentry_2027" not in by:
                    continue
                q = by["rm_sentry_2027"]["orientation"]
                xy = math.hypot(q.get("x", 0.0), q.get("y", 0.0))
                stamp = row["header"]["stamp"]
                t = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) * 1e-9
                if xy > maximum[0]:
                    maximum = (xy, t)
                if xy > 1e-3:
                    count += 1
                    if first is None:
                        first = (xy, t)
            case["planar_oracle_valid"] = False
            case["nonplanar_pose"] = {
                "threshold_quaternion_xy": 1e-3,
                "first_xy_norm_and_sim_s": first,
                "max_xy_norm_and_sim_s": maximum,
                "samples_over_threshold": count,
            }
            assert first is not None
            assert (trial / "audit_error.txt").exists()
        else:
            dynamic = json.loads((trial / "dynamic_summary.json").read_text())
            geometry = dynamic["geometry"]["new_car_reference"]
            case.update({
                "dynamic_evidence_valid": dynamic["evidence_valid"],
                "dynamic_gate_pass": dynamic["limited_dynamic_geometry_and_goal_pass"],
                "checks": dynamic["checks"],
                "body_moving_interpolation_bound_m": geometry["body"]["moving_interpolation_bound_m"],
                "padded_moving_interpolation_bound_m": geometry["padded"]["moving_interpolation_bound_m"],
            })
            decisions = rows(trial / "guard_decisions.jsonl")
            actual = [x for x in decisions if x["kind"] == "decision"]
            counts = collections.Counter(x["reason"] for x in actual)
            first_reject = next(x for x in actual if x["reason"] == "reject_sweep")
            assert all(x["emitted"] == ([0.0, 0.0, 0.0] if x["reason"] != "admit" else x["proposed"])
                       for x in actual)
            chain = rows(trial / "observation/command_chain.jsonl")
            by_topic = {topic: [x for x in chain if x["topic"] == topic]
                        for topic in ("/cmd_vel", "/cmd_vel_guarded", "/simulation/chassis/cmd_vel")}
            assert all(by_topic.values())
            # ROS callbacks on separate subscriptions have no shared message id.
            # Count equal ordered values, rather than inventing per-message latency.
            downstream = by_topic["/simulation/chassis/cmd_vel"]
            filtered = by_topic["/cmd_vel_guarded"]
            matching = sum(vector(a) == vector(b) for a, b in zip(filtered, downstream))
            case["guard"] = {
                "decisions_by_reason": dict(counts),
                "first_reject_sim_s": first_reject["sim_s"],
                "first_reject_pose": first_reject["pose"],
                "first_reject_body_gap_m": first_reject["certificate"]["body_gap_m"],
                "command_chain_counts": {k: len(v) for k, v in by_topic.items()},
                "filtered_to_chassis_ordered_value_matches": matching,
                "filtered_to_chassis_ordered_comparisons": min(len(filtered), len(downstream)),
            }
        result[mode] = case
    result["accepted_for_deployment"] = False
    out = Path(__file__).with_name("pilot_audit.json")
    with out.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")


if __name__ == "__main__":
    audit()
