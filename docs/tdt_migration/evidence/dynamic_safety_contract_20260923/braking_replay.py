#!/usr/bin/env python3
"""Hindsight-only assumed braking and actual moving-box polygon replay."""
import bisect
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
RUNS = ROOT / "build/tdt_p2b/runs/dynamic_map_age_pilot_v1"
sys.path[:0] = [str(HERE.parent / "dynamic_reference_20260922"),
                str(HERE.parent / "new_car_geometry_audit_20260917")]
from dynamic_metrics import obstacle_polygon, rows_from_transport
from audit_geometry import padded, placed, radius
from simulation_geometry import polygon_distance

HORIZON = 2.0
STEP = 0.01
DELAYS = (0.0, 0.1, 0.2)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def obstacle_at(rows, times, t):
    i = bisect.bisect_left(times, t)
    if i == 0 and t == times[0]:
        return rows[0]["obstacle"]
    if i == 0 or i == len(rows):
        raise ValueError("future obstacle sample missing")
    a, b = rows[i - 1], rows[i]
    f = (t - a["t"]) / (b["t"] - a["t"])
    pa, pb = a["obstacle"], b["obstacle"]
    return (pa[0] + f * (pb[0] - pa[0]), pa[1] + f * (pb[1] - pa[1]),
            pa[2] + f * math.remainder(pb[2] - pa[2], 2 * math.pi))


def slowing(v, t, delay, decel):
    if decel <= 0 or delay < 0:
        raise ValueError("invalid deceleration or delay")
    return math.copysign(max(0.0, abs(v) - decel * max(0.0, t - delay)), v)


def braking_poses(pose, speed, delay, horizon=HORIZON, step=STEP):
    """Ideal body-frame velocity with configured 1/1/2 deceleration."""
    if horizon <= 0 or step <= 0 or delay < 0:
        raise ValueError("invalid integration window")
    x, y, yaw = map(float, pose)
    output = [(x, y, yaw)]
    t = 0.0
    while t < horizon - 1e-12:
        dt = min(step, horizon - t)
        mid = t + dt / 2
        vx = slowing(speed[0], mid, delay, 1.0)
        vy = slowing(speed[1], mid, delay, 1.0)
        wz = slowing(speed[2], mid, delay, 2.0)
        heading = yaw + wz * dt / 2
        x += dt * (vx * math.cos(heading) - vy * math.sin(heading))
        y += dt * (vx * math.sin(heading) + vy * math.cos(heading))
        yaw += dt * wz
        output.append((x, y, yaw))
        t += dt
    return output


def obstacle_speed_bound(rows, times, start, end, box_radius):
    if start < times[0] or end > times[-1]:
        raise ValueError("observed obstacle interval incomplete")
    first = max(0, bisect.bisect_right(times, start) - 1)
    last = min(len(rows) - 1, bisect.bisect_left(times, end))
    bound = 0.0
    for i in range(first, last):
        a, b = rows[i], rows[i + 1]
        pa, pb = a["obstacle"], b["obstacle"]
        motion = math.dist(pa[:2], pb[:2]) + box_radius * abs(math.remainder(pb[2] - pa[2], 2 * math.pi))
        bound = max(bound, motion / (b["t"] - a["t"]))
    return bound


def check_brake(rows, times, t, pose, speed, body, box, delay):
    poses = braking_poses(pose, speed, delay)
    box_speed = obstacle_speed_bound(rows, times, t, t + HORIZON, radius(box))
    robot_speed = math.hypot(*speed[:2])
    result = {}
    for name, polygon, gate in (("body", body, 0.05),
                                ("padded", padded(body, 0.03), 0.0)):
        gaps = [polygon_distance(placed(polygon, robot),
                                 placed(box, obstacle_at(rows, times, t + i * STEP)))
                for i, robot in enumerate(poses)]
        # Distance is Lipschitz under translation and rotation. A nearest
        # sample is at most half a step away. Applies to this assumed model.
        lower = min(gaps) - STEP / 2 * (robot_speed + radius(polygon) * abs(speed[2]) + box_speed)
        result[name] = {"sample_min_m": min(gaps), "continuous_bound_m": lower,
                        "passes_assumed_model": lower >= gate if gate else lower > gate}
    return {"delay_s": delay, "observed_future_box_speed_max_m_s": box_speed,
            "body": result["body"], "padded": result["padded"],
            "assumed_braking_gate_pass": (result["body"]["passes_assumed_model"]
                                          and result["padded"]["passes_assumed_model"]),
            "assumed_stop_pose": poses[-1]}


def trial(name, first_violation, body, box):
    d = RUNS / f"{name}_1"
    rows = rows_from_transport(d / "gazebo_poses.jsonl")
    times = [r["t"] for r in rows]
    cycles = []
    for path in sorted((d / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        m = json.loads(path.read_text())
        t = m["map_sim_ns"] * 1e-9
        if first_violation - 1.5 <= t <= first_violation + 0.1 and not m["unwinding_exception"]:
            cycles.append({
                "cycle_id": m["cycle_id"], "sim_s": t,
                "relative_to_first_violation_s": t - first_violation,
                "pose": m["pose"], "odom_speed": m["speed"],
                "actual_box_gap_at_decision_m": polygon_distance(
                    placed(body, m["pose"]), placed(box, obstacle_at(rows, times, t))),
                "cases": [check_brake(rows, times, t, m["pose"], m["speed"], body, box, delay)
                          for delay in DELAYS],
            })
    assert cycles
    return {"planner": name, "profile_sha256": sha(d / "profile.yaml"),
            "gazebo_poses_sha256": sha(d / "gazebo_poses.jsonl"),
            "first_body_gap_violation_sim_s": first_violation, "cycles": cycles}


def main():
    source = HERE.parent / "map_age_diagnostic_20260923/map_age_analysis_v2.json"
    previous = json.loads(source.read_text())
    inputs = json.loads((RUNS / "inputs.json").read_text())
    body = [tuple(p) for p in inputs["body_polygon_m"]]
    box = obstacle_polygon()
    result = {
        "schema": "tdt_dynamic_braking_hindsight/v1",
        "source_series": str(RUNS.relative_to(ROOT)),
        "source_map_analysis_sha256": sha(source),
        "scope": "actual future box poses and ideal configured smoother decel; not online prediction or verified chassis braking",
        "assumptions": {"horizon_s": HORIZON, "step_s": STEP, "response_delays_s": DELAYS,
                        "linear_deceleration_per_axis_m_s2": 1.0,
                        "yaw_deceleration_rad_s2": 2.0, "body_gate_m": 0.05,
                        "padding_m": 0.03},
        "trials": [trial(r["planner"], r["first_body_below_005_sim_s"], body, box)
                   for r in previous["trials"]],
    }
    with (HERE / "braking_replay.json").open("x") as f:
        json.dump(result, f, indent=2)
        f.write("\n")
    for item in result["trials"]:
        print(item["planner"], len(item["cycles"]))
        for delay in DELAYS:
            good = [r for r in item["cycles"]
                    if next(c for c in r["cases"] if c["delay_s"] == delay)["assumed_braking_gate_pass"]]
            print("delay", delay, "passing", len(good),
                  "last_cycle", good[-1]["cycle_id"] if good else None)


if __name__ == "__main__":
    main()
