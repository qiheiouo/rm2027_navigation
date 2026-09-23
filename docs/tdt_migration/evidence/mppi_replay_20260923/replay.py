#!/usr/bin/env python3
"""Read-only counterfactual replay of the frozen MPPI diagnostic cycles.

The replay uses ideal Omni kinematics, one frozen raw local map per cycle, and
the *observed future* obstacle motion. It is diagnostic, not an executable
braking trajectory or a reproduction of Nav2's raster footprint checker.
"""
import bisect
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
PREV = ROOT / "docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922"
sys.path.insert(0, str(PREV))
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/new_car_geometry_audit_20260917"))
sys.path.insert(0, str(ROOT / "experiments/tdt_planner/rm_tdt_planner/tools"))
from read_trace import read_cycle, last
from dynamic_metrics import rows_from_transport, obstacle_polygon
from audit_geometry import placed
from simulation_geometry import polygon_distance

SERIES = ROOT / "build/tdt_p2b/runs/dynamic_cycle_diagnostic_v2"
OUT = Path(__file__).with_name("replay.json")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interpolate_obstacle(rows, times, t):
    k = bisect.bisect_left(times, t)
    if k == 0 or k == len(times):
        raise ValueError("obstacle time is outside actual observed samples")
    a, b = rows[k - 1], rows[k]
    f = (t - a["t"]) / (b["t"] - a["t"])
    pa, pb = a["obstacle"], b["obstacle"]
    return (pa[0] + f * (pb[0] - pa[0]),
            pa[1] + f * (pb[1] - pa[1]),
            pa[2] + f * math.remainder(pb[2] - pa[2], 2 * math.pi))


def replay_controls(pose, vx, vy, wz, dt, start=1):
    """Apply sequence indices start..end then hold last; first applied is output."""
    assert len(vx) == len(vy) == len(wz) and 0 <= start < len(vx)
    x, y, yaw = pose
    result = []
    for i in range(len(vx)):
        j = min(i + start, len(vx) - 1)
        u, v, w = map(float, (vx[j], vy[j], wz[j]))
        x += dt * (u * math.cos(yaw) - v * math.sin(yaw))
        y += dt * (u * math.sin(yaw) + v * math.cos(yaw))
        yaw += dt * w
        result.append((x, y, yaw))
    return result


def nearest_odom(trajectory, times, t):
    k = bisect.bisect_left(times, t)
    candidates = [trajectory[j] for j in (k - 1, k) if 0 <= j < len(trajectory)]
    return min(candidates, key=lambda row: abs(row["t"] - t))


def first_step_from_twist(pose, twist, dt):
    x, y, yaw = pose
    vx, vy, wz = twist
    return (x + dt * (vx * math.cos(yaw) - vy * math.sin(yaw)),
            y + dt * (vx * math.sin(yaw) + vy * math.cos(yaw)),
            yaw + dt * wz)


def closed_cells(raw, geom):
    ys, xs = np.where((raw == 254) | (raw == 255))
    res = geom["resolution"]
    ox, oy = geom["origin"]
    return [(ox + int(ix) * res, oy + int(iy) * res,
             ox + (int(ix) + 1) * res, oy + (int(iy) + 1) * res)
            for iy, ix in zip(ys, xs)]


def min_raw_gap(polygon, poses, cells):
    # Circle bound avoids applying the expensive exact convex distance to
    # distant cells; all cells remain eligible, so the minimum stays exact.
    rad = max(math.hypot(x, y) for x, y in polygon)
    best = math.inf
    for pose in poses:
        body = placed(polygon, pose)
        x, y, _ = pose
        for x0, y0, x1, y1 in cells:
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            if max(0.0, math.hypot(dx, dy) - rad) >= best:
                continue
            box = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            best = min(best, polygon_distance(body, box))
    return None if math.isinf(best) else best


def min_future_obstacle_gap(polygon, poses, start_t, dt, rows, times, box):
    return min(polygon_distance(placed(polygon, pose),
                               placed(box, interpolate_obstacle(rows, times, start_t + (i + 1) * dt)))
               for i, pose in enumerate(poses))


def trial(name, window):
    d = SERIES / f"{name}_1"
    profile = yaml.safe_load((d / "profile.yaml").read_text())
    params = profile["controller_server"]["ros__parameters"]
    costmap = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = [tuple(p) for p in yaml.safe_load(costmap["footprint"])]
    odom = [json.loads(line) for line in (d / "observation/trajectory.jsonl").read_text().splitlines()]
    odom_times = [r["t"] for r in odom]
    rows = rows_from_transport(d / "gazebo_poses.jsonl")
    times = [r["t"] for r in rows]
    box = obstacle_polygon()
    speed_rows = []
    selected = []
    paths = sorted((d / "mppi_cycles").glob("cycle_*.json"),
                   key=lambda path: int(path.stem.split("_")[1]))
    for path in paths:
        m, arrays = read_cycle(path)
        t = m["sim_ns"] * 1e-9
        near = nearest_odom(odom, odom_times, t)
        measured = [near[k] for k in ("vx", "vy", "wz")]
        dt = float(next(e["value"]["dt"] for e in m["events"] if e["kind"] == "settings"))
        first = first_step_from_twist(m["pose"], measured, dt)
        dx = math.dist(first[:2], m["pose"][:2])
        score_first = [last(arrays, "rollout.x")[:, 0],
                       last(arrays, "rollout.y")[:, 0],
                       last(arrays, "rollout.yaw")[:, 0]]
        stationary = all(np.max(np.abs(a - m["pose"][i])) < 1e-6 for i, a in enumerate(score_first))
        speed_rows.append({"cycle_id": m["cycle_id"], "t": t, "speed_recorded": m["speed"],
                           "nearest_odom_t": near["t"], "nearest_odom_twist": measured,
                           "odom_time_error_s": abs(near["t"] - t),
                           "counterfactual_first_step_m": dx,
                           "scored_rollouts_first_step_stationary": stationary})
        if m["cycle_id"] not in window:
            continue
        cells = closed_cells(last(arrays, "locked.raw_map"), m["map"])
        run = {"cycle_id": m["cycle_id"], "t": t, "raw_closed_cells": len(cells),
               "scored_rollouts_collision_flags": int(np.sum(last(arrays, "cost_critic.collisions"))),
               "returned_command": next(e["value"] for e in m["events"] if e["kind"] == "output")}
        for stage in ("before_filter", "after_filter"):
            vx, vy, wz = (last(arrays, f"{stage}.{field}") for field in ("vx", "vy", "wz"))
            poses = replay_controls(m["pose"], vx, vy, wz, dt)
            assert stage != "after_filter" or np.allclose([vx[1], vy[1], wz[1]], run["returned_command"])
            run[stage] = {
                "first_command": [float(vx[1]), float(vy[1]), float(wz[1])],
                "raw_closed_cell_min_body_gap_m": min_raw_gap(body, poses, cells),
                "observed_future_box_min_body_gap_m": min_future_obstacle_gap(body, poses, t, dt, rows, times, box),
                "first_predicted_pose": poses[0],
            }
        selected.append(run)
    assert len(speed_rows) == len(paths)
    return {
        "profile_sha256": sha(d / "profile.yaml"),
        "controller_server_odom_topic_in_profile": params.get("odom_topic"),
        "cycles": len(speed_rows),
        "zero_speed_cycles": sum(r["speed_recorded"] == [0.0, 0.0, 0.0] for r in speed_rows),
        "cycles_nearest_actual_translation_gt_0p1mps": sum(math.hypot(*r["nearest_odom_twist"][:2]) > .1 for r in speed_rows),
        "max_nearest_odom_time_error_s": max(r["odom_time_error_s"] for r in speed_rows),
        "median_actual_translation_mps": float(np.median([math.hypot(*r["nearest_odom_twist"][:2]) for r in speed_rows])),
        "median_counterfactual_first_step_m": float(np.median([r["counterfactual_first_step_m"] for r in speed_rows])),
        "max_counterfactual_first_step_m": max(r["counterfactual_first_step_m"] for r in speed_rows),
        "all_scored_rollouts_first_step_stationary": all(r["scored_rollouts_first_step_stationary"] for r in speed_rows),
        "window": selected,
        "window_speed_inputs": [r for r in speed_rows if r["cycle_id"] in window],
    }


def main():
    baseline = json.loads((PREV / "cycle_analysis.json").read_text())
    # Frozen report's five-cycle windows, chosen before this replay.
    ids = {}
    for item in baseline["trials"]:
        ids[item["planner"]] = [r["cycle"] for r in item["window_cycles"]]
    result = {
        "schema": "tdt_mppi_offline_replay/v1",
        "source_series": "dynamic_cycle_diagnostic_v2",
        "scope": "diagnostic ideal-kinematic open-loop counterfactual; not executed commands, physical braking, or an exact Nav2 raster critic replay",
        "source_cycle_analysis_sha256": sha(PREV / "cycle_analysis.json"),
        "trials": {name: trial(name, ids[name]) for name in ("tdt_astar", "tdt_qp")},
    }
    with OUT.open("x") as file:
        json.dump(result, file, indent=2)
        file.write("\n")
    for name, item in result["trials"].items():
        print(name, item["cycles"], item["zero_speed_cycles"],
              item["cycles_nearest_actual_translation_gt_0p1mps"],
              item["median_counterfactual_first_step_m"])
        for row in item["window"]:
            print(row["cycle_id"], row["scored_rollouts_collision_flags"],
                  row["before_filter"]["raw_closed_cell_min_body_gap_m"],
                  row["after_filter"]["raw_closed_cell_min_body_gap_m"],
                  row["after_filter"]["observed_future_box_min_body_gap_m"])


if __name__ == "__main__":
    main()
