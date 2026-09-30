#!/usr/bin/env python3
"""Read-only cross-check of frozen face interval on a moving-robot archive."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_tracker_probe_20260924"))
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_extent_shadow_20260924"))
from probe import static_map
from rm_dynamic_obstacle_tracking.core import Point2D, dynamic_candidates
from dynamic_metrics import rows_from_transport
from envelope import predicted_box

ACCELERATION = 0.5551652475612764
SIGMA_RANGE = 0.01
FIRST_BREACH_S = 40.893
WINDOW_START_S = 36.0
WINDOW_END_S = 39.99


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentile(values, p):
    ordered = sorted(values)
    if not ordered:
        return None
    position = p * (len(ordered) - 1)
    index = int(position)
    return ordered[index] + (position - index) * (ordered[min(index + 1, len(ordered) - 1)] - ordered[index])


def describe_errors(values):
    absolute = [abs(value) for value in values]
    return {"n": len(absolute), "median_abs_m": statistics.median(absolute) if absolute else None,
            "p90_abs_m": percentile(absolute, .9),
            "max_abs_m": max(absolute) if absolute else None,
            "within_0_05_m": sum(value <= .05 for value in absolute)}


def interpolate_robot(rows, times, stamp):
    index = bisect.bisect_right(times, stamp)
    if not 0 < index < len(rows):
        return None
    a, b = rows[index - 1], rows[index]
    if b["t"] - a["t"] > .2:
        return None
    fraction = (stamp - a["t"]) / (b["t"] - a["t"])
    return (a["x"] + fraction * (b["x"] - a["x"]),
            a["y"] + fraction * (b["y"] - a["y"]),
            a["yaw"] + fraction * math.remainder(b["yaw"] - a["yaw"], 2 * math.pi))


def interpolate_box(rows, times, stamp):
    index = bisect.bisect_right(times, stamp)
    if not 0 < index < len(rows):
        raise ValueError(f"physical truth does not bracket {stamp}")
    a, b = rows[index - 1], rows[index]
    fraction = (stamp - a["t"]) / (b["t"] - a["t"])
    return (a["obstacle"][0] + fraction * (b["obstacle"][0] - a["obstacle"][0]),
            a["obstacle"][1] + fraction * (b["obstacle"][1] - a["obstacle"][1]))


def scan_points(scan, robot):
    if scan["frame"] != "sim_lidar_link":
        raise ValueError("scan frame changed")
    x, y, yaw = robot
    points = []
    for index, distance in enumerate(scan["ranges"]):
        if not isinstance(distance, (int, float)) or not math.isfinite(distance):
            continue
        if not scan["range_min"] <= distance <= scan["range_max"]:
            continue
        angle = yaw + scan["angle_min"] + index * scan["angle_increment"]
        points.append(Point2D(x + distance * math.cos(angle),
                              y + distance * math.sin(angle)))
    return points


def line_fit(history):
    ts = [row["source_t"] for row in history]
    ys = [row["corrected_y"] for row in history]
    mean_t, mean_y = statistics.mean(ts), statistics.mean(ys)
    sxx = sum((t - mean_t) ** 2 for t in ts)
    vy = sum((t - mean_t) * (y - mean_y) for t, y in zip(ts, ys)) / sxx
    half_velocity = 3 * SIGMA_RANGE / math.sqrt(sxx) + \
        ACCELERATION * (ts[-1] - mean_t)
    return vy, half_velocity


def summarize_steps(rows):
    if not rows:
        return {"source_steps": 0}
    valid = [row for row in rows if row["velocity_valid"]]
    failed = [row for row in valid if not row["interval_y_covers_box"]]
    return {"source_steps": len(rows), "valid_source_steps": len(valid),
            "interval_covered_all": sum(row["interval_y_covers_box"] for row in rows),
            "interval_covered_valid": len(valid) - len(failed),
            "original_v1_covered_valid": sum(row["original_v1_y_covers_box"] for row in valid),
            "interval_width_median_m": statistics.median(
                row["interval_width_m"] for row in valid) if valid else None,
            "original_v1_width_median_m": statistics.median(
                row["original_v1_width_m"] for row in valid) if valid else None,
            "max_interval_excess_m": max((row["interval_excess_m"] for row in valid), default=None),
            "first_interval_miss": ({key: failed[0][key] for key in
                ("source_t", "step", "horizon_s", "interval_excess_m")}
                if failed else None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("phase2_evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    trial, evidence, output = (path.resolve() for path in
                               (args.trial, args.phase2_evidence, args.output))
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip():
        raise ValueError("cross-check plan is not committed")
    evaluation_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    manifest = json.loads((evidence / "manifest.json").read_text())
    files = {"gazebo_poses.jsonl": trial / "gazebo_poses.jsonl",
             "predictions.jsonl": trial / "predictions.jsonl",
             "scans.jsonl": trial / "observation/scans.jsonl",
             "trajectory.jsonl": trial / "observation/trajectory.jsonl"}
    hashes = {}
    for key, path in files.items():
        hashes[key] = digest(path)
        if hashes[key] != manifest["compressed_streams"][key]["source_sha256"]:
            raise ValueError(f"archive changed: {key}")
    occupancy, _ = static_map()
    scan_by_stamp = {round(row["t"], 6): row for row in
                     map(json.loads, files["scans.jsonl"].open())}
    trajectory = [json.loads(line) for line in files["trajectory.jsonl"].open()]
    traj_times = [row["t"] for row in trajectory]
    physical = rows_from_transport(files["gazebo_poses.jsonl"])
    physical_times = [row["t"] for row in physical]
    source_by_stamp = {}
    for row in map(json.loads, files["predictions.jsonl"].open()):
        stamp = row["source_t"]
        if not trajectory[0]["t"] <= stamp <= WINDOW_END_S:
            continue
        confirmed = [track for track in row["tracks"] if track["state"] == 2]
        if len(confirmed) != 1 or not row["complete"] or row["frame"] != "odom":
            continue
        key = round(stamp, 6)
        if key in source_by_stamp:
            raise ValueError(f"duplicate confirmed source stamp: {stamp}")
        source_by_stamp[key] = (row, confirmed[0])
    source_rows = []
    valid_history = []
    for key in sorted(source_by_stamp):
        row, track = source_by_stamp[key]
        stamp = row["source_t"]
        scan = scan_by_stamp.get(key)
        robot = interpolate_robot(trajectory, traj_times, stamp)
        truth_x, truth_y = interpolate_box(physical, physical_times, stamp)
        side_truth = "north" if robot and robot[1] > truth_y else "south"
        side_online = "north" if robot and robot[1] > track["xy"][1] else "south"
        selected = []
        if scan is not None and robot is not None:
            candidates = dynamic_candidates(scan_points(scan, robot), occupancy, .25, True)
            selected = [point for point in candidates if
                        abs(point.x - track["xy"][0]) <= .35 and
                        abs(point.y - track["xy"][1]) <= .50]
        corrected = ((min(p.y for p in selected) + .275) if side_online == "south" else
                     (max(p.y for p in selected) - .275)) if selected else None
        history = None
        if corrected is not None:
            valid_history.append({"source_t": stamp, "corrected_y": corrected})
            if len(valid_history) >= 4:
                last_four = valid_history[-4:]
                if .15 <= last_four[-1]["source_t"] - last_four[0]["source_t"] <= .30:
                    history = line_fit(last_four)
        if WINDOW_START_S <= stamp <= WINDOW_END_S:
            source_rows.append({"source_t": stamp,
                "scan_exact_match": scan is not None,
                "robot_pose_available": robot is not None,
                "selected_points": len(selected),
                "online_side": side_online if robot else None,
                "truth_side": side_truth if robot else None,
                "tracker_x_m": track["xy"][0],
                "tracker_y_m": track["xy"][1],
                "tracker_vy_mps": track["vxy"][1],
                "visible_span_y_m": track["size_xy"][1],
                "physical_x_m": truth_x, "physical_y_m": truth_y,
                "corrected_y_m": corrected,
                "source_y_error_m": corrected - truth_y if corrected is not None else None,
                "tracker_x_error_m": track["xy"][0] - truth_x,
                "causal_vy_mps": history[0] if history else None,
                "velocity_half_width_mps": history[1] if history else None})
    if not source_rows:
        raise ValueError("no confirmed sources in fixed window")
    step_rows = []
    for source in source_rows:
        stamp = source["source_t"]
        for step in range(1, 10):
            horizon = round(.1 * step, 1)
            if stamp + horizon >= FIRST_BREACH_S:
                raise ValueError("future label reaches dynamic clearance breach")
            truth_y = interpolate_box(physical, physical_times, stamp + horizon)[1]
            velocity_valid = source["causal_vy_mps"] is not None
            center = (source["corrected_y_m"] + source["causal_vy_mps"] * horizon
                      if velocity_valid else None)
            vhalf = source["velocity_half_width_mps"]
            margin = .05 + .5 * ACCELERATION * horizon ** 2 + vhalf * horizon if velocity_valid else None
            excess = max(0., abs(center - truth_y) - margin) if velocity_valid else None
            original = predicted_box((0., source["tracker_y_m"]),
                                     (0., source["tracker_vy_mps"]),
                                     (0., source["visible_span_y_m"]),
                                     (.45, .55), 0., horizon, ACCELERATION)
            step_rows.append({"source_t": stamp, "step": step, "horizon_s": horizon,
                "truth_y_m": truth_y, "velocity_valid": velocity_valid,
                "interval_center_y_m": center,
                "interval_excess_m": excess,
                "interval_y_covers_box": excess is not None and excess <= 1e-9,
                "interval_width_m": 2 * (.275 + margin) if velocity_valid else None,
                "original_v1_y_covers_box": original.min_y <= truth_y - .275 + 1e-9 and
                                             original.max_y >= truth_y + .275 - 1e-9,
                "original_v1_width_m": original.max_y - original.min_y})
    output.mkdir(parents=True)
    for name, rows in (("sources.csv", source_rows), ("source_steps.csv", step_rows)):
        with (output / name).open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
    source_errors = [row["source_y_error_m"] for row in source_rows
                     if row["source_y_error_m"] is not None]
    x_errors = [row["tracker_x_error_m"] for row in source_rows]
    result = {"schema": "rm_dynamic_prediction_moving_view_crosscheck/v1",
              "scope": "Already inspected Navfn+V1 phase-2 moving robot; same fixed online near-face rule. Truth only labels pre-breach coverage, not an independent blind test.",
              "evaluation_commit": evaluation_commit,
              "source_run_commit": manifest["source_commit"],
              "source_window_s": [WINDOW_START_S, WINDOW_END_S],
              "first_body_clearance_breach_s": FIRST_BREACH_S,
              "input_sha256": hashes,
              "confirmed_sources": len(source_rows),
              "exact_scan_matches": sum(row["scan_exact_match"] for row in source_rows),
              "online_pose_available": sum(row["robot_pose_available"] for row in source_rows),
              "near_face_point_available": len(source_errors),
              "four_source_velocity_available": sum(row["causal_vy_mps"] is not None for row in source_rows),
              "view_side_mismatches": sum(row["online_side"] != row["truth_side"] for row in source_rows),
              "source_y_error": describe_errors(source_errors),
              "tracker_x_error": describe_errors(x_errors),
              "steps_all": summarize_steps(step_rows),
              "steps_by_horizon": {str(step): summarize_steps([row for row in step_rows
                                                               if row["step"] == step])
                                   for step in range(1, 10)},
              "sources_sha256": digest(output / "sources.csv"),
              "source_steps_sha256": digest(output / "source_steps.csv")}
    result["y_only_prebreach_gate_failed"] = (
        result["near_face_point_available"] < result["confirmed_sources"] or
        result["source_y_error"]["within_0_05_m"] < len(source_errors) or
        result["steps_all"]["interval_covered_all"] <
        result["steps_all"]["source_steps"])
    (output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: result[key] for key in (
        "confirmed_sources", "exact_scan_matches", "online_pose_available",
        "near_face_point_available", "four_source_velocity_available",
        "view_side_mismatches", "source_y_error", "tracker_x_error",
        "steps_all", "y_only_prebreach_gate_failed")}, indent=2))


if __name__ == "__main__":
    main()
