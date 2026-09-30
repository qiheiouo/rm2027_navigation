#!/usr/bin/env python3
"""Read-only near-face scan geometry probe on one frozen paired capture."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[3]
PROBE_DIR = ROOT / "docs/dynamic_navigation/evidence/v1_tracker_probe_20260924"
sys.path.insert(0, str(PROBE_DIR))
from probe import static_map
from rm_dynamic_obstacle_tracking.core import Point2D, dynamic_candidates

CORE = ROOT / "src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py"
HALF_BOX_HEIGHT_M = 0.275
GATE_X_M = 0.35
GATE_Y_M = 0.50


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quantile(values, q):
    ordered = sorted(values)
    if not ordered:
        return None
    i = q * (len(ordered) - 1)
    lower = int(i)
    return ordered[lower] + (i - lower) * (ordered[min(lower + 1, len(ordered) - 1)] - ordered[lower])


def error_stats(rows, name):
    errors = [abs(row[name]) for row in rows if row[name] is not None]
    return {"valid_sources": len(errors), "missing_sources": len(rows) - len(errors),
            "median_abs_error_m": statistics.median(errors) if errors else None,
            "p90_abs_error_m": quantile(errors, 0.9),
            "max_abs_error_m": max(errors) if errors else None,
            "within_0_05_m": sum(value <= 0.05 for value in errors)}


def odom_at(rows, times, stamp):
    index = bisect.bisect_right(times, stamp)
    if not 0 < index < len(rows):
        raise ValueError(f"odometry does not bracket scan stamp {stamp}")
    a, b = rows[index - 1], rows[index]
    if b["t"] - a["t"] > 0.2:
        raise ValueError(f"online odometry gap at {stamp}")
    f = (stamp - a["t"]) / (b["t"] - a["t"])
    x = a["x"] + f * (b["x"] - a["x"])
    y = a["y"] + f * (b["y"] - a["y"])
    # Both poses are planar and stationary in this frozen fixture.
    yaw_a = math.atan2(2 * a["yaw_w"] * a["yaw_z"], 1 - 2 * a["yaw_z"] ** 2)
    yaw_b = math.atan2(2 * b["yaw_w"] * b["yaw_z"], 1 - 2 * b["yaw_z"] ** 2)
    yaw = yaw_a + f * math.remainder(yaw_b - yaw_a, 2 * math.pi)
    return x, y, yaw


def scan_points(scan, robot):
    if scan["frame"] != "sim_lidar_link":
        raise ValueError("unexpected scan frame")
    x, y, yaw = robot
    points = []
    for index, distance in enumerate(scan["ranges"]):
        if not isinstance(distance, (int, float)) or not math.isfinite(distance):
            continue
        if not scan["range_min"] <= distance <= scan["range_max"]:
            continue
        theta = yaw + scan["angle_min"] + index * scan["angle_increment"]
        points.append(Point2D(x + distance * math.cos(theta),
                              y + distance * math.sin(theta)))
    return points


def phase(velocity):
    if velocity > 0.3:
        return "positive"
    if velocity < -0.3:
        return "negative"
    if abs(velocity) <= 0.2:
        return "turn"
    return "other"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series", type=Path)
    parser.add_argument("input_evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    series, evidence, output = (path.resolve() for path in
                                (args.series, args.input_evidence, args.output))
    if output.exists():
        raise FileExistsError(output)
    plan = json.loads((series / "plan.json").read_text())
    manifest = json.loads((evidence / "manifest.json").read_text())
    if plan["view_set"] != "offset" or plan["run_commit"] != manifest["run_commit"]:
        raise ValueError("wrong frozen offset capture")
    if digest(ROOT / "src/rm_simulation/worlds/phase1_omni.sdf") != plan["file_sha256"]["source_world"]:
        raise ValueError("static map source changed")
    occupancy, _ = static_map()
    output.mkdir(parents=True)
    all_rows = []
    input_hashes = {}
    for side in ("south", "north"):
        trial = series / side
        for name in ("scans.jsonl", "odometry.jsonl"):
            path = trial / name
            key = f"{side}/{name}"
            input_hashes[key] = digest(path)
            if input_hashes[key] != manifest["source_sha256"][key]:
                raise ValueError(f"raw input changed: {key}")
        source_file = evidence / f"{side}_sources.csv"
        input_hashes[source_file.name] = digest(source_file)
        if input_hashes[source_file.name] != manifest["packaged_sha256"][source_file.name]:
            raise ValueError(f"source table changed: {side}")
        scans = {round(scan["t"], 6): scan for scan in
                 map(json.loads, (trial / "scans.jsonl").open())}
        odometry = [json.loads(line) for line in (trial / "odometry.jsonl").open()]
        odom_times = [row["t"] for row in odometry]
        with source_file.open() as stream:
            sources = list(csv.DictReader(stream))
        for item in sources:
            stamp = float(item["source_t"])
            scan = scans.get(round(stamp, 6))
            if scan is None:
                raise ValueError(f"missing exact source scan: {side} {stamp}")
            robot = odom_at(odometry, odom_times, stamp)
            tracker_x = float(item["tracker_x_m"])
            tracker_y = float(item["tracker_y_m"])
            true_y = float(item["physical_box_y_m"])
            side_from_online = "north" if robot[1] > tracker_y else "south"
            if side_from_online != side:
                raise ValueError(f"online view side changed: {side} {stamp}")
            candidates = dynamic_candidates(scan_points(scan, robot), occupancy,
                                            0.25, True)
            selected = [point for point in candidates
                        if abs(point.x - tracker_x) <= GATE_X_M and
                           abs(point.y - tracker_y) <= GATE_Y_M]
            near_y = (min(point.y for point in selected) if side == "south" else
                      max(point.y for point in selected)) if selected else None
            center_y = (near_y + HALF_BOX_HEIGHT_M * (1 if side == "south" else -1)
                        if near_y is not None else None)
            row = {"side": side, "source_t": stamp,
                   "physical_phase": phase(float(item["physical_vy_mps"])),
                   "online_robot_y_m": robot[1], "tracker_y_m": tracker_y,
                   "physical_box_y_m": true_y,
                   "dynamic_candidate_count": len(candidates),
                   "selected_point_count": len(selected),
                   "near_scan_y_m": near_y, "near_face_center_y_m": center_y,
                   "near_face_error_y_m": center_y - true_y if center_y is not None else None,
                   "tracker_error_y_m": tracker_y - true_y}
            all_rows.append(row)
    with (output / "sources.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(all_rows)
    groups = {}
    for side in ("south", "north"):
        by_side = [row for row in all_rows if row["side"] == side]
        groups[side] = {"all": {"near_face": error_stats(by_side, "near_face_error_y_m"),
                                "tracker": error_stats(by_side, "tracker_error_y_m")}}
        for label in ("positive", "negative", "turn", "other"):
            subset = [row for row in by_side if row["physical_phase"] == label]
            groups[side][label] = {"near_face": error_stats(subset, "near_face_error_y_m"),
                                   "tracker": error_stats(subset, "tracker_error_y_m")}
    result = {"schema": "rm_dynamic_prediction_visible_face_probe/v1",
              "scope": "Exploratory source-time Y-only geometry; current truth already inspected. No fitted parameters, future prediction, MPPI, or deployment claim.",
              "run_commit": plan["run_commit"], "core_sha256": digest(CORE),
              "static_map_source_sha256": plan["file_sha256"]["source_world"],
              "input_sha256": input_hashes, "rule": {
                  "dynamic_static_distance_m": 0.25, "gate_x_m": GATE_X_M,
                  "gate_y_m": GATE_Y_M, "known_half_box_height_m": HALF_BOX_HEIGHT_M},
              "groups": groups,
              "sampled_y_source_gate_failed": any(
                  groups[side]["all"]["near_face"]["missing_sources"] > 0 or
                  groups[side]["all"]["near_face"]["max_abs_error_m"] > 0.05
                  for side in ("south", "north")),
              "details_sha256": digest(output / "sources.csv")}
    (output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
