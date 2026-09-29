#!/usr/bin/env python3
"""Audit one frozen paired-view capture; Gazebo is an offline label only."""
import argparse
import bisect
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
from dynamic_metrics import obstacle_polygon, placed, polygon_distance, radius, rows_from_transport, travel

PROFILE = ROOT / "docs/dynamic_navigation/evidence/phase2_holdout_20260929/profile.yaml"
RAW_FILES = ("gazebo_poses.jsonl", "predictions.jsonl", "diagnostics.jsonl",
             "moving_target.jsonl", "scans.jsonl", "odometry.jsonl",
             "observation_summary.json", "docker_exit.txt", "launch.log",
             "tracker.log", "observer.log")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = percent * (len(ordered) - 1) / 100
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (index - lower) * (ordered[upper] - ordered[lower])


def describe(values):
    if not values:
        return {"n": 0}
    return {"n": len(values), "median_m": statistics.median(values),
            "p10_m": percentile(values, 10), "p90_m": percentile(values, 90),
            "min_m": min(values), "max_m": max(values)}


def at(rows, times, t):
    index = bisect.bisect_right(times, t)
    if not 0 < index < len(rows):
        raise ValueError(f"physical pose does not bracket {t:.6f}")
    a, b = rows[index - 1], rows[index]
    weight = (t - a["t"]) / (b["t"] - a["t"])
    return {key: tuple(a[key][j] + weight * (b[key][j] - a[key][j])
                       for j in range(3)) for key in ("robot", "obstacle")}


def geometry(poses, body, box):
    gaps = [polygon_distance(placed(body, row["robot"]),
                             placed(box, row["obstacle"])) for row in poses]
    body_radius, box_radius = radius(body), radius(box)
    lower = min(gaps)
    for index, (a, b) in enumerate(zip(poses, poses[1:])):
        relative_travel = travel(a["robot"], b["robot"], body_radius) + \
            travel(a["obstacle"], b["obstacle"], box_radius)
        lower = min(lower, min(gaps[index:index + 2]) - 0.5 * relative_travel)
    return {"sample_min_m": min(gaps), "interpolation_lower_bound_m": lower,
            "at_least_0_05_m": lower >= 0.05,
            "box_y_range_m": [min(p["obstacle"][1] for p in poses),
                              max(p["obstacle"][1] for p in poses)],
            "robot_y_range_m": [min(p["robot"][1] for p in poses),
                                max(p["robot"][1] for p in poses)]}


def audit_side(path, side, body, box):
    observation = json.loads((path / "observation_summary.json").read_text())
    if observation["actual_stop_sim_s"] < 44 or (path / "docker_exit.txt").read_text().strip() != "0":
        raise ValueError(f"incomplete fixed observation: {side}")
    poses = rows_from_transport(path / "gazebo_poses.jsonl")
    times = [row["t"] for row in poses]
    seen = {}
    source_messages = 0
    rejected_multiple_confirmed = 0
    for line in (path / "predictions.jsonl").open():
        item = json.loads(line)
        stamp = item["source_t"]
        if not 16 <= stamp <= 43.9:
            continue  # Keep +/-0.1 s physical-velocity stencil inside truth.
        source_messages += 1
        confirmed = [track for track in item["tracks"] if track["state"] == 2]
        if len(confirmed) != 1 or not item["complete"] or item["frame"] != "odom":
            rejected_multiple_confirmed += 1
            continue
        track = confirmed[0]
        signature = (track["xy"], track["vxy"], track["size_xy"])
        if stamp in seen:
            if signature != seen[stamp]["signature"]:
                raise ValueError(f"same source stamp has different tracker state: {side} {stamp}")
            seen[stamp]["repeat_count"] += 1
            continue
        physical = at(poses, times, stamp)
        y = physical["obstacle"][1]
        robot_y = physical["robot"][1]
        velocity = (at(poses, times, stamp + 0.1)["obstacle"][1] -
                    at(poses, times, stamp - 0.1)["obstacle"][1]) / 0.2
        tracker_y = track["xy"][1]
        if math.hypot(track["xy"][0] - physical["obstacle"][0], tracker_y - y) > 1:
            raise ValueError(f"confirmed track is not near physical box: {side} {stamp}")
        seen[stamp] = {"source_t": stamp, "tracker_x_m": track["xy"][0],
                       "tracker_y_m": tracker_y, "tracker_vy_mps": track["vxy"][1],
                       "visible_span_y_m": track["size_xy"][1],
                       "physical_box_y_m": y, "physical_vy_mps": velocity,
                       "robot_y_m": robot_y,
                       "tracker_minus_box_y_m": tracker_y - y,
                       "signed_visible_bias_m": (tracker_y - y) * (1 if side == "north" else -1),
                       "online_view_side_from_fixed_pose_and_track":
                           "north" if (1.65 if side == "north" else -1.65) > tracker_y else "south",
                       "truth_view_side": "north" if robot_y > y else "south",
                       "repeat_count": 1, "signature": signature}
    details = [seen[t] for t in sorted(seen)]
    for row in details:
        del row["signature"]
    narrow = [row for row in details if row["visible_span_y_m"] < 0.275]
    wide = [row for row in details if row["visible_span_y_m"] >= 0.275]
    phases = {"positive_gt_0_3": sum(row["physical_vy_mps"] > 0.3 for row in details),
              "negative_lt_minus_0_3": sum(row["physical_vy_mps"] < -0.3 for row in details),
              "turn_abs_le_0_2": sum(abs(row["physical_vy_mps"]) <= 0.2 for row in details)}
    physical_gap = geometry(poses, body, box)
    view_mismatches = sum(row["online_view_side_from_fixed_pose_and_track"] !=
                          row["truth_view_side"] for row in details)
    summary = {"side": side, "observation": observation,
               "physical_pose_rows": len(poses), "source_messages_in_window": source_messages,
               "rejected_non_unique_or_unconfirmed_messages": rejected_multiple_confirmed,
               "unique_confirmed_source_stamps": len(details),
               "repeated_source_messages": sum(row["repeat_count"] - 1 for row in details),
               "view_side_mismatches": view_mismatches,
               "signed_bias": describe([row["signed_visible_bias_m"] for row in details]),
               "raw_tracker_minus_box_y": describe([row["tracker_minus_box_y_m"] for row in details]),
               "narrow_signed_bias": describe([row["signed_visible_bias_m"] for row in narrow]),
               "wide_signed_bias": describe([row["signed_visible_bias_m"] for row in wide]),
               "physical_velocity_coverage": phases, "physical_body_gap": physical_gap}
    summary["track_geometry_gate"] = (len(details) >= 40 and
        all(value > 0 for value in phases.values()) and view_mismatches == 0 and
        physical_gap["at_least_0_05_m"] and
        summary["signed_bias"]["median_m"] > 0.10)
    summary["full_preregistered_coverage_gate"] = (summary["track_geometry_gate"] and
        observation["scan_count"] > 0 and len(poses) > 0)
    return summary, details


def compress(source, target):
    with source.open("rb") as inp, target.open("xb") as out:
        with gzip.GzipFile(fileobj=out, filename="", mode="wb", mtime=0) as zipped:
            while chunk := inp.read(1024 * 1024):
                zipped.write(chunk)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    series, output = args.series.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    plan = json.loads((series / "plan.json").read_text())
    if plan["schema"] != "rm_dynamic_prediction_paired_view/v1":
        raise ValueError("wrong frozen capture plan")
    profile = yaml.safe_load(PROFILE.read_text())
    body = [tuple(vertex) for vertex in yaml.safe_load(
        profile["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])]
    box = obstacle_polygon()
    output.mkdir(parents=True)
    (output / "plan.json").write_bytes((series / "plan.json").read_bytes())
    manifest = {"schema": "rm_dynamic_prediction_paired_view_evidence/v1",
                "source_root": str(series), "run_commit": plan["run_commit"],
                "profile_sha256": digest(PROFILE),
                "body_footprint_xy_m": body, "physical_box_xy_m": box,
                "source_sha256": {}, "packaged_sha256": {}}
    results = {}
    for side in ("south", "north"):
        trial = series / side
        result, details = audit_side(trial, side, body, box)
        results[side] = result
        with (output / f"{side}_sources.csv").open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(details[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(details)
        for name in RAW_FILES:
            source = trial / name
            if not source.exists():
                raise FileNotFoundError(source)
            key = f"{side}/{name}"
            manifest["source_sha256"][key] = digest(source)
            archive = output / f"{side}_{name}.gz"
            compress(source, archive)
            manifest["packaged_sha256"][archive.name] = digest(archive)
    report = {"schema": "rm_dynamic_prediction_paired_view_audit/v1",
              "scope": "Descriptive two-run, same-box input audit. Gazebo and physical velocity are offline labels; no prediction model was fit. Adjacent frames are correlated.",
              "plan_sha256": digest(series / "plan.json"),
              "profile_sha256": digest(PROFILE),
              "results": results,
              "pair_track_geometry_gate": all(result["track_geometry_gate"] for result in results.values()),
              "pair_full_preregistered_coverage_gate": all(result["full_preregistered_coverage_gate"] for result in results.values())}
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    for name in ("plan.json", "summary.json", "south_sources.csv", "north_sources.csv"):
        manifest["packaged_sha256"][name] = digest(output / name)
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
