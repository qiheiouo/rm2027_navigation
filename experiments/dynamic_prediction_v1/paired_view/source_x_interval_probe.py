#!/usr/bin/env python3
"""Check a scan-derived physical center X interval in frozen captures."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_tracker_probe_20260924"))
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
from probe import static_map
from rm_dynamic_obstacle_tracking.core import dynamic_candidates
from dynamic_metrics import rows_from_transport
from visible_face_probe import odom_at, scan_points
from moving_view_crosscheck import interpolate_robot, interpolate_box

HALF_BOX_WIDTH = .225
POINT_ALLOWANCE = .05


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def confirmed_predictions(path):
    found = {}
    for item in read_rows(path):
        tracks = [track for track in item["tracks"] if track["state"] == 2]
        if len(tracks) != 1 or not item["complete"]:
            continue
        key = round(item["source_t"], 6)
        if key in found:
            raise ValueError(f"duplicate confirmed prediction source: {key}")
        found[key] = tracks[0]
    return found


def check_one(group, stamp, scan, robot, track, truth_x, occupancy):
    points = dynamic_candidates(scan_points(scan, robot), occupancy, .25, True)
    selected = [point for point in points if
                abs(point.x - track["xy"][0]) <= .35 and
                abs(point.y - track["xy"][1]) <= .50]
    minimum = min((p.x for p in selected), default=None)
    maximum = max((p.x for p in selected), default=None)
    lower = maximum - HALF_BOX_WIDTH - POINT_ALLOWANCE if selected else None
    upper = minimum + HALF_BOX_WIDTH + POINT_ALLOWANCE if selected else None
    valid = lower is not None and lower <= upper
    covered = valid and lower <= truth_x + 1e-9 and truth_x <= upper + 1e-9
    return {"group": group, "source_t": stamp,
            "selected_point_count": len(selected), "selected_min_x_m": minimum,
            "selected_max_x_m": maximum, "center_lower_x_m": lower,
            "center_upper_x_m": upper, "valid_interval": valid,
            "physical_center_x_m": truth_x,
            "physical_center_covered": covered,
            "full_physical_x_union_width_m": upper - lower + 2 * HALF_BOX_WIDTH if valid else None,
            "tracker_center_x_error_m": track["xy"][0] - truth_x,
            "original_v1_x_width_m": track["size_xy"][0] + .90}


def summarize(rows):
    valid = [row for row in rows if row["valid_interval"]]
    missed = [row for row in rows if not row["physical_center_covered"]]
    errors = [abs(row["tracker_center_x_error_m"]) for row in rows]
    return {"sources": len(rows), "valid_intervals": len(valid),
            "physical_center_covered": sum(row["physical_center_covered"] for row in rows),
            "tracker_x_error_gt_0_05_m": sum(value > .05 for value in errors),
            "tracker_x_error_max_m": max(errors),
            "union_width_median_m": statistics.median(
                row["full_physical_x_union_width_m"] for row in valid) if valid else None,
            "original_v1_x_width_median_m": statistics.median(
                row["original_v1_x_width_m"] for row in rows),
            "first_invalid_or_missed": ({key: missed[0][key] for key in
                ("group", "source_t", "selected_point_count", "center_lower_x_m",
                 "center_upper_x_m", "physical_center_x_m")}
                if missed else None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("offset_series", "offset_evidence", "phase2_trial",
                 "phase2_evidence", "moving_cross_evidence", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    offset, offset_evidence, phase2, phase2_evidence, cross, output = (
        getattr(args, name).resolve() for name in
        ("offset_series", "offset_evidence", "phase2_trial",
         "phase2_evidence", "moving_cross_evidence", "output"))
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip():
        raise ValueError("X interval rule not committed")
    evaluation_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    offset_manifest = json.loads((offset_evidence / "manifest.json").read_text())
    phase2_manifest = json.loads((phase2_evidence / "manifest.json").read_text())
    cross_summary = json.loads((cross / "summary.json").read_text())
    if digest(cross / "sources.csv") != cross_summary["sources_sha256"]:
        raise ValueError("moving source window changed")
    occupancy, _ = static_map()
    output_rows = []
    input_hashes = {"moving_window_sources": digest(cross / "sources.csv")}
    for side in ("south", "north"):
        trial = offset / side
        for name in ("scans.jsonl", "odometry.jsonl", "predictions.jsonl", "gazebo_poses.jsonl"):
            path = trial / name
            key = f"{side}/{name}"
            input_hashes[key] = digest(path)
            if input_hashes[key] != offset_manifest["source_sha256"][key]:
                raise ValueError(f"offset input changed: {key}")
        source_path = offset_evidence / f"{side}_sources.csv"
        if digest(source_path) != offset_manifest["packaged_sha256"][source_path.name]:
            raise ValueError(f"offset source table changed: {side}")
        with source_path.open() as stream:
            sources = list(csv.DictReader(stream))
        scan_by_stamp = {round(row["t"], 6): row for row in read_rows(trial / "scans.jsonl")}
        odometry = read_rows(trial / "odometry.jsonl")
        odom_times = [row["t"] for row in odometry]
        predictions = confirmed_predictions(trial / "predictions.jsonl")
        truth = rows_from_transport(trial / "gazebo_poses.jsonl")
        truth_times = [row["t"] for row in truth]
        for source in sources:
            stamp = float(source["source_t"])
            key = round(stamp, 6)
            if key not in scan_by_stamp or key not in predictions:
                raise ValueError(f"offset source input missing: {side} {stamp}")
            robot = odom_at(odometry, odom_times, stamp)
            truth_x = interpolate_box(truth, truth_times, stamp)[0]
            output_rows.append(check_one(f"offset_{side}", stamp,
                                         scan_by_stamp[key], robot,
                                         predictions[key], truth_x, occupancy))
    phase2_paths = {"scans.jsonl": phase2 / "observation/scans.jsonl",
                    "trajectory.jsonl": phase2 / "observation/trajectory.jsonl",
                    "predictions.jsonl": phase2 / "predictions.jsonl"}
    for name, path in phase2_paths.items():
        input_hashes[f"phase2/{name}"] = digest(path)
        if input_hashes[f"phase2/{name}"] != phase2_manifest["compressed_streams"][name]["source_sha256"]:
            raise ValueError(f"moving input changed: {name}")
    with (cross / "sources.csv").open() as stream:
        moving_sources = list(csv.DictReader(stream))
    scans = {round(row["t"], 6): row for row in read_rows(phase2_paths["scans.jsonl"])}
    trajectory = read_rows(phase2_paths["trajectory.jsonl"])
    trajectory_times = [row["t"] for row in trajectory]
    predictions = confirmed_predictions(phase2_paths["predictions.jsonl"])
    for source in moving_sources:
        stamp = float(source["source_t"])
        key = round(stamp, 6)
        if key not in scans or key not in predictions:
            raise ValueError(f"moving source input missing: {stamp}")
        robot = interpolate_robot(trajectory, trajectory_times, stamp)
        if robot is None:
            raise ValueError(f"moving robot pose missing: {stamp}")
        output_rows.append(check_one("moving_prebreach", stamp, scans[key], robot,
                                     predictions[key], float(source["physical_x_m"]), occupancy))
    output.mkdir(parents=True)
    with (output / "sources.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)
    groups = {name: summarize([row for row in output_rows if row["group"] == name])
              for name in ("moving_prebreach", "offset_south", "offset_north")}
    report = {"schema": "rm_dynamic_prediction_source_x_interval_probe/v1",
              "scope": "Known-width scan point X-center interval on already inspected moving and static fixture data. No future X dynamics or runtime claim.",
              "evaluation_commit": evaluation_commit,
              "point_allowance_m": POINT_ALLOWANCE,
              "known_half_box_width_m": HALF_BOX_WIDTH,
              "input_sha256": input_hashes, "groups": groups,
              "moving_source_x_gate_failed": groups["moving_prebreach"]["physical_center_covered"] <
                                             groups["moving_prebreach"]["sources"],
              "details_sha256": digest(output / "sources.csv")}
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"groups": groups,
                      "moving_source_x_gate_failed": report["moving_source_x_gate_failed"]}, indent=2))


if __name__ == "__main__":
    main()
