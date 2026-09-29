#!/usr/bin/env python3
"""Audit joint scan-phase and observer-view support without future truth labels."""
import argparse
import json
from pathlib import Path

from heldout_geometry_audit import digest, recorded_messages
from raw_scan_support_audit import (near_full_span, one_trial, static_map,
                                    trajectory_pose)


BANDS = (.1, .2, .3)
TARGET_NAMES = ("intrusion_162", "goal_263")


def source_records(path, occupancy):
    audit, rows = one_trial(path, recorded_messages(path), occupancy,
                            return_source_rows=True,
                            pose_source="recorded_odom")
    trajectory = [json.loads(line) for line in
                  (path / "observation/trajectory.jsonl").open()]
    times = [row["t"] for row in trajectory]
    records = []
    for row in rows:
        if not near_full_span(row) or "scan_center_velocity" not in row:
            continue
        y = float(row["raw_bbox_mid"][1])
        vy = float(row["scan_center_velocity"][1])
        robot = trajectory_pose(trajectory, times, row["source_t"])
        if robot is None:
            continue
        records.append({"source_t": row["source_t"],
                        "center_y_m": y, "velocity_y_mps": vy,
                        "view_y": "north" if robot[1] > y else "south"})
    return audit, records


def compare(target, records, band):
    y = target["scan_center_xy"][1]
    vy = target["scan_velocity_xy"][1]
    by_view = {}
    for view in ("north", "south"):
        group = [row for row in records if row["view_y"] == view]
        eligible = [row for row in group
                    if abs(row["center_y_m"] - y) <= band and
                    abs(row["velocity_y_mps"] - vy) <= band]
        nearest = min(group, key=lambda row: max(
            abs(row["center_y_m"] - y),
            abs(row["velocity_y_mps"] - vy)), default=None)
        by_view[view] = {
            "total_source_scans": len(group),
            "within_joint_band": len(eligible),
            "nearest_joint_max_component_delta":
                max(abs(nearest["center_y_m"] - y),
                    abs(nearest["velocity_y_mps"] - vy)) if nearest else None,
            "nearest_source_t": nearest["source_t"] if nearest else None,
            "nearest_center_y_m": nearest["center_y_m"] if nearest else None,
            "nearest_velocity_y_mps": nearest["velocity_y_mps"]
            if nearest else None}
    return by_view


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    model = json.loads(args.model_evidence.read_text())
    targets = json.loads(args.target_evidence.read_text())
    if targets.get("training_message_source") != "recorded_odom":
        raise ValueError("target evidence must use odometry-matched training")
    paths = [Path(row["trial"]) for row in
             model["historical_leave_one_trial_out"][:4]]
    if len(paths) != 4 or any("tdt_" not in path.name for path in paths):
        raise ValueError("expected four fixed old T-DT training trials")
    occupancy, _ = static_map()
    training = []
    for path in paths:
        audit, records = source_records(path, occupancy)
        training.append({"trial": str(path),
                         "source_sha256": audit["source_sha256"],
                         "eligible_source_scans": len(records),
                         "records": records})
    output = []
    for name in TARGET_NAMES:
        target = next(row for row in targets["cases"] if row["name"] == name)
        result = {"name": name,
                  "source_t": target["scan_source_t"],
                  "scan_center_y_m": target["scan_center_xy"][1],
                  "scan_velocity_y_mps": target["scan_velocity_xy"][1],
                  "view_y": target["observer_view_y"],
                  "bands": {str(band): [
                      {"trial": row["trial"],
                       "views": compare(target, row["records"], band)}
                      for row in training] for band in BANDS}}
        output.append(result)
    report = {
        "schema": "rm_dynamic_prediction/scan_phase_view_support_audit/v1",
        "scope": "Causal support diagnostic only: source scan midpoint y, up to 0.4s fitted scan velocity y, and recorded-odometry observer view. Four old T-DT trials compared with two fixed Navfn source scans. All bands apply the same numeric value to y meters and vy meters/second, solely as an audit tolerance. No future obstacle truth or MPPI safety labels enter selection; no model fit or controller change.",
        "model_evidence_sha256": digest(args.model_evidence),
        "target_evidence_sha256": digest(args.target_evidence),
        "training": [{key: value for key, value in row.items()
                      if key != "records"} for row in training],
        "target_cases": output}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"training_trials": len(training),
                      "target_cases": len(output), "output": str(args.output)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-evidence", type=Path, required=True)
    parser.add_argument("--target-evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args())
