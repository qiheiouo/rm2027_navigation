#!/usr/bin/env python3
"""Check a fixed source-noise and history-curvature velocity interval."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
WORLD = ROOT / "src/rm_simulation/worlds/phase1_omni.sdf"
ACCELERATION = 0.5551652475612764
SIGMA_RANGE_M = 0.01


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path, expected):
    if digest(path) != expected:
        raise ValueError(f"frozen input changed: {path}")
    with path.open() as stream:
        return list(csv.DictReader(stream))


def med(values):
    return statistics.median(values) if values else None


def describe(rows):
    if not rows:
        return {"source_steps": 0}
    valid = [row for row in rows if row["velocity_half_width_mps"] is not None]
    misses = [row for row in valid if not row["interval_y_covers_physical_box"]]
    return {"source_steps": len(rows), "valid_source_steps": len(valid),
            "interval_y_covered_all": sum(row["interval_y_covers_physical_box"] for row in rows),
            "interval_y_covered_valid": len(valid) - len(misses),
            "causal_point_y_covered_valid": sum(row["causal_point_y_covers_physical_box"] for row in valid),
            "original_v1_y_covered_valid": sum(row["original_v1_y_covers_physical_box"] for row in valid),
            "interval_excess_max_m": max((row["interval_excess_m"] for row in valid), default=None),
            "interval_width_median_m": med([row["interval_y_width_m"] for row in valid]),
            "original_v1_width_median_m": med([row["original_v1_y_width_m"] for row in valid]),
            "first_interval_miss": ({key: misses[0][key] for key in
                ("side", "source_t", "step", "horizon_s", "interval_excess_m")}
                if misses else None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source_evidence", "face_evidence", "nine_step_evidence",
                 "causal_evidence", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    evidence, face, nine, causal, output = (
        getattr(args, name).resolve() for name in
        ("source_evidence", "face_evidence", "nine_step_evidence",
         "causal_evidence", "output"))
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip():
        raise ValueError("interval plan not committed")
    evaluation_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    manifest = json.loads((evidence / "manifest.json").read_text())
    plan = json.loads((evidence / "plan.json").read_text())
    face_summary = json.loads((face / "summary.json").read_text())
    nine_summary = json.loads((nine / "summary.json").read_text())
    causal_summary = json.loads((causal / "summary.json").read_text())
    if digest(WORLD) != plan["file_sha256"]["source_world"] or \
            nine_summary["reference_acceleration_mps2"] != ACCELERATION:
        raise ValueError("world source changed")
    sensor = ET.parse(WORLD).find(".//sensor[@name='phase1_planar_lidar']")
    if sensor is None or float(sensor.find(".//noise/stddev").text) != SIGMA_RANGE_M:
        raise ValueError("source lidar noise differs")
    face_path, nine_path, causal_path = (
        face / "sources.csv", nine / "source_steps.csv", causal / "source_steps.csv")
    faces = read_csv(face_path, face_summary["details_sha256"])
    nine_rows = read_csv(nine_path, nine_summary["details_sha256"])
    causal_rows = read_csv(causal_path, causal_summary["details_sha256"])
    if len(nine_rows) != len(causal_rows) or not (
        manifest["run_commit"] == nine_summary["source_run_commit"] ==
        causal_summary["source_run_commit"]):
        raise ValueError("source runs differ")
    by_side = {}
    for row in faces:
        by_side.setdefault(row["side"], []).append(float(row["source_t"]))
    velocity_bounds = {}
    for side, stamps in by_side.items():
        stamps.sort()
        for index, stamp in enumerate(stamps):
            key = (side, round(stamp, 6))
            if index < 3:
                velocity_bounds[key] = None
                continue
            history = stamps[index - 3:index + 1]
            span = history[-1] - history[0]
            if not 0.15 <= span <= 0.30:
                velocity_bounds[key] = None
                continue
            mean_t = statistics.mean(history)
            sxx = sum((t - mean_t) ** 2 for t in history)
            velocity_bounds[key] = 3 * SIGMA_RANGE_M / math.sqrt(sxx) + \
                ACCELERATION * (stamp - mean_t)
    detail = []
    for baseline, causal_row in zip(nine_rows, causal_rows):
        key = (baseline["side"], round(float(baseline["source_t"]), 6))
        if key != (causal_row["side"], round(float(causal_row["source_t"]), 6)) or \
                baseline["step"] != causal_row["step"]:
            raise ValueError("frozen step order differs")
        velocity_bound = velocity_bounds[key]
        causal_vy = causal_row["causal_vy_mps"]
        if (velocity_bound is None) != (causal_vy == ""):
            raise ValueError(f"velocity availability differs: {key}")
        horizon = float(baseline["horizon_s"])
        allowed = float(baseline["candidate_allowed_center_error_m"])
        if abs(allowed - (.05 + .5 * ACCELERATION * horizon ** 2)) > 1e-9:
            raise ValueError("frozen temporal budget differs")
        center_error = (float(causal_row["causal_center_error_m"])
                        if velocity_bound is not None else None)
        interval_margin = allowed + horizon * velocity_bound if velocity_bound is not None else None
        excess = (max(0.0, abs(center_error) - interval_margin)
                  if interval_margin is not None else None)
        detail.append({"side": key[0], "source_t": float(baseline["source_t"]),
                       "step": int(baseline["step"]), "horizon_s": horizon,
                       "truth_turns_within_0_9_s": baseline["truth_turns_within_0_9_s"] == "True",
                       "velocity_half_width_mps": velocity_bound,
                       "center_error_m": center_error,
                       "interval_margin_m": interval_margin,
                       "interval_excess_m": excess,
                       "interval_y_covers_physical_box": excess is not None and excess <= 1e-9,
                       "causal_point_y_covers_physical_box": causal_row["causal_y_covers_physical_box"] == "True",
                       "original_v1_y_covers_physical_box": baseline["original_v1_y_covers_physical_box"] == "True",
                       "interval_y_width_m": 2 * (.275 + interval_margin) if interval_margin is not None else None,
                       "original_v1_y_width_m": float(baseline["original_v1_y_width_m"])})
    output.mkdir(parents=True)
    with (output / "source_steps.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(detail[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(detail)
    results = {}
    for side in ("south", "north"):
        side_rows = [row for row in detail if row["side"] == side]
        first = [row for row in side_rows if row["step"] == 1]
        bounds = [row["velocity_half_width_mps"] for row in first
                  if row["velocity_half_width_mps"] is not None]
        results[side] = {"total_sources": len(first), "valid_sources": len(bounds),
                         "velocity_half_width_mps": {"median": med(bounds),
                             "min": min(bounds), "max": max(bounds)},
                         "all": describe(side_rows),
                         "turn_window": describe([row for row in side_rows
                                                  if row["truth_turns_within_0_9_s"]]),
                         "no_turn_window": describe([row for row in side_rows
                                                     if not row["truth_turns_within_0_9_s"]]),
                         "steps": {str(step): describe([row for row in side_rows
                                                        if row["step"] == step])
                                   for step in range(1, 10)}}
    report = {"schema": "rm_dynamic_prediction_face_velocity_uncertainty_probe/v1",
              "scope": "Exploratory Y-only interval on already inspected same-box data. Gaussian extrema assumption is not a safety guarantee; no runtime or MPPI claim.",
              "evaluation_commit": evaluation_commit,
              "source_run_commit": manifest["run_commit"],
              "noise_sigma_m": SIGMA_RANGE_M,
              "reference_acceleration_mps2": ACCELERATION,
              "input_sha256": {"world": digest(WORLD), "face": digest(face_path),
                               "nine": digest(nine_path), "causal": digest(causal_path)},
              "results": results,
              "nine_step_y_coverage_failed": any(
                  results[side]["all"]["interval_y_covered_all"] <
                  results[side]["all"]["source_steps"] for side in ("south", "north")),
              "details_sha256": digest(output / "source_steps.csv")}
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"failed": report["nine_step_y_coverage_failed"],
                      "results": {side: {"valid_sources": results[side]["valid_sources"],
                                         "velocity_half_width_mps": results[side]["velocity_half_width_mps"],
                                         "all": results[side]["all"],
                                         "step_1": results[side]["steps"]["1"],
                                         "step_9": results[side]["steps"]["9"]}
                                  for side in ("south", "north")}}, indent=2))


if __name__ == "__main__":
    main()
