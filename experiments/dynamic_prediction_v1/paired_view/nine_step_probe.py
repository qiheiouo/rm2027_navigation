#!/usr/bin/env python3
"""Evaluate frozen source-time Y correction through nine future fixture steps."""
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

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_extent_shadow_20260924"))
from dynamic_metrics import rows_from_transport
from envelope import predicted_box

PROFILE = ROOT / "docs/dynamic_navigation/evidence/phase2_holdout_20260929/profile.yaml"
ACCELERATION = 0.5551652475612764
SOURCE_BUDGET_M = 0.05
HALF_BOX_Y_M = 0.275
HORIZONS_S = tuple(round(step * 0.1, 1) for step in range(1, 10))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def interpolate_y(rows, times, stamp):
    index = bisect.bisect_right(times, stamp)
    if not 0 < index < len(rows):
        raise ValueError(f"truth does not bracket {stamp:.6f}")
    a, b = rows[index - 1], rows[index]
    weight = (stamp - a["t"]) / (b["t"] - a["t"])
    return a["obstacle"][1] + weight * (b["obstacle"][1] - a["obstacle"][1])


def quantile(values, p):
    if not values:
        return None
    ordered = sorted(values)
    index = p * (len(ordered) - 1)
    lower = int(index)
    return ordered[lower] + (index - lower) * (ordered[min(lower + 1, len(ordered) - 1)]
                                              - ordered[lower])


def turns_inside_window(rows, times, stamp):
    points = [interpolate_y(rows, times, stamp + 0.1 * step) for step in range(10)]
    velocities = [(b - a) / 0.1 for a, b in zip(points, points[1:])]
    return any(value > 0.05 for value in velocities) and any(
        value < -0.05 for value in velocities)


def summarize(rows):
    if not rows:
        return {"source_steps": 0}
    residuals = [abs(row["corrected_center_error_m"]) for row in rows]
    missed = [row for row in rows if not row["candidate_y_covers_physical_box"]]
    return {"source_steps": len(rows),
            "candidate_y_covered": len(rows) - len(missed),
            "original_v1_y_covered": sum(row["original_v1_y_covers_physical_box"] for row in rows),
            "residual_abs_p90_m": quantile(residuals, 0.9),
            "residual_abs_max_m": max(residuals),
            "candidate_excess_max_m": max(row["candidate_excess_m"] for row in rows),
            "candidate_first_miss": ({key: missed[0][key] for key in
                ("side", "source_t", "step", "horizon_s", "candidate_excess_m")}
                if missed else None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series", type=Path)
    parser.add_argument("source_evidence", type=Path)
    parser.add_argument("face_evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    series, evidence, face, output = (path.resolve() for path in
                                      (args.series, args.source_evidence,
                                       args.face_evidence, args.output))
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip():
        raise ValueError("evaluation source or documents are not committed")
    evaluation_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    plan = json.loads((series / "plan.json").read_text())
    manifest = json.loads((evidence / "manifest.json").read_text())
    face_summary = json.loads((face / "summary.json").read_text())
    profile = yaml.safe_load(PROFILE.read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    if plan.get("view_set") != "offset" or plan["run_commit"] != manifest["run_commit"] or \
            float(params["reference_acceleration"]) != ACCELERATION or \
            float(params["object_height"]) / 2 != HALF_BOX_Y_M or \
            float(params["object_width"]) != 0.45:
        raise ValueError("frozen geometry/profile mismatch")
    if digest(face / "sources.csv") != face_summary["details_sha256"]:
        raise ValueError("frozen near-face source table changed")
    with (face / "sources.csv").open() as stream:
        face_rows = {(row["side"], round(float(row["source_t"]), 6)): row
                     for row in csv.DictReader(stream)}
    output.mkdir(parents=True)
    detail = []
    inputs = {"profile": digest(PROFILE), "near_face_sources": digest(face / "sources.csv")}
    for side in ("south", "north"):
        trial = series / side
        truth_file = trial / "gazebo_poses.jsonl"
        inputs[f"{side}/gazebo_poses.jsonl"] = digest(truth_file)
        if inputs[f"{side}/gazebo_poses.jsonl"] != manifest["source_sha256"][f"{side}/gazebo_poses.jsonl"]:
            raise ValueError(f"physical truth changed: {side}")
        source_file = evidence / f"{side}_sources.csv"
        inputs[source_file.name] = digest(source_file)
        if inputs[source_file.name] != manifest["packaged_sha256"][source_file.name]:
            raise ValueError(f"tracker source table changed: {side}")
        truth = rows_from_transport(truth_file)
        times = [row["t"] for row in truth]
        with source_file.open() as stream:
            sources = list(csv.DictReader(stream))
        for source in sources:
            stamp = float(source["source_t"])
            if not 16 <= stamp <= 43.0:
                continue
            face_row = face_rows.get((side, round(stamp, 6)))
            if face_row is None or face_row["near_face_center_y_m"] == "":
                raise ValueError(f"missing frozen face estimate: {side} {stamp}")
            if abs(interpolate_y(truth, times, stamp) - float(source["physical_box_y_m"])) > 1e-7:
                raise ValueError(f"source truth label mismatch: {side} {stamp}")
            corrected_y = float(face_row["near_face_center_y_m"])
            tracker_y = float(source["tracker_y_m"])
            vy = float(source["tracker_vy_mps"])
            visible_y = float(source["visible_span_y_m"])
            turning = turns_inside_window(truth, times, stamp)
            for step, horizon in enumerate(HORIZONS_S, 1):
                true_y = interpolate_y(truth, times, stamp + horizon)
                corrected_forecast = corrected_y + vy * horizon
                corrected_error = corrected_forecast - true_y
                mismatch = 0.5 * ACCELERATION * horizon ** 2
                allowed_center_error = SOURCE_BUDGET_M + mismatch
                candidate_excess = max(0.0, abs(corrected_error) - allowed_center_error)
                original_box = predicted_box((0.0, tracker_y), (0.0, vy),
                                             (0.0, visible_y), (0.45, 0.55),
                                             0.0, horizon, ACCELERATION)
                original_y_covers = (original_box.min_y <= true_y - HALF_BOX_Y_M + 1e-9 and
                                     original_box.max_y >= true_y + HALF_BOX_Y_M - 1e-9)
                detail.append({"side": side, "source_t": stamp, "step": step,
                               "horizon_s": horizon, "truth_turns_within_0_9_s": turning,
                               "tracker_vy_mps": vy, "true_box_center_y_m": true_y,
                               "corrected_forecast_center_y_m": corrected_forecast,
                               "corrected_center_error_m": corrected_error,
                               "candidate_allowed_center_error_m": allowed_center_error,
                               "candidate_excess_m": candidate_excess,
                               "candidate_y_covers_physical_box": candidate_excess <= 1e-9,
                               "original_v1_y_covers_physical_box": original_y_covers,
                               "original_v1_y_width_m": original_box.max_y - original_box.min_y,
                               "candidate_y_width_m": 2 * (HALF_BOX_Y_M + allowed_center_error)})
    with (output / "source_steps.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(detail[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(detail)
    by_side = {}
    for side in ("south", "north"):
        selected = [row for row in detail if row["side"] == side]
        by_side[side] = {"all": summarize(selected),
                         "turn_window": summarize([row for row in selected
                                                   if row["truth_turns_within_0_9_s"]]),
                         "no_turn_window": summarize([row for row in selected
                                                      if not row["truth_turns_within_0_9_s"]]),
                         "steps": {str(step): summarize([row for row in selected
                                                         if row["step"] == step])
                                   for step in range(1, 10)}}
        by_side[side]["unique_sources"] = len({row["source_t"] for row in selected})
    report = {"schema": "rm_dynamic_prediction_nine_step_probe/v1",
              "scope": "Same already inspected fixture, source-age zero and Y-only. Truth only labels future occupancy and turn groups; no MPPI controller claim.",
              "source_run_commit": plan["run_commit"],
              "evaluation_commit": evaluation_commit,
              "horizons_s": HORIZONS_S, "source_center_budget_m": SOURCE_BUDGET_M,
              "reference_acceleration_mps2": ACCELERATION,
              "input_sha256": inputs, "results": by_side,
              "candidate_nine_step_y_coverage_failed": any(
                  by_side[side]["all"]["candidate_y_covered"] <
                  by_side[side]["all"]["source_steps"] for side in ("south", "north")),
              "details_sha256": digest(output / "source_steps.csv")}
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"results": {side: by_side[side]["all"] for side in by_side},
                      "failed": report["candidate_nine_step_y_coverage_failed"]}, indent=2))


if __name__ == "__main__":
    main()
