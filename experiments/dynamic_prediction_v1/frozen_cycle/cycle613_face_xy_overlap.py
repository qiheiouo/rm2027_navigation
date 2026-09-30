#!/usr/bin/env python3
"""Screen one fixed overlap-area rank on the already inspected cycle 613."""
import argparse
import csv
import json
from pathlib import Path
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from native_critic_sensitivity import aggregate, open_loop_geometry
from phase2_filtered_audit import candidates, selected
from rank_probe import area, overlap_area
from envelope import AxisBox

ROOT = Path(__file__).resolve().parents[3]
SCALE = 3.81 / 254.
HARD_COMMON = SCALE * 1_000_000. / 9.
NEAR_UNIT = SCALE * 300.


def rank(values, safe):
    order = np.argsort(values, kind="stable")
    found = np.flatnonzero(safe[order])
    return {"best_safe_rank": int(found[0] + 1) if len(found) else None,
            "safe_top_10": int(safe[order[:10]].sum()),
            "top_rollout": int(order[0]), "top_safe": bool(safe[order[0]])}


def run(trial, evidence, output):
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT,
                               text=True).strip():
        raise ValueError("fixed overlap rule must be committed before evaluation")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                     text=True).strip()
    phase2, face = evidence / "phase2_holdout_20260929", evidence / "cycle613_face_xy_rank_20260930"
    prior = json.loads((face / "summary.json").read_text())
    fixed = json.loads((phase2 / "filtered_summary.json").read_text())
    cycle, meta, arrays, settings, profile_path, truth_path = selected(trial)
    if prior["cycle_json_sha256"] != digest(cycle) or \
            prior["cycle_binary_sha256"] != digest(cycle.with_suffix(".bin")) or \
            digest(face / "candidates.csv") != prior["details_sha256"] or \
            prior["predicted_hits_by_step"] != [0] + [300] * 8:
        raise ValueError("frozen face-box result changed")
    if digest(phase2 / "filtered_candidates.csv") != fixed["detail_sha256"] or \
            digest(phase2 / "filtered_native_scores.bin") != fixed["input_sha256"]["filtered_native_scores"]:
        raise ValueError("frozen native result changed")
    controls, _, poses, history = candidates(trial, meta, arrays, settings)
    if controls.shape != (300, 30, 3):
        raise ValueError("frozen batch changed")
    with (phase2 / "filtered_candidates.csv").open() as stream:
        candidate_rows = list(csv.DictReader(stream))
    if len(candidate_rows) != 300 or any(int(row["rollout"]) != i for i, row in
                                         enumerate(candidate_rows)):
        raise ValueError("frozen candidate table changed")
    safe = np.asarray([row["filtered_joint_safe"] == "1" for row in candidate_rows])
    if int(safe.sum()) != 3 or np.flatnonzero(safe).tolist() != [5, 176, 194]:
        raise ValueError("frozen true safety labels changed")
    native = np.fromfile(phase2 / "filtered_native_scores.bin", dtype="<f4")
    if native.shape != (300,) or not np.isfinite(native).all():
        raise ValueError("native filtered critic scores invalid")
    padded = meta["padded_footprint"]
    robot_area = area(padded)
    if robot_area <= 0:
        raise ValueError("invalid padded polygon area")
    by_step = np.zeros((300, 9), dtype=np.float64)
    for j, row in enumerate(prior["prediction_boxes"]):
        box = AxisBox(row["min_x"], row["min_y"], row["max_x"], row["max_y"])
        for i in range(300):
            robot = analyze.placed(padded, tuple(float(x) for x in poses[i, j]))
            by_step[i, j] = overlap_area(robot, box) / robot_area
    if np.min(by_step) < -1e-9 or np.max(by_step) > 1 + 1e-9 or \
            np.any(by_step[:, 1] <= 0):
        raise ValueError("overlap fractions do not match second-step strict contact")
    overlap = by_step.mean(axis=1)
    grade = np.asarray(NEAR_UNIT * overlap, dtype=np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in ("vx", "vy", "wz")], axis=-1)
    baseline = aggregate(native + np.float32(HARD_COMMON), controls,
                         initial, settings, history)
    candidate = aggregate(native + np.float32(HARD_COMMON) + grade, controls,
                          initial, settings, history)
    if abs(float(baseline["probability"][safe].sum()) -
           fixed["rescored_probability_on_filtered_joint_safe"]) > 1e-5:
        raise ValueError("captured constant-hard filtered baseline does not replay")
    truth = analyze.rows_from_transport(truth_path)
    truth_times = [row["t"] for row in truth]
    consumer = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, truth_times,
                  consumer + (j + 1) * settings["dt"]))
              for j in range(settings["steps"])]
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"]
                                 ["ros__parameters"]["footprint"])
    baseline_geometry = open_loop_geometry(baseline["filtered_sequence"], meta,
                                           settings, actual, body, padded)
    candidate_geometry = open_loop_geometry(candidate["filtered_sequence"], meta,
                                            settings, actual, body, padded)
    output.mkdir(parents=True)
    detail = output / "candidates.csv"
    with detail.open("x", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("rollout", "joint_safe", "mean_overlap_fraction", "grade_score",
                         "native_standard", "new_probability", "new_weighted_total"))
        for i in range(300):
            writer.writerow((i, int(safe[i]), overlap[i], grade[i], native[i],
                             candidate["probability"][i], candidate["weighted"][i]))
    report = {"schema": "rm_dynamic_prediction_cycle613_face_xy_overlap/v1",
              "scope": "Exploratory fixed score shape on already inspected one cycle; hard penalty stays common, area fraction is not calibrated collision probability. Other native scores from individually filtered candidates.",
              "evaluation_commit": commit,
              "input_sha256": {"cycle": digest(cycle),
                               "cycle_binary": digest(cycle.with_suffix(".bin")),
                               "face_summary": digest(face / "summary.json"),
                               "face_candidates": digest(face / "candidates.csv"),
                               "filtered_candidates": digest(phase2 / "filtered_candidates.csv"),
                               "native_scores": digest(phase2 / "filtered_native_scores.bin"),
                               "truth": digest(truth_path),
                               "profile": digest(profile_path)},
              "joint_safe_ids": [5, 176, 194],
              "overlap_fraction_span": float(np.ptp(overlap)),
              "grade_score_span": float(np.ptp(grade)),
              "overlap_only_rank": rank(overlap, safe),
              "native_plus_grade_rank": rank(native + grade, safe),
              "weighted_total_rank": rank(candidate["weighted"], safe),
              "baseline_safe_probability": float(baseline["probability"][safe].sum()),
              "candidate_safe_probability": float(candidate["probability"][safe].sum()),
              "candidate_effective_sample_size": float(1. / np.sum(candidate["probability"] ** 2)),
              "baseline_returned_control": baseline["returned_control"],
              "candidate_returned_control": candidate["returned_control"],
              "baseline_truth": baseline_geometry,
              "candidate_truth": candidate_geometry,
              "details_sha256": digest(detail)}
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({key: report[key] for key in (
        "overlap_fraction_span", "grade_score_span", "overlap_only_rank",
        "native_plus_grade_rank", "weighted_total_rank",
        "baseline_safe_probability", "candidate_safe_probability",
        "candidate_effective_sample_size", "baseline_truth", "candidate_truth")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    run(args.trial.resolve(), args.evidence.resolve(), args.output.resolve())
