#!/usr/bin/env python3
"""Transfer already frozen scan-box and overlap rules to selected phase-6 cycle."""
import argparse
import csv
import json
from pathlib import Path
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from costmap_mask_fixture import export, shortcut_threshold
from cycle613_face_xy_rank import boxes, four_sources, polygon
from cycle613_face_xy_overlap import NEAR_UNIT, rank
from envelope import AxisBox
from native_critic_sensitivity import aggregate, open_loop_geometry
from phase2_filtered_audit import candidates, selected
from rank_probe import area, overlap_area

ROOT = Path(__file__).resolve().parents[3]
HARD_UNIT = np.float32((3.81 / 254.) * 1_000_000. / 9.)
NEAR_STEP = np.float32((3.81 / 254.) * 300. / 9.)


def prepare(trial, output):
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT,
                               text=True).strip():
        raise ValueError("transfer rule and record must be committed before evaluation")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"],
                                     cwd=ROOT, text=True).strip()
    cycle, meta, arrays, settings, profile_path, truth_path = selected(trial)
    if settings["batch"] != 300 or settings["steps"] != 30:
        raise ValueError("phase-6 frozen batch changed")
    filtered_dir = trial / f"filtered_analysis_{meta['cycle_id']}"
    fixture = trial / f"filtered_fixture_{meta['cycle_id']}"
    filtered_summary = json.loads((filtered_dir / "summary.json").read_text())
    detail_path = filtered_dir / "candidates.csv"
    native_path = fixture / "filtered_native_scores.bin"
    mask_path = fixture / "filtered_static_mask.txt"
    if digest(detail_path) != filtered_summary["detail_sha256"] or \
            digest(native_path) != filtered_summary["input_sha256"]["filtered_native_scores"] or \
            digest(mask_path) != filtered_summary["input_sha256"]["filtered_static_mask"]:
        raise ValueError("filtered baseline changed")
    with detail_path.open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 300 or any(int(row["rollout"]) != i for i, row in enumerate(rows)):
        raise ValueError("filtered candidate table is incomplete")
    safe = np.asarray([row["filtered_joint_safe"] == "1" for row in rows])
    if int(safe.sum()) != filtered_summary["filtered_joint_safe_count"]:
        raise ValueError("joint safety count changed")
    native = np.fromfile(native_path, dtype="<f4")
    if native.shape != (300,) or not np.isfinite(native).all():
        raise ValueError("native score array invalid")
    prediction = analyze.event(meta, "prediction.input")
    confirmed = [item for item in prediction["tracks"] if item["state"] == 2]
    if prediction["status"] != "accepted" or len(confirmed) != 1:
        raise ValueError("selected cycle lacks exactly one accepted confirmed track")
    paths = {"predictions.jsonl": trial / "predictions.jsonl",
             "scans.jsonl": trial / "observation/scans.jsonl",
             "trajectory.jsonl": trial / "observation/trajectory.jsonl"}
    stamp = prediction["source_stamp_ns"] / 1e9
    history = four_sources(paths, stamp, confirmed[0])
    future, movement = boxes(history, prediction["source_age_s"], settings["dt"])
    controls, _, poses, control_history = candidates(trial, meta, arrays, settings)
    padded = meta["padded_footprint"]
    robot_area = area(padded)
    if robot_area <= 0:
        raise ValueError("invalid padded footprint")
    hits = np.zeros((300, 9), dtype=bool)
    near = np.zeros((300, 9), dtype=bool)
    overlap = np.zeros((300, 9), dtype=np.float64)
    for j, box in enumerate(future):
        pred = AxisBox(box["min_x"], box["min_y"], box["max_x"], box["max_y"])
        for i in range(300):
            robot = analyze.placed(padded, tuple(float(v) for v in poses[i, j]))
            gap = analyze.polygon_distance(robot, polygon(box))
            hits[i, j] = gap <= 1e-9
            near[i, j] = 1e-9 < gap < .02
            overlap[i, j] = overlap_area(robot, pred) / robot_area
    if np.min(overlap) < -1e-9 or np.max(overlap) > 1 + 1e-9:
        raise ValueError("invalid overlap fraction")
    hard = np.where(hits.any(axis=1), HARD_UNIT,
                    NEAR_STEP * near.sum(axis=1)).astype(np.float32)
    grade = (NEAR_UNIT * overlap.mean(axis=1)).astype(np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in ("vx", "vy", "wz")], axis=-1)
    hard_result = aggregate(native + hard, controls, initial, settings, control_history)
    graded_result = aggregate(native + hard + grade, controls, initial, settings,
                              control_history)
    truth = analyze.rows_from_transport(truth_path)
    truth_times = [row["t"] for row in truth]
    consumer = prediction["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, truth_times,
                  consumer + (j + 1) * settings["dt"]))
              for j in range(settings["steps"])]
    coverage = []
    for j, box in enumerate(future):
        xs, ys = zip(*actual[j])
        coverage.append(box["min_x"] <= min(xs) + 1e-9 and
                        box["max_x"] >= max(xs) - 1e-9 and
                        box["min_y"] <= min(ys) + 1e-9 and
                        box["max_y"] >= max(ys) - 1e-9)
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"]
                                 ["ros__parameters"]["footprint"])
    actual_sequence = np.stack([analyze.last(arrays, "after_filter." + axis)
                                for axis in ("vx", "vy", "wz")], axis=-1)
    names = ("captured", "scan_hard", "scan_hard_plus_fixed_overlap")
    sequences = (actual_sequence, hard_result["filtered_sequence"],
                 graded_result["filtered_sequence"])
    geometry = {name: open_loop_geometry(sequence, meta, settings, actual, body,
                                         padded)
                for name, sequence in zip(names, sequences)}
    aggregate_poses = [analyze.integrate_omni(
        *(sequence[:, axis] for axis in range(3)), meta["pose"], settings["dt"])
        for sequence in sequences]
    goal = json.loads((trial / "observation/summary.json").read_text())["goal"]
    start_goal_distance = float(np.hypot(meta["pose"][0] - goal[0],
                                         meta["pose"][1] - goal[1]))
    goal_progress = {name: start_goal_distance - float(np.hypot(
        poses_for_sequence[0][-1] - goal[0], poses_for_sequence[1][-1] - goal[1]))
        for name, poses_for_sequence in zip(names, aggregate_poses)}
    output.mkdir(parents=True)
    aggregate_map = output / "aggregate_map.bin"
    export(aggregate_map, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack([pose[axis] for pose in aggregate_poses])
                 for axis in range(3)), shortcut_threshold(meta, profile_path))
    detail = output / "candidates.csv"
    with detail.open("x", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("rollout", "joint_safe", "truth_body_gap_m", "native_static_collision",
                         "first_scan_hit_step", "scan_near_count", "scan_overlap_fraction",
                         "scan_hard_score", "fixed_overlap_score", "native_standard_score",
                         "hard_probability", "graded_probability"))
        for i, row in enumerate(rows):
            first = int(np.argmax(hits[i]) + 1) if hits[i].any() else 0
            writer.writerow((i, int(safe[i]), row["filtered_truth_body_gap_m"],
                             row["native_static_collision"], first, int(near[i].sum()),
                             overlap[i].mean(), hard[i], grade[i], native[i],
                             hard_result["probability"][i], graded_result["probability"][i]))
    report = {
        "schema": "rm_dynamic_prediction_phase6_fixed_transfer/v1",
        "scope": "One selected new phase, frozen prior scan box and overlap units, offline counterfactual only; truth labels never enter prediction or scoring.",
        "evaluation_commit": commit, "cycle_id": meta["cycle_id"],
        "input_sha256": {"cycle": digest(cycle), "cycle_bin": digest(cycle.with_suffix(".bin")),
                         "profile": digest(profile_path), "truth": digest(truth_path),
                         "filtered_summary": digest(filtered_dir / "summary.json"),
                         "filtered_candidates": digest(detail_path),
                         "filtered_native_scores": digest(native_path),
                         "filtered_static_mask": digest(mask_path),
                         **{key: digest(value) for key, value in paths.items()}},
        "source_stamp_s": stamp, "consumer_sim_s": consumer,
        "source_age_s": prediction["source_age_s"],
        "source_history": history, "movement": movement,
        "prediction_boxes": future, "physical_full_box_covered_by_step": coverage,
        "scan_predicted_hits_by_step": hits.sum(axis=0).tolist(),
        "scan_predicted_any_hit": int(hits.any(axis=1).sum()),
        "original_v1_predicted_hits_by_step": filtered_summary["prediction_hits_by_step"],
        "joint_safe_count": int(safe.sum()),
        "score_span": {"scan_hard": float(np.ptp(hard)),
                       "fixed_overlap": float(np.ptp(grade))},
        "rank": {"scan_hard_only": rank(hard, safe),
                 "fixed_overlap_only": rank(grade, safe),
                 "native_plus_scan_hard": rank(native + hard, safe),
                 "native_plus_scan_hard_and_overlap": rank(native + hard + grade, safe),
                 "weighted_scan_hard": rank(hard_result["weighted"], safe),
                 "weighted_scan_hard_and_overlap": rank(graded_result["weighted"], safe)},
        "safe_probability_mass": {"captured": filtered_summary[
            "captured_probability_on_filtered_joint_safe"],
            "scan_hard": float(hard_result["probability"][safe].sum()),
            "scan_hard_plus_fixed_overlap": float(
                graded_result["probability"][safe].sum())},
        "returned_control": {"captured": analyze.event(meta, "output"),
                             "scan_hard": hard_result["returned_control"],
                             "scan_hard_plus_fixed_overlap": graded_result["returned_control"]},
        "truth_geometry": geometry,
        "goal_xy": goal[:2], "three_second_goal_progress_m": goal_progress,
        "aggregate_map_sha256": digest(aggregate_map),
        "candidates_sha256": digest(detail),
    }
    (output / "transfer.json").write_text(json.dumps(report, indent=2,
                                                sort_keys=True) + "\n")
    print(json.dumps({key: report[key] for key in (
        "cycle_id", "physical_full_box_covered_by_step", "scan_predicted_hits_by_step",
        "joint_safe_count", "rank", "safe_probability_mass", "truth_geometry")},
                     indent=2))


def finalize(output):
    report = json.loads((output / "transfer.json").read_text())
    if digest(output / "aggregate_map.bin") != report["aggregate_map_sha256"]:
        raise ValueError("aggregate static fixture changed")
    mask_path = output / "aggregate_static_mask.txt"
    mask = [int(value) for value in mask_path.read_text().split()]
    if len(mask) != 3 or any(value not in (0, 1) for value in mask):
        raise ValueError("expected three native static collision flags")
    if (output / "summary.json").exists():
        raise FileExistsError(output / "summary.json")
    report["native_static_collision"] = dict(zip(
        ("captured", "scan_hard", "scan_hard_plus_fixed_overlap"),
        (bool(value) for value in mask)))
    report["aggregate_static_mask_sha256"] = digest(mask_path)
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                               sort_keys=True) + "\n")
    print(json.dumps({"cycle_id": report["cycle_id"],
                      "static_collision": report["native_static_collision"],
                      "truth_geometry": report["truth_geometry"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("trial", type=Path)
    prep.add_argument("output", type=Path)
    done = sub.add_parser("finalize")
    done.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial.resolve(), args.output.resolve())
    else:
        finalize(args.output.resolve())
