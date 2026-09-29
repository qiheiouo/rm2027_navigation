#!/usr/bin/env python3
"""Offline truth-input ceiling for V1 overlap and existing near-distance terms.

This deliberately scores with future Gazebo box poses. It is a diagnostic of
score representation, never a deployable predictor or controller change.
"""
import argparse
import csv
import json
from pathlib import Path
import shutil
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from costmap_mask_fixture import export, shortcut_threshold
from envelope import AxisBox
from filtered_batch_candidate_rank import raw_controls
from filtered_candidate_rank_audit import safe_auc
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_sensitivity import AXES, aggregate
from output_first_step_audit import actual, pose_array
from peak_temporal_rank_probe import CASES, reference_controls, safe_labels
import rank_probe
import replay_ranking


CONTROL_HEADER = struct.Struct("<3I")


def truth_features(meta, poses, boxes, steps):
    if poses.shape[1] < steps:
        raise ValueError("candidate trajectory shorter than V1 horizon")
    area = rank_probe.area(meta["padded_footprint"])
    if area <= 0:
        raise ValueError("invalid padded footprint")
    overlap = np.zeros((len(poses), steps), dtype=np.float64)
    gap = np.zeros_like(overlap)
    for step in range(steps):
        polygon = boxes[step]
        box = AxisBox(min(point[0] for point in polygon),
                      min(point[1] for point in polygon),
                      max(point[0] for point in polygon),
                      max(point[1] for point in polygon))
        if abs(rank_probe.area(polygon) -
               (box.max_x - box.min_x) * (box.max_y - box.min_y)) > 1e-6:
            raise ValueError("truth box is rotated; axis-aligned oracle invalid")
        for index in range(len(poses)):
            robot = analyze.placed(meta["padded_footprint"],
                                   tuple(map(float, poses[index, step])))
            overlap[index, step] = rank_probe.overlap_area(robot, box) / area
            gap[index, step] = analyze.polygon_distance(robot, polygon)
    return overlap.mean(axis=1), (gap < .02).sum(axis=1), gap.min(axis=1)


def inputs(args):
    raw = json.loads(args.raw_inputs.read_text())
    result = {}
    for name, cycle_id, trial_key, seed, score_key in CASES:
        trial, score = getattr(args, trial_key), getattr(args, score_key)
        result[name] = (cycle_id, trial, seed, score, raw)
    return result


def prepare(args):
    if args.work_dir.exists():
        raise FileExistsError(args.work_dir)
    args.work_dir.mkdir(parents=True)
    evidence = Path("docs/dynamic_navigation/evidence")
    cases = []
    for name, (cycle_id, trial, seed, score_path, raw) in inputs(args).items():
        directory = args.work_dir / name
        directory.mkdir()
        cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
        profile_path = trial / "profile.yaml"
        truth_path = trial / "gazebo_poses.jsonl"
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        if (meta["cycle_id"], settings["batch"], settings["steps"],
                settings["iterations"]) != (cycle_id, 300, 30, 1):
            raise ValueError(f"frozen cycle differs: {name}")
        profile = yaml.safe_load(profile_path.read_text())
        params = profile["controller_server"]["ros__parameters"][
            "FollowPath"]["PredictionV1Critic"]
        steps = int(np.floor(params["horizon"] / settings["dt"] + 1e-9))
        if steps != 9:
            raise ValueError("V1 horizon differs")
        prior = replay_ranking.history_from_trial(
            trial / "mppi_cycles", cycle_id, arrays)
        history = np.stack([prior[axis] for axis in AXES], axis=-1)
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1)
        if seed is None:
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
            raw_path = None
        else:
            source = next(item for item in raw["cases"] if item["name"] == name)
            raw_path = Path(source["controls"])
            if digest(raw_path) != source["sha256"]["controls"]:
                raise ValueError(f"raw control hash differs: {name}")
            controls = raw_controls(raw_path, 300)
        poses = filtered_poses(controls, meta, settings, history)
        original, hits, original_mean = filtered_prediction_score(
            meta, poses, params)
        if np.any((original >= 100.) & (original < 1000.)):
            raise ValueError("V1 scale differs")
        hard = original > 100.
        boxes = actual(meta, settings, truth_path)
        truth_overlap, truth_near, truth_min_gap = truth_features(
            meta, poses, boxes, steps)
        hard_unit = np.float32((3.81 / 254.) * 1_000_000. / steps)
        near_unit = np.float32((3.81 / 254.) * 300. / steps)
        oracle_overlap = original + np.float32(
            hard_unit * (truth_overlap - original_mean) * hard)
        oracle_near = original + np.float32(
            (near_unit * truth_near - hard_unit * original_mean) * hard)
        standard = np.fromfile(score_path, dtype="<f4")
        if standard.shape != (300,) or not np.isfinite(standard).all():
            raise ValueError(f"native standard score differs: {name}")
        variants = {
            "baseline": (original, aggregate(standard + original, controls,
                                            initial, settings, history)),
            "truth_overlap": (oracle_overlap, aggregate(
                standard + oracle_overlap, controls, initial, settings, history)),
            "truth_near": (oracle_near, aggregate(
                standard + oracle_near, controls, initial, settings, history)),
        }
        reference, reference_path = reference_controls(name, evidence)
        baseline_error = float(np.max(np.abs(
            variants["baseline"][1]["filtered_sequence"] - reference)))
        if baseline_error > 3e-5:
            raise ValueError(f"published baseline differs: {name}")
        label_path = (evidence / "filtered_candidate_rank_20260928" /
                      f"{name}_detail.csv" if seed is None else
                      evidence / "filtered_batch_candidate_rank_20260928" /
                      f"seed_{seed}_batch2000_labels.csv")
        safe = safe_labels(label_path)
        with label_path.open(newline="") as stream:
            labels = list(csv.DictReader(stream))[:300]
        if len(labels) != 300 or any(int(row["rollout"]) != index or
                                      (row["joint_safe_3s"] == "1") != safe[index]
                                      for index, row in enumerate(labels)):
            raise ValueError(f"candidate label order differs: {name}")
        controls_out = np.asarray([
            result["filtered_sequence"] for _, result in variants.values()],
            dtype="<f4")
        control_path = directory / "output_controls.bin"
        control_path.write_bytes(CONTROL_HEADER.pack(0x43545231, 3, 30) +
                                 controls_out.tobytes())
        output_poses = pose_array(controls_out, meta, settings, True)
        pose_path = directory / "output_poses.bin"
        output_poses.tofile(pose_path)
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        body_gap = np.asarray([[analyze.polygon_distance(
            analyze.placed(body, tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in output_poses])
        padded_gap = np.asarray([[analyze.polygon_distance(
            analyze.placed(meta["padded_footprint"], tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in output_poses])
        static_path = directory / "static.bin"
        export(static_path, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(output_poses[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        csv_path = directory / "candidates.csv"
        with csv_path.open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("index", "joint_safe_3s", "hard_v1",
                             "body_gap_3s_m", "padded_gap_3s_m",
                             "static_collision",
                             "standard_score", "original_v1_score",
                             "original_overlap_mean", "truth_overlap_mean",
                             "truth_near_count_9", "truth_min_padded_gap_9_m",
                             "baseline_total", "truth_overlap_total",
                             "truth_near_total"))
            for index in range(300):
                writer.writerow((index, int(safe[index]), int(hard[index]),
                                 labels[index]["body_gap_3s_m"],
                                 labels[index]["padded_gap_3s_m"],
                                 labels[index]["static_collision"],
                                 float(standard[index]), float(original[index]),
                                 float(original_mean[index]),
                                 float(truth_overlap[index]),
                                 int(truth_near[index]),
                                 float(truth_min_gap[index]),
                                 *(float(result["weighted"][index])
                                   for _, result in variants.values())))
        goal = (json.loads((trial / "runtime_audit.json").read_text())[
            "navigation_result"]["goal"] if cycle_id == 263 else None)
        outputs = []
        for index, (label, (v1, result)) in enumerate(variants.items()):
            probability = result["probability"]
            top = int(np.argmin(result["weighted"]))
            item = {"name": label, "top_index": top,
                    "top_candidate_joint_safe_3s": bool(safe[top]),
                    "top_candidate_body_gap_3s_m": float(
                        labels[top]["body_gap_3s_m"]),
                    "top_candidate_padded_gap_3s_m": float(
                        labels[top]["padded_gap_3s_m"]),
                    "top_probability": float(probability[top]),
                    "safe_probability_mass": float(probability[safe].sum()),
                    "effective_sample_size": float(1. / np.sum(probability ** 2)),
                    "v1_safe_auc_3s": safe_auc(v1, safe),
                    "total_safe_auc_3s": safe_auc(result["weighted"], safe),
                    "returned_control": result["returned_control"],
                    "body_min_gap_m": float(body_gap[index].min()),
                    "body_min_gap_step": int(body_gap[index].argmin() + 1),
                    "padded_min_gap_m": float(padded_gap[index].min())}
            if goal is not None:
                item["endpoint_goal_position_distance_m"] = float(np.hypot(
                    output_poses[index, -1, 0] - goal[0],
                    output_poses[index, -1, 1] - goal[1]))
            outputs.append(item)
        cases.append({"name": name, "cycle_id": cycle_id, "seed": seed,
                      "input_sha256": {"cycle": digest(cycle),
                                       "cycle_bin": digest(cycle.with_suffix(".bin")),
                                       "profile": digest(profile_path),
                                       "truth": digest(truth_path),
                                       "standard": digest(score_path),
                                       "raw_controls": digest(raw_path)
                                       if raw_path else None,
                                       "reference_controls": digest(reference_path),
                                       "labels": digest(label_path)},
                      "file_sha256": {path.name: digest(path) for path in (
                          csv_path, control_path, pose_path, static_path)},
                      "hard_collision_rollouts": int(hard.sum()),
                      "hard_hits_by_step": hits,
                      "baseline_control_max_abs_error": baseline_error,
                      "joint_safe_filtered_candidates": int(safe.sum()),
                      "truth_overlap_nonzero_candidates": int(
                          np.count_nonzero(truth_overlap)),
                      "truth_near_nonzero_candidates": int(
                          np.count_nonzero(truth_near)),
                      "outputs": outputs})
    report = {"schema": "rm_dynamic_prediction/truth_geometry_score_probe/v1",
              "scope": "Five preselected frozen 300-rollout inputs. Offline future Gazebo box truth replaces only V1 continuous term on candidates in original hard branch, first with exact padded overlap, then with existing 0.02m padded-near threshold and 300-unit near penalty per step. Original nine-step hard envelope, non-hard branch, seven native standard critics, controls, MPPI regularizer/temperature, final filter, footprint/padding and safety gates remain. This oracle is impossible online and is a score-representation ceiling only. Static masks pending.",
              "raw_inputs_sha256": digest(args.raw_inputs),
              "hard_unit": float(hard_unit), "near_unit": float(near_unit),
              "cases": cases}
    (args.work_dir / "prepared.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(cases), "work_dir": str(args.work_dir)}))


def finalize(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    report = json.loads(args.prepared.read_text())
    args.output_dir.mkdir(parents=True)
    for row in report["cases"]:
        name = row["name"]
        directory = args.prepared.parent / name
        for filename, expected in row["file_sha256"].items():
            if digest(directory / filename) != expected:
                raise ValueError(f"prepared artifact changed: {name}/{filename}")
        mask = args.mask_dir / f"{name}_mask.txt"
        values = [int(value) for value in mask.read_text().split()]
        if len(values) != 3 or any(value not in (0, 1) for value in values):
            raise ValueError(f"native static mask malformed: {name}")
        row["native_static_mask_sha256"] = digest(mask)
        for item, collision in zip(row["outputs"], values):
            item["costcritic_collision"] = bool(collision)
            item["joint_gate_met"] = bool(item["body_min_gap_m"] >= .05 and
                                          item["padded_min_gap_m"] > 0 and
                                          not collision)
        shutil.copyfile(directory / "candidates.csv",
                        args.output_dir / f"{name}_candidates.csv")
    report["scope"] = report["scope"].replace(
        "Static masks pending.",
        "Original raw-costmap native CostCritic static masks checked.")
    report["prepared_sha256"] = digest(args.prepared)
    report["native_mask_binary_sha256"] = digest(args.mask_binary)
    (args.output_dir / "summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["cases"]),
                      "output_dir": str(args.output_dir)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    for field in ("collision-trial", "goal-trial", "raw-inputs", "score147",
                  "score162", "score263", "seed2-score", "seed3-score",
                  "work-dir"):
        p.add_argument("--" + field, type=Path, required=True)
    p = sub.add_parser("finalize")
    p.add_argument("--prepared", type=Path, required=True)
    p.add_argument("--mask-dir", type=Path, required=True)
    p.add_argument("--mask-binary", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "finalize": finalize}[args.command](args)
