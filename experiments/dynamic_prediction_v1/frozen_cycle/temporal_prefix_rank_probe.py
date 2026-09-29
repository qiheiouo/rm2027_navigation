#!/usr/bin/env python3
"""Screen a fixed four-step continuous V1 overlap on five frozen 300-rollout inputs.

The nine-step conservative hard envelope, native standard critics and raw
MPPI control aggregation are unchanged. Future truth is evaluation only.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from costmap_mask_fixture import export, shortcut_threshold
from filtered_batch_candidate_rank import raw_controls
from filtered_candidate_rank_audit import safe_auc
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_sensitivity import AXES, aggregate
from output_first_step_audit import actual, pose_array
from peak_temporal_rank_probe import (CASES, reference_controls, safe_labels,
                                      step_overlap)
import replay_ranking


CONTROL_HEADER = struct.Struct("<3I")


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    raw = json.loads(args.raw_inputs.read_text())
    evidence = Path("docs/dynamic_navigation/evidence")
    unit = np.float32((3.81 / 254.) * 1_000_000. / 9)
    rows = []
    for name, cycle_id, trial_key, seed, score_key in CASES:
        trial, score_path = getattr(args, trial_key), getattr(args, score_key)
        cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
        profile_path = trial / "profile.yaml"
        truth_path = trial / "gazebo_poses.jsonl"
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        if (meta["cycle_id"], settings["batch"], settings["steps"]) != (
                cycle_id, 300, 30):
            raise ValueError("frozen MPPI input differs")
        profile = yaml.safe_load(profile_path.read_text())
        params = profile["controller_server"]["ros__parameters"]["FollowPath"][
            "PredictionV1Critic"]
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
                raise ValueError("raw controls changed")
            controls = raw_controls(raw_path, 300)
        poses = filtered_poses(controls, meta, settings, history)
        original, hits, mean = filtered_prediction_score(meta, poses, params)
        by_step = step_overlap(meta, poses, params)
        if float(np.max(np.abs(by_step.mean(axis=1) - mean))) > 1e-8:
            raise ValueError("stepwise overlap no longer reproduces V1")
        hard = original > 100.
        first_four = by_step[:, :4].mean(axis=1)
        candidate_v1 = original + np.float32(unit * (first_four - mean) * hard)
        standard = np.fromfile(score_path, dtype="<f4")
        if standard.shape != (300,):
            raise ValueError("native standard critic dimensions differ")
        baseline = aggregate(standard + original, controls, initial,
                             settings, history)
        changed = aggregate(standard + candidate_v1, controls, initial,
                            settings, history)
        reference, reference_path = reference_controls(name, evidence)
        baseline_error = float(np.max(np.abs(
            baseline["filtered_sequence"] - reference)))
        if baseline_error > 3e-5:
            raise ValueError(f"published baseline does not replay: {name}")
        label_path = (evidence / "filtered_candidate_rank_20260928" /
                      f"{name}_detail.csv" if seed is None else
                      evidence / "filtered_batch_candidate_rank_20260928" /
                      f"seed_{seed}_batch2000_labels.csv")
        safe = safe_labels(label_path)
        target = args.output_dir / name
        target.mkdir()
        overlap_path = target / "step_overlap.bin"
        by_step.astype("<f8").tofile(overlap_path)
        control_path = target / "output_controls.bin"
        output_controls = np.asarray((baseline["filtered_sequence"],
                                      changed["filtered_sequence"]), dtype="<f4")
        control_path.write_bytes(CONTROL_HEADER.pack(0x43545231, 2, 30) +
                                 output_controls.tobytes())
        output_poses = pose_array(output_controls, meta, settings, True)
        pose_path = target / "output_poses.bin"
        output_poses.tofile(pose_path)
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        boxes = actual(meta, settings, truth_path)
        body_gap = np.asarray([[analyze.polygon_distance(
            analyze.placed(body, tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in output_poses])
        padded_gap = np.asarray([[analyze.polygon_distance(
            analyze.placed(meta["padded_footprint"], tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in output_poses])
        static_path = target / "static.bin"
        export(static_path, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(output_poses[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        goal = (json.loads((trial / "runtime_audit.json").read_text())[
            "navigation_result"]["goal"] if cycle_id == 263 else None)
        outputs = []
        for index, (label, v1, outcome) in enumerate((
                ("mean9", original, baseline),
                ("prefix4", candidate_v1, changed))):
            probability = outcome["probability"]
            item = {"name": label,
                    "top_index": int(np.argmin(outcome["weighted"])),
                    "top_candidate_joint_safe_3s": bool(safe[
                        np.argmin(outcome["weighted"])]),
                    "top_probability": float(np.max(probability)),
                    "safe_probability_mass": float(probability[safe].sum()),
                    "effective_sample_size": float(1. / np.sum(probability ** 2)),
                    "v1_safe_auc_3s": safe_auc(v1, safe),
                    "total_safe_auc_3s": safe_auc(outcome["weighted"], safe),
                    "returned_control": outcome["returned_control"],
                    "body_min_gap_m": float(body_gap[index].min()),
                    "body_min_gap_step": int(body_gap[index].argmin() + 1),
                    "body_min_gap_first_4_m": float(body_gap[index, :4].min()),
                    "body_min_gap_first_9_m": float(body_gap[index, :9].min()),
                    "padded_min_gap_m": float(padded_gap[index].min())}
            if goal is not None:
                item["endpoint_goal_position_distance_m"] = float(np.hypot(
                    output_poses[index, -1, 0] - goal[0],
                    output_poses[index, -1, 1] - goal[1]))
            outputs.append(item)
        rows.append({"name": name, "cycle_id": cycle_id, "seed": seed,
                     "input_sha256": {
                         "cycle": digest(cycle),
                         "cycle_bin": digest(cycle.with_suffix(".bin")),
                         "profile": digest(profile_path),
                         "truth": digest(truth_path),
                         "standard": digest(score_path),
                         "raw_controls": digest(raw_path) if raw_path else None,
                         "reference_controls": digest(reference_path),
                         "labels": digest(label_path)},
                     "file_sha256": {path.name: digest(path) for path in (
                         overlap_path, control_path, pose_path, static_path)},
                     "hard_collision_count": int(hard.sum()),
                     "hard_hits_by_step": hits,
                     "baseline_control_max_abs_error": baseline_error,
                     "joint_safe_filtered_candidate_count": int(safe.sum()),
                     "outputs": outputs})
    report = {"schema": "rm_dynamic_prediction/temporal_prefix_rank_probe/v1",
              "scope": "Five preselected 300-rollout frozen inputs. Only the continuous V1 overlap's temporal aggregation changes from mean of nine steps to mean of the first four; the original nine-step hard envelope, near term, standard critics, noise, controls, MPPI regularizer/temperature, final filter and safety gates remain unchanged. Truth labels candidates and outputs after scoring. Native-first Python integration previously validated against Nav2 C++; static masks pending.",
              "raw_inputs_sha256": digest(args.raw_inputs),
              "cases": rows}
    (args.output_dir / "prepared.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


def native(args):
    report = json.loads(args.prepared.read_text())
    for row in report["cases"]:
        target = args.prepared.parent / row["name"]
        fixture = target / "static.bin"
        if digest(fixture) != row["file_sha256"]["static.bin"]:
            raise ValueError("static fixture changed")
        done = subprocess.run([str(args.mask_binary), str(fixture),
                               str(target / "mask.txt")],
                              capture_output=True, text=True, check=False)
        if done.returncode:
            raise RuntimeError(f"{row['name']}: {done.stderr}")
    print(json.dumps({"native_static_masks": len(report["cases"])}))


def finalize(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    report = json.loads(args.prepared.read_text())
    for row in report["cases"]:
        target = args.prepared.parent / row["name"]
        for filename, expected in row["file_sha256"].items():
            if digest(target / filename) != expected:
                raise ValueError("prepared evidence changed")
        mask_path = target / "mask.txt"
        mask = np.loadtxt(mask_path, dtype=bool)
        if mask.shape != (2,):
            raise ValueError("native static mask differs")
        row["native_static_mask_sha256"] = digest(mask_path)
        for item, collision in zip(row["outputs"], mask):
            item["costcritic_collision"] = bool(collision)
            item["joint_gate_met"] = bool(item["body_min_gap_m"] >= .05 and
                                          item["padded_min_gap_m"] > 0 and
                                          not collision)
    report["scope"] = report["scope"].replace(
        "static masks pending.", "original raw-costmap native static masks checked.")
    report["prepared_sha256"] = digest(args.prepared)
    report["native_mask_binary_sha256"] = digest(args.mask_binary)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["cases"])}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    for field in ("collision-trial", "goal-trial", "raw-inputs", "score147",
                  "score162", "score263", "seed2-score", "seed3-score",
                  "output-dir"):
        p.add_argument("--" + field, type=Path, required=True)
    p = sub.add_parser("native")
    p.add_argument("--prepared", type=Path, required=True)
    p.add_argument("--mask-binary", type=Path, required=True)
    p = sub.add_parser("finalize")
    p.add_argument("--prepared", type=Path, required=True)
    p.add_argument("--mask-binary", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "native": native,
     "finalize": finalize}[args.command](args)
