#!/usr/bin/env python3
"""Nested MPPI batch ceiling with future-truth padded proximity at cycle 162.

All scores use the already verified guarded native filtered-candidate critics.
Future truth appears only in this offline oracle feature and final labels.
"""
import argparse
import csv
import json
from pathlib import Path
import shutil

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from native_critic_sensitivity import AXES, aggregate
from output_first_step_audit import actual, pose_array
from truth_geometry_score_probe import truth_features
import replay_ranking


def prepare(args):
    if args.work_dir.exists():
        raise FileExistsError(args.work_dir)
    args.work_dir.mkdir(parents=True)
    cycle = args.trial / "mppi_cycles/cycle_162.json"
    profile_path = args.trial / "profile.yaml"
    truth_path = args.trial / "gazebo_poses.jsonl"
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"],
            settings["iterations"]) != (162, 300, 30, 1):
        raise ValueError("not the frozen 162 input")
    profile = yaml.safe_load(profile_path.read_text())
    params = profile["controller_server"]["ros__parameters"][
        "FollowPath"]["PredictionV1Critic"]
    steps = int(np.floor(params["horizon"] / settings["dt"] + 1e-9))
    if steps != 9:
        raise ValueError("V1 horizon changed")
    boxes = actual(meta, settings, truth_path)
    prior = replay_ranking.history_from_trial(
        args.trial / "mppi_cycles", 162, arrays)
    history = np.stack([prior[axis] for axis in AXES], axis=-1)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    guarded = json.loads(args.guarded_manifest.read_text())
    if guarded["schema"] != "rm_dynamic_prediction/guarded_filtered_score_replay/v1":
        raise ValueError("guarded filtered-score audit differs")
    guarded_by_name = {row["name"]: row for row in guarded["cases"]}
    published = json.loads(args.published_summary.read_text())
    published_by_name = {row["name"]: row for row in published["rows"]}
    hard_unit = np.float32((3.81 / 254.) * 1_000_000. / steps)
    near_unit = np.float32((3.81 / 254.) * 300. / steps)
    output_sequences = []
    rows = []
    for seed in range(4):
        sampled, trajectory = sample_omni(meta, arrays, 2000, seed)
        if any(float(np.ptp(trajectory[axis][:, 0])) > 1e-6
               for axis in range(3)):
            raise ValueError("first-step pose is not common to all samples")
        controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
        poses = filtered_poses(controls, meta, settings, history)
        first = analyze.placed(meta["padded_footprint"],
                               tuple(map(float, poses[0, 0])))
        prediction = analyze.event(meta, "prediction.input")
        track = next(track for track in prediction["tracks"]
                     if track["state"] == 2)
        if sum(track["state"] == 2 for track in prediction["tracks"]) != 1:
            raise ValueError("single confirmed obstacle required")
        hard_box = analyze.predicted_box(
            track["xy"], track["vxy"], track["size_xy"],
            (params["object_width"], params["object_height"]),
            prediction["source_age_s"], settings["dt"],
            params["reference_acceleration"]).polygon()
        if analyze.polygon_distance(first, hard_box) > 1e-9:
            raise ValueError("not the saturated first-step V1 fixture")
        _, near, min_gap = truth_features(meta, poses, boxes, steps)
        near_path = args.work_dir / f"seed_{seed}_truth_near_i16.bin"
        near.astype("<i2").tofile(near_path)
        overlap_path = args.overlap_dir / f"seed_{seed}_batch2000_filtered_risk.bin"
        overlap = np.fromfile(overlap_path, dtype="<f8")
        if overlap.shape != (2000,) or not np.isfinite(overlap).all():
            raise ValueError("frozen continuous overlap differs")
        original = np.asarray((3.81 / 254.) * 1_000_000. *
                              (1. + overlap) / steps, dtype=np.float32)
        oracle = original + np.float32(near_unit * near - hard_unit * overlap)
        label_path = args.label_dir / f"seed_{seed}_batch2000_labels.csv"
        with label_path.open(newline="") as stream:
            labels = list(csv.DictReader(stream))
        if len(labels) != 2000 or any(int(row["rollout"]) != index
                                      for index, row in enumerate(labels)):
            raise ValueError("candidate labels differ")
        safe = np.asarray([row["joint_safe_3s"] == "1" for row in labels])
        for batch in BATCHES:
            name = f"seed_{seed}_batch{batch}"
            score_path = args.score_dir / f"{name}_scores.bin"
            standard = np.fromfile(score_path, dtype="<f4")
            if standard.shape != (batch,) or not np.isfinite(standard).all():
                raise ValueError(f"native standard score differs: {name}")
            if (guarded_by_name[name]["old_score_sha256"] != digest(score_path)
                    or guarded_by_name[name]["guarded_score_sha256"] !=
                    digest(score_path)):
                raise ValueError(f"same-fixture guard score mismatch: {name}")
            baseline = aggregate(standard + original[:batch],
                                 controls[:batch], initial, settings, history)
            changed = aggregate(standard + oracle[:batch],
                                controls[:batch], initial, settings, history)
            reference = published_by_name[name]["all_critic_returned_control"]
            replay_error = float(np.max(np.abs(
                np.asarray(baseline["returned_control"]) - reference)))
            if replay_error > 2e-5:
                raise ValueError(f"published baseline differs: {name}")
            if seed in (2, 3) and batch == 300:
                with (args.oracle300_dir /
                      f"seed_{seed}_batch300_candidates.csv").open(newline="") as stream:
                    reference_rows = list(csv.DictReader(stream))
                expected = np.asarray([float(row["truth_near_total"])
                                       for row in reference_rows])
                if float(np.max(np.abs(changed["weighted"] - expected))) > 1e-4:
                    raise ValueError(f"previous oracle scores differ: {name}")
            sequences = np.asarray((baseline["filtered_sequence"],
                                    changed["filtered_sequence"]), dtype="<f4")
            output_poses = pose_array(sequences, meta, settings, True)
            output_sequences.extend(list(output_poses))
            results = []
            for index, (kind, state) in enumerate((
                    ("baseline", baseline), ("truth_near", changed))):
                probability = state["probability"]
                top = int(np.argmin(state["weighted"]))
                body_gaps = [analyze.polygon_distance(
                    analyze.placed(body, tuple(map(float, pose))), box)
                    for pose, box in zip(output_poses[index], boxes)]
                padded_gaps = [analyze.polygon_distance(
                    analyze.placed(meta["padded_footprint"],
                                   tuple(map(float, pose))), box)
                    for pose, box in zip(output_poses[index], boxes)]
                results.append({
                    "name": kind, "top_index": top,
                    "top_candidate_joint_safe_3s": bool(safe[top]),
                    "top_candidate_body_gap_3s_m": float(
                        labels[top]["body_gap_3s_m"]),
                    "safe_probability_mass": float(
                        probability[:batch][safe[:batch]].sum()),
                    "effective_sample_size": float(1. /
                        np.sum(probability ** 2)),
                    "returned_control": state["returned_control"],
                    "body_min_gap_m": float(min(body_gaps)),
                    "body_min_gap_step": int(np.argmin(body_gaps) + 1),
                    "padded_min_gap_m": float(min(padded_gaps)),
                    "endpoint_path_distance_m": float(np.hypot(
                        output_poses[index, -1, 0] - meta["path"][-1][0],
                        output_poses[index, -1, 1] - meta["path"][-1][1]))})
            rows.append({"name": name, "seed": seed, "batch": batch,
                         "joint_safe_filtered_candidates": int(safe[:batch].sum()),
                         "best_joint_safe_body_gap_m": max(
                             (float(labels[i]["body_gap_3s_m"])
                              for i in range(batch) if safe[i]), default=None),
                         "truth_near_nonzero_candidates": int(np.count_nonzero(
                             near[:batch])),
                         "truth_min_padded_gap_best_m": float(
                             np.max(min_gap[:batch])),
                         "native_standard_sha256": digest(score_path),
                         "published_baseline_control_error": replay_error,
                         "outputs": results})
    output_poses = np.asarray(output_sequences, dtype="<f4")
    if output_poses.shape != (32, 30, 3):
        raise ValueError("expected 16 pairs of output trajectories")
    fixture = args.work_dir / "output_static.bin"
    export(fixture, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(output_poses[:, :, axis] for axis in range(3)),
           shortcut_threshold(meta, profile_path))
    report = {"schema": "rm_dynamic_prediction/truth_near_batch_probe/v1",
              "scope": "Cycle 162 only, four preselected seeds and nested 300/600/1000/2000 batches. Original V1 first-step hard tie retained; its uniform-center overlap is replaced by the offline true-box padded gap <0.02m count using original 300-unit near scale. Same guarded seven native filtered-candidate standard critics, original controls, temperature, regularizer and final filter. Future truth impossible online. Static output masks pending.",
              "source_sha256": {"cycle": digest(cycle),
                                "cycle_bin": digest(cycle.with_suffix(".bin")),
                                "profile": digest(profile_path),
                                "truth": digest(truth_path),
                                "guarded_manifest": digest(args.guarded_manifest),
                                "published_summary": digest(args.published_summary)},
              "hard_unit": float(hard_unit), "near_unit": float(near_unit),
              "fixture_sha256": digest(fixture),
              "near_files_sha256": {f"seed_{seed}": digest(
                  args.work_dir / f"seed_{seed}_truth_near_i16.bin")
                  for seed in range(4)},
              "rows": rows}
    (args.work_dir / "prepared.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"groups": len(rows), "work_dir": str(args.work_dir)}))


def finalize(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    report = json.loads(args.prepared.read_text())
    fixture = args.prepared.parent / "output_static.bin"
    if digest(fixture) != report["fixture_sha256"]:
        raise ValueError("output static fixture changed")
    mask = np.loadtxt(args.mask, dtype=bool)
    if mask.shape != (32,):
        raise ValueError("native static mask count differs")
    args.output_dir.mkdir(parents=True)
    index = 0
    for row in report["rows"]:
        for result in row["outputs"]:
            result["costcritic_collision"] = bool(mask[index])
            result["joint_gate_met"] = bool(
                result["body_min_gap_m"] >= .05 and
                result["padded_min_gap_m"] > 0 and not mask[index])
            index += 1
    report["scope"] = report["scope"].replace(
        "Static output masks pending.",
        "Original raw-costmap native CostCritic output masks checked.")
    report["prepared_sha256"] = digest(args.prepared)
    report["mask_sha256"] = digest(args.mask)
    report["mask_binary_sha256"] = digest(args.mask_binary)
    for seed in range(4):
        filename = f"seed_{seed}_truth_near_i16.bin"
        source = args.prepared.parent / filename
        if digest(source) != report["near_files_sha256"][f"seed_{seed}"]:
            raise ValueError("truth near array changed")
        shutil.copyfile(source, args.output_dir / filename)
    (args.output_dir / "summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"groups": len(report["rows"]),
                      "output_dir": str(args.output_dir)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    for field in ("trial", "score-dir", "overlap-dir", "label-dir",
                  "guarded-manifest", "published-summary", "oracle300-dir",
                  "work-dir"):
        p.add_argument("--" + field, type=Path, required=True)
    p = sub.add_parser("finalize")
    for field in ("prepared", "mask", "mask-binary", "output-dir"):
        p.add_argument("--" + field, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "finalize": finalize}[args.command](args)
