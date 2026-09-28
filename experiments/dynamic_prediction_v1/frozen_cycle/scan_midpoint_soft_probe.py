#!/usr/bin/env python3
"""Offline point-center risk diagnostic from causal near-full raw scans.

The original hard envelope is retained. This is an input ablation, not a
calibrated probability model or a runtime tracker replacement.
"""
import argparse
import json
from pathlib import Path

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
from peak_temporal_rank_probe import CASES, CONTROL_HEADER, reference_controls, safe_labels
from raw_scan_support_audit import (fitted_velocity, near_full_span, one_trial,
                                    static_map, training_messages)
import rank_probe
import replay_ranking


def scan_sources(trial, occupancy):
    audit, rows = one_trial(trial, training_messages(trial), occupancy,
                            return_source_rows=True, pose_source="recorded_odom")
    qualifying = [row for row in rows if near_full_span(row)]
    return audit["source_sha256"], qualifying


def estimate_at_source(meta, sources):
    prediction = analyze.event(meta, "prediction.input")
    source_t = prediction["source_stamp_ns"] / 1e9
    match = [row for row in sources if abs(row["source_t"] - source_t) < 1e-6]
    if len(match) != 1:
        raise ValueError(f"no unique near-full scan for cycle {meta['cycle_id']}")
    row = match[0]
    recent = [old for old in sources if
              0 <= source_t - old["source_t"] <= .4][-5:]
    track = next(track for track in prediction["tracks"] if track["state"] == 2)
    velocity = fitted_velocity(recent) if len(recent) >= 3 else None
    fallback = velocity is None
    if fallback:
        velocity = track["vxy"]
    return row, np.asarray(velocity, dtype=np.float64), fallback, len(recent)


def point_overlap(meta, poses, params, center, velocity):
    prediction = analyze.event(meta, "prediction.input")
    settings = analyze.event(meta, "settings")
    if poses.shape != (300, 30, 3):
        raise ValueError("not the frozen 300-rollout input")
    steps = int(np.floor(params["horizon"] / settings["dt"] + 1e-9))
    if steps != 9:
        raise ValueError("not the frozen nine-step horizon")
    extent = np.array((params["object_width"], params["object_height"]))
    area = rank_probe.area(meta["padded_footprint"])
    overlap = np.zeros(300, dtype=np.float64)
    for step in range(steps):
        duration = prediction["source_age_s"] + (step + 1) * settings["dt"]
        xy = center + velocity * duration
        box = AxisBox(xy[0] - extent[0] / 2, xy[1] - extent[1] / 2,
                      xy[0] + extent[0] / 2, xy[1] + extent[1] / 2)
        for index in range(300):
            polygon = analyze.placed(meta["padded_footprint"],
                                     tuple(map(float, poses[index, step])))
            overlap[index] += rank_probe.overlap_area(polygon, box) / area / steps
    return overlap


def run(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    occupancy, _ = static_map()
    source_data = {}
    for trial in (args.collision_trial, args.goal_trial):
        source_data[trial] = scan_sources(trial, occupancy)
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
            raise ValueError(f"frozen settings differ: {name}")
        profile = yaml.safe_load(profile_path.read_text())
        params = profile["controller_server"]["ros__parameters"][
            "FollowPath"]["PredictionV1Critic"]
        scan_hashes, sources = source_data[trial]
        scan, velocity, fallback, history_count = estimate_at_source(meta, sources)
        center = np.asarray(scan["raw_bbox_mid"], dtype=np.float64)
        if seed is None:
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
            raw_controls_path = None
        else:
            source = next(case for case in raw["cases"] if case["name"] == name)
            raw_controls_path = Path(source["controls"])
            controls = raw_controls(raw_controls_path, 300)
        previous = replay_ranking.history_from_trial(
            trial / "mppi_cycles", cycle_id, arrays)
        history = np.stack([previous[axis] for axis in AXES], axis=-1)
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1)
        poses = filtered_poses(controls, meta, settings, history)
        original, hits, mean = filtered_prediction_score(meta, poses, params)
        point = point_overlap(meta, poses, params, center, velocity)
        hard = original > 100.
        if np.any((original >= 100.) & (original < 1000.)):
            raise ValueError(f"unexpected frozen V1 score scale: {name}")
        changed = original + np.float32(unit * (point - mean) * hard)
        standard = np.fromfile(score_path, dtype="<f4")
        if standard.shape != (300,) or not np.isfinite(standard).all():
            raise ValueError(f"native standard score differs: {name}")
        baseline = aggregate(standard + original, controls, initial,
                             settings, history)
        candidate = aggregate(standard + changed, controls, initial,
                              settings, history)
        reference, reference_path = reference_controls(name, evidence)
        baseline_error = float(np.max(np.abs(
            baseline["filtered_sequence"] - reference)))
        if baseline_error > 3e-5:
            raise ValueError(f"published baseline differs: {name}, {baseline_error}")
        if seed is None:
            label_path = (evidence / "filtered_candidate_rank_20260928" /
                          f"{name}_detail.csv")
        else:
            label_path = (evidence / "filtered_batch_candidate_rank_20260928" /
                          f"seed_{seed}_batch2000_labels.csv")
        safe = safe_labels(label_path)
        directory = args.output_dir / name
        directory.mkdir()
        point_path = directory / "point_overlap.bin"
        point.astype("<f8").tofile(point_path)
        sequences = np.asarray((baseline["filtered_sequence"],
                                candidate["filtered_sequence"]), dtype="<f4")
        poses_output = pose_array(sequences, meta, settings, True)
        (directory / "output_controls.bin").write_bytes(
            CONTROL_HEADER.pack(0x43545231, 2, 30) + sequences.tobytes())
        poses_output.tofile(directory / "output_poses.bin")
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        boxes = actual(meta, settings, truth_path)
        gaps = np.asarray([[analyze.polygon_distance(
            analyze.placed(body, tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in poses_output])
        padded_gaps = np.asarray([[analyze.polygon_distance(
            analyze.placed(meta["padded_footprint"], tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in poses_output])
        fixture = directory / "static.bin"
        export(fixture, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(poses_output[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        result = {"name": name, "cycle_id": cycle_id, "seed": seed,
                  "scan_source_stamp_s": scan["source_t"],
                  "scan_raw_mid_xy_m": scan["raw_bbox_mid"],
                  "scan_raw_span_xy_m": scan["raw_span"],
                  "scan_fitted_velocity_xy_mps": velocity.tolist(),
                  "scan_velocity_track_fallback": fallback,
                  "scan_recent_count": history_count,
                  "scan_source_truth_error_xy_m_for_audit_only":
                      scan["raw_bbox_mid_error"],
                  "input_sha256": {
                      "cycle": digest(cycle),
                      "cycle_bin": digest(cycle.with_suffix(".bin")),
                      "profile": digest(profile_path),
                      "truth": digest(truth_path),
                      "standard": digest(score_path),
                      "raw_controls": digest(raw_controls_path) if seed is not None else None,
                      "reference_controls": digest(reference_path),
                      "labels": digest(label_path),
                      "reconstructed_scan_sources": scan_hashes},
                  "file_sha256": {path.name: digest(path) for path in (
                      point_path, directory / "output_controls.bin",
                      directory / "output_poses.bin", fixture)},
                  "hard_collision_rollouts": int(hard.sum()),
                  "hard_hits_by_step": hits,
                  "baseline_control_max_abs_error": baseline_error,
                  "point_overlap_min": float(point.min()),
                  "point_overlap_max": float(point.max()),
                  "joint_safe_filtered_candidates": int(safe.sum()),
                  "outputs": []}
        if cycle_id == 263:
            goal = json.loads((trial / "runtime_audit.json").read_text())[
                "navigation_result"]["goal"]
        for label, score, ranked, index in (("uniform", original, baseline, 0),
                                            ("scan_point", changed, candidate, 1)):
            weight = ranked["probability"]
            item = {"name": label,
                    "v1_safe_auc": safe_auc(score, safe),
                    "total_safe_auc": safe_auc(ranked["weighted"], safe),
                    "top_candidate_joint_safe": bool(safe[np.argmax(weight)]),
                    "safe_probability_mass": float(weight[safe].sum()),
                    "effective_sample_size": float(1. / np.sum(weight ** 2)),
                    "returned_control": ranked["returned_control"],
                    "body_min_gap_m": float(gaps[index].min()),
                    "body_min_gap_step": int(gaps[index].argmin() + 1),
                    "padded_min_gap_m": float(padded_gaps[index].min())}
            if cycle_id == 263:
                item["endpoint_goal_distance_m"] = float(np.hypot(
                    poses_output[index, -1, 0] - goal[0],
                    poses_output[index, -1, 1] - goal[1]))
            result["outputs"].append(item)
        rows.append(result)
    report = {"schema": "rm_dynamic_prediction/scan_midpoint_soft_probe/v1",
              "scope": "Five preselected frozen 300-rollout inputs. Causally reconstructed near-full raw scan midpoint and recent <=0.4s last-five midpoint velocity (>=3 frames, otherwise captured track velocity) replace only the soft center distribution with a point estimate. Original hard conservative V1 envelope and near term, seven native standard critics, MPPI aggregation/filter, controls and safety thresholds remain. Recorded simulation odometry is used to project scan, not future box truth; future truth labels only. Point estimate is not a safety bound or calibrated distribution. Native static mask pending.",
              "raw_inputs_sha256": digest(args.raw_inputs),
              "cases": rows}
    (args.output_dir / "prepared.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


def finalize(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    report = json.loads(args.prepared.read_text())
    for row in report["cases"]:
        name = row["name"]
        directory = args.prepared.parent / name
        for filename, expected in row["file_sha256"].items():
            if digest(directory / filename) != expected:
                raise ValueError(f"frozen output changed: {name}, {filename}")
        mask = args.mask_dir / f"{name}_mask.txt"
        values = [int(value) for value in mask.read_text().split()]
        if len(values) != 2 or any(value not in (0, 1) for value in values):
            raise ValueError(f"native static mask malformed: {name}")
        row["static_mask_sha256"] = digest(mask)
        for output, collision in zip(row["outputs"], values):
            output["costcritic_collision"] = bool(collision)
            output["joint_gate_met"] = bool(
                output["body_min_gap_m"] >= .05 and
                output["padded_min_gap_m"] > 0 and not collision)
    report["scope"] = report["scope"].replace(
        "Native static mask pending.", "Original raw-costmap native static mask checked.")
    report["prepared_sha256"] = digest(args.prepared)
    report["native_mask_binary_sha256"] = digest(args.mask_binary)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["cases"])}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("collision-trial", "goal-trial", "raw-inputs", "score147",
                 "score162", "score263", "seed2-score", "seed3-score",
                 "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    done = sub.add_parser("finalize")
    for name in ("prepared", "mask-dir", "mask-binary", "output"):
        done.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": run, "finalize": finalize}[args.command](args)
