#!/usr/bin/env python3
"""Offline soft-rank probe using view-reflected historical scan residuals.

Only four old T-DT trials fit the residual cloud. Frozen Navfn future truth is
used after scoring for candidate/output labels, never to choose model weights.
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
from historical_scan_residual_calibration import residual_rows
from native_critic_sensitivity import AXES, aggregate
from output_first_step_audit import actual, pose_array
from peak_temporal_rank_probe import CASES, reference_controls, safe_labels
from raw_scan_support_audit import static_map, trajectory_pose
from scan_midpoint_soft_probe import estimate_at_source, scan_sources
from view_reflection_calibration import reflected
import replay_ranking


HEADER = struct.Struct("<3I")


def historical_clouds(paths, occupancy, durations, training_message_source):
    clouds = []
    counts = []
    for path in paths:
        observations = reflected(residual_rows(
            path, occupancy, training_message_source, durations))
        by_step = []
        for duration in durations:
            residual = np.asarray([
                row["residual_xy_m"] for row in observations["rows"]
                if row["horizon_s"] == duration], dtype=np.float64)
            if residual.ndim != 2 or residual.shape[1] != 2 or len(residual) < 10:
                raise ValueError(f"historical residual support too small: {path}")
            by_step.append(residual)
        clouds.append(by_step)
        counts.append([len(item) for item in by_step])
    return clouds, counts


def target_view(trial, source, center):
    trajectory = [json.loads(line) for line in
                  (trial / "observation/trajectory.jsonl").open()]
    times = [row["t"] for row in trajectory]
    pose = trajectory_pose(trajectory, times, source)
    if pose is None:
        raise ValueError("no causal odometry at target scan source")
    return "north" if pose[1] > center[1] else "south"


def expected_near(meta, poses, params, source_center, velocity,
                  source_age, view, clouds):
    steps = len(clouds[0])
    dt = analyze.event(meta, "settings")["dt"]
    extent = np.asarray((params["object_width"], params["object_height"]))
    if not np.allclose(extent, (.45, .55), atol=1e-9):
        raise ValueError("known physical box dimensions changed")
    sign = -1. if view == "north" else 1.
    scores = np.zeros(len(poses), dtype=np.float64)
    for step in range(steps):
        duration = source_age + (step + 1) * dt
        nominal = source_center + velocity * duration
        trial_boxes = []
        for trial in clouds:
            residual = trial[step]
            centers = nominal[None, :] + residual * np.array((1., sign))
            trial_boxes.append([AxisBox(
                center[0] - extent[0] / 2, center[1] - extent[1] / 2,
                center[0] + extent[0] / 2, center[1] + extent[1] / 2)
                for center in centers])
        for index in range(len(poses)):
            robot = analyze.placed(meta["padded_footprint"],
                                   tuple(map(float, poses[index, step])))
            rx0 = min(point[0] for point in robot)
            rx1 = max(point[0] for point in robot)
            ry0 = min(point[1] for point in robot)
            ry1 = max(point[1] for point in robot)
            risk = 0.
            for boxes in trial_boxes:
                near = 0
                for box in boxes:
                    if (rx0 > box.max_x + .02 or rx1 < box.min_x - .02 or
                            ry0 > box.max_y + .02 or ry1 < box.min_y - .02):
                        continue
                    if analyze.polygon_distance(robot, box.polygon()) < .02:
                        near += 1
                risk += near / len(boxes)
            scores[index] += risk / len(trial_boxes)
    return scores


def prepare(args):
    if args.work_dir.exists():
        raise FileExistsError(args.work_dir)
    args.work_dir.mkdir(parents=True)
    evidence = Path("docs/dynamic_navigation/evidence")
    model = json.loads(args.model_evidence.read_text())
    paths = [Path(row["trial"]) for row in model["historical_leave_one_trial_out"]]
    if len(paths) != 6 or any("tdt_" not in path.name for path in paths[:4]):
        raise ValueError("four old T-DT fit trials are required")
    training = paths[:4]
    if any(path in (args.collision_trial, args.goal_trial) for path in training):
        raise ValueError("frozen target trial must not train residual model")
    occupancy, _ = static_map()
    scan_data = {trial: scan_sources(trial, occupancy)
                 for trial in (args.collision_trial, args.goal_trial)}
    raw = json.loads(args.raw_inputs.read_text())
    cloud_cache = {}
    rows = []
    for name, cycle_id, trial_key, seed, score_key in CASES:
        trial, score_path = getattr(args, trial_key), getattr(args, score_key)
        directory = args.work_dir / name
        directory.mkdir()
        cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
        profile_path = trial / "profile.yaml"
        truth_path = trial / "gazebo_poses.jsonl"
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        if (meta["cycle_id"], settings["batch"], settings["steps"]) != (
                cycle_id, 300, 30):
            raise ValueError("frozen MPPI case differs")
        profile = yaml.safe_load(profile_path.read_text())
        params = profile["controller_server"]["ros__parameters"][
            "FollowPath"]["PredictionV1Critic"]
        steps = int(np.floor(params["horizon"] / settings["dt"] + 1e-9))
        if steps != 9:
            raise ValueError("V1 horizon differs")
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
        previous = replay_ranking.history_from_trial(
            trial / "mppi_cycles", cycle_id, arrays)
        history = np.stack([previous[axis] for axis in AXES], axis=-1)
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1)
        poses = filtered_poses(controls, meta, settings, history)
        original, hits, original_overlap = filtered_prediction_score(
            meta, poses, params)
        hard = original > 100.
        scan_hashes, sources = scan_data[trial]
        scan, velocity, fallback, history_count = estimate_at_source(meta, sources)
        center = np.asarray(scan["raw_bbox_mid"], dtype=np.float64)
        velocity = np.asarray(velocity, dtype=np.float64)
        view = target_view(trial, scan["source_t"], center)
        predicted_near = None
        counts = None
        if not fallback:
            prediction = analyze.event(meta, "prediction.input")
            durations = tuple(prediction["source_age_s"] +
                              (step + 1) * settings["dt"]
                              for step in range(steps))
            if durations not in cloud_cache:
                cloud_cache[durations] = historical_clouds(
                    training, occupancy, durations,
                    args.training_message_source)
            clouds, counts = cloud_cache[durations]
            predicted_near = expected_near(
                meta, poses, params, center, velocity,
                prediction["source_age_s"], view, clouds)
            hard_unit = np.float32((3.81 / 254.) * 1_000_000. / steps)
            near_unit = np.float32((3.81 / 254.) * 300. / steps)
            candidate_v1 = original + np.float32(
                (near_unit * predicted_near -
                 hard_unit * original_overlap) * hard)
        else:
            candidate_v1 = original.copy()
        standard = np.fromfile(score_path, dtype="<f4")
        if standard.shape != (300,) or not np.isfinite(standard).all():
            raise ValueError("native standard scores differ")
        baseline = aggregate(standard + original, controls, initial,
                             settings, history)
        candidate = aggregate(standard + candidate_v1, controls, initial,
                              settings, history)
        reference, reference_path = reference_controls(name, evidence)
        replay_error = float(np.max(np.abs(
            baseline["filtered_sequence"] - reference)))
        if replay_error > 3e-5:
            raise ValueError(f"published baseline differs: {name}")
        if fallback and not np.array_equal(
                baseline["filtered_sequence"], candidate["filtered_sequence"]):
            raise ValueError("no-velocity fallback changed output")
        # Only target scan fields enter the score. The scan helper also computes
        # separate truth-audit fields, which are ignored above; future truth
        # labels and output geometry are evaluated below.
        label_path = (evidence / "filtered_candidate_rank_20260928" /
                      f"{name}_detail.csv" if seed is None else
                      evidence / "filtered_batch_candidate_rank_20260928" /
                      f"seed_{seed}_batch2000_labels.csv")
        safe = safe_labels(label_path)
        controls_out = np.asarray((baseline["filtered_sequence"],
                                   candidate["filtered_sequence"]), dtype="<f4")
        poses_out = pose_array(controls_out, meta, settings, True)
        (directory / "output_controls.bin").write_bytes(
            HEADER.pack(0x43545231, 2, 30) + controls_out.tobytes())
        poses_out.tofile(directory / "output_poses.bin")
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        boxes = actual(meta, settings, truth_path)
        body_gaps = np.asarray([[analyze.polygon_distance(
            analyze.placed(body, tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in poses_out])
        padded_gaps = np.asarray([[analyze.polygon_distance(
            analyze.placed(meta["padded_footprint"], tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in poses_out])
        static_path = directory / "static.bin"
        export(static_path, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(poses_out[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        risk_path = directory / "expected_near_f64.bin"
        if predicted_near is not None:
            predicted_near.astype("<f8").tofile(risk_path)
        with (directory / "candidates.csv").open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("index", "joint_safe_3s", "hard_v1",
                             "original_overlap_mean", "expected_near_count_9",
                             "original_v1_score", "candidate_v1_score",
                             "baseline_total", "candidate_total"))
            for index in range(300):
                writer.writerow((index, int(safe[index]), int(hard[index]),
                                 float(original_overlap[index]),
                                 float(predicted_near[index])
                                 if predicted_near is not None else "",
                                 float(original[index]),
                                 float(candidate_v1[index]),
                                 float(baseline["weighted"][index]),
                                 float(candidate["weighted"][index])))
        goal = (json.loads((trial / "runtime_audit.json").read_text())[
            "navigation_result"]["goal"] if cycle_id == 263 else None)
        outputs = []
        for index, (label, v1, state) in enumerate((
                ("baseline", original, baseline),
                ("reflected_residual_near", candidate_v1, candidate))):
            probability = state["probability"]
            top = int(np.argmin(state["weighted"]))
            result = {"name": label, "top_index": top,
                      "top_candidate_joint_safe_3s": bool(safe[top]),
                      "safe_probability_mass": float(probability[safe].sum()),
                      "effective_sample_size": float(1. /
                          np.sum(probability ** 2)),
                      "v1_safe_auc_3s": safe_auc(v1, safe),
                      "total_safe_auc_3s": safe_auc(state["weighted"], safe),
                      "returned_control": state["returned_control"],
                      "body_min_gap_m": float(body_gaps[index].min()),
                      "body_min_gap_step": int(body_gaps[index].argmin() + 1),
                      "padded_min_gap_m": float(padded_gaps[index].min())}
            if goal is not None:
                result["endpoint_goal_position_distance_m"] = float(np.hypot(
                    poses_out[index, -1, 0] - goal[0],
                    poses_out[index, -1, 1] - goal[1]))
            outputs.append(result)
        files = [directory / "output_controls.bin",
                 directory / "output_poses.bin", directory / "static.bin",
                 directory / "candidates.csv"]
        if predicted_near is not None:
            files.append(risk_path)
        rows.append({"name": name, "cycle_id": cycle_id, "seed": seed,
                     "scan_source_t": scan["source_t"],
                     "scan_center_xy": center.tolist(),
                     "scan_velocity_xy": velocity.tolist(),
                     "scan_recent_count": history_count,
                     "scan_velocity_fallback": fallback,
                     "observer_view_y": view,
                     "historical_cloud_counts_by_trial_and_step": counts,
                     "source_sha256": {"cycle": digest(cycle),
                                       "cycle_bin": digest(cycle.with_suffix(".bin")),
                                       "profile": digest(profile_path),
                                       "truth": digest(truth_path),
                                       "standard": digest(score_path),
                                       "raw_controls": digest(raw_path)
                                       if raw_path else None,
                                       "reference_controls": digest(reference_path),
                                       "labels": digest(label_path),
                                       "scans": scan_hashes},
                     "file_sha256": {path.name: digest(path) for path in files},
                     "baseline_control_max_abs_error": replay_error,
                     "hard_collision_rollouts": int(hard.sum()),
                     "hard_hits_by_step": hits,
                     "joint_safe_filtered_candidates": int(safe.sum()),
                     "outputs": outputs})
    report = {"schema": "rm_dynamic_prediction/reflected_residual_rank_probe/v1",
              "scope": "Five preselected frozen 300-rollout inputs. Four old T-DT trials only fit joint xy future-center residual clouds at each exact source-age-adjusted V1 time step. Training scan projection uses " + args.training_message_source + ". Mirror historical y residual by observer side and unmirror at target side; equal mass per training trial. Target source scan midpoint and past <=0.4s fitted velocity use recorded scans and odometry. If fewer than three source scans (147), retain original V1. Only the continuous term for candidates already in original hard branch is replaced by expected true-size-box padded-near (<0.02m) count with existing 300-unit near scale; original hard envelope, non-hard branch, seven standard critics, controls, temperature, filter and safety gates unchanged. The scan reconstruction helper computes separate target truth audit fields but these are excluded from scoring; target future truth labels and output geometry are evaluated afterward. No runtime probability or safety claim. Native output masks pending.",
              "model_evidence_sha256": digest(args.model_evidence),
              "training_message_source": args.training_message_source,
              "raw_inputs_sha256": digest(args.raw_inputs),
              "cases": rows}
    (args.work_dir / "prepared.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "work_dir": str(args.work_dir)}))


def finalize(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    report = json.loads(args.prepared.read_text())
    args.output_dir.mkdir(parents=True)
    for row in report["cases"]:
        directory = args.prepared.parent / row["name"]
        for filename, expected in row["file_sha256"].items():
            if digest(directory / filename) != expected:
                raise ValueError(f"prepared artifact changed: {row['name']}/{filename}")
        mask = args.mask_dir / f"{row['name']}_mask.txt"
        values = [int(value) for value in mask.read_text().split()]
        if len(values) != 2 or any(value not in (0, 1) for value in values):
            raise ValueError("native static mask malformed")
        row["native_static_mask_sha256"] = digest(mask)
        for result, collision in zip(row["outputs"], values):
            result["costcritic_collision"] = bool(collision)
            result["joint_gate_met"] = bool(
                result["body_min_gap_m"] >= .05 and
                result["padded_min_gap_m"] > 0 and not collision)
        shutil.copyfile(directory / "candidates.csv",
                        args.output_dir / f"{row['name']}_candidates.csv")
        if "expected_near_f64.bin" in row["file_sha256"]:
            shutil.copyfile(directory / "expected_near_f64.bin",
                            args.output_dir / f"{row['name']}_expected_near_f64.bin")
    report["scope"] = report["scope"].replace(
        "Native output masks pending.",
        "Original raw-costmap native CostCritic output masks checked.")
    report["prepared_sha256"] = digest(args.prepared)
    report["mask_binary_sha256"] = digest(args.mask_binary)
    (args.output_dir / "summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["cases"]),
                      "output_dir": str(args.output_dir)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    for field in ("collision-trial", "goal-trial", "model-evidence",
                  "raw-inputs", "score147", "score162", "score263",
                  "seed2-score", "seed3-score", "work-dir"):
        p.add_argument("--" + field, type=Path, required=True)
    p.add_argument("--training-message-source", choices=("recorded", "recorded_odom"),
                   default="recorded")
    p = sub.add_parser("finalize")
    for field in ("prepared", "mask-dir", "mask-binary", "output-dir"):
        p.add_argument("--" + field, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "finalize": finalize}[args.command](args)
