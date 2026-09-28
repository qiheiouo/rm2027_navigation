#!/usr/bin/env python3
"""Audit deterministic batch prefixes with native standard Nav2 critic scores.

This is an offline score/aggregation probe, not a full controller-cycle replay.
The V1 term is omitted only because the frozen first-step predicted collision
gives every rollout the same hard-collision score in this specific cycle.
"""
import argparse
import json
from pathlib import Path
import re

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni
import replay_ranking


AXES = ("vx", "vy", "wz")


def captured_history(cycle):
    stem = cycle.stem
    number = int(stem.removeprefix("cycle_"))
    if stem != f"cycle_{number}":
        raise ValueError("cycle filename does not include a numeric ID")
    history = []
    for index in range(number - 4, number):
        previous, _ = analyze.read_cycle(cycle.with_name(f"cycle_{index}.json"))
        history.append(analyze.event(previous, "output"))
    return np.asarray(history, dtype=np.float32)


def aggregate(scores, controls, initial, settings, history,
              probability_override=None):
    scores = np.asarray(scores, dtype=np.float32)
    if controls.shape != (len(scores), settings["steps"], 3):
        raise ValueError("control dimensions differ from score dimensions")
    regularizer = np.zeros(len(scores), dtype=np.float32)
    for axis, name in enumerate(AXES):
        gamma_over_variance = np.float32(
            settings["gamma"] / settings[name + "_std"] ** 2)
        noise = controls[:, :, axis] - initial[None, :, axis]
        regularizer += gamma_over_variance * np.sum(
            initial[None, :, axis] * noise, axis=1, dtype=np.float32)
    weighted = scores + regularizer
    relative = weighted - weighted.min()
    probability = np.exp(-relative / np.float32(settings["temperature"]))
    probability /= np.sum(probability, dtype=np.float32)
    if probability_override is not None:
        probability = np.asarray(probability_override, dtype=np.float32)
        if probability.shape != (len(scores),) or \
                abs(float(np.sum(probability)) - 1.) > 1e-4:
            raise ValueError("captured probability override is invalid")
    updated = np.einsum("i,ijk->jk", probability, controls, dtype=np.float32)
    limits = ((settings["vx_min"], settings["vx_max"]),
              (-settings["vy_max"], settings["vy_max"]),
              (-settings["wz_max"], settings["wz_max"]))
    clipped = updated.copy()
    for axis, (lower, upper) in enumerate(limits):
        clipped[:, axis] = np.clip(clipped[:, axis], lower, upper)
    filtered = np.stack([replay_ranking.smooth_axis(
        clipped[:, axis], history[:, axis]) for axis in range(3)], axis=-1)
    return {"weighted": weighted, "probability": probability,
            "unfiltered_sequence": clipped,
            "filtered_sequence": filtered,
            "returned_control": filtered[settings["offset"]].astype(float).tolist(),
            "constraint_clip_max_abs": float(np.max(np.abs(updated - clipped)))}


def open_loop_geometry(sequence, meta, settings, actual, body, padded):
    trajectory = analyze.integrate_omni(
        *(sequence[:, axis] for axis in range(3)),
        meta["pose"], settings["dt"])
    body_gaps = []
    padded_gaps = []
    for step, polygon in enumerate(actual):
        pose = tuple(float(axis[step]) for axis in trajectory)
        body_gaps.append(analyze.polygon_distance(
            analyze.placed(body, pose), polygon))
        padded_gaps.append(analyze.polygon_distance(
            analyze.placed(padded, pose), polygon))
    goal = meta["path"][-1]
    return {
        "body_min_gap_m": float(min(body_gaps)),
        "body_min_gap_step": int(np.argmin(body_gaps) + 1),
        "padded_min_gap_m": float(min(padded_gaps)),
        "dynamic_clearance_gate_met": bool(min(body_gaps) >= .05 and
                                           min(padded_gaps) > 0),
        "endpoint_path_goal_distance_m": float(np.hypot(
            trajectory[0][-1] - goal[0], trajectory[1][-1] - goal[1])),
    }


def run(cycle, profile_path, truth_path, history_path, score_root, mask_dir,
        aggregate_mask=None, aggregate_fixture_record=None):
    if (aggregate_mask is None) != (aggregate_fixture_record is None):
        raise ValueError("aggregate mask and fixture record must be paired")
    if aggregate_mask is not None and history_path is None:
        raise ValueError("aggregate fixture verification requires history file")
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if settings["iterations"] != 1 or settings["batch"] != 300 or \
            settings["offset"] != 1 or settings["steps"] != 30:
        raise ValueError("frozen MPPI settings changed")
    if history_path is None:
        history = captured_history(cycle)
    else:
        history_payload = json.loads(history_path.read_text())
        if history_payload.get("cycle_id") != meta["cycle_id"]:
            raise ValueError("control history belongs to a different cycle")
        history = np.asarray(history_payload["previous_outputs"],
                             dtype=np.float32)
        if history.shape != (4, 3):
            raise ValueError("control history must have four 3-axis outputs")
    initial = np.stack([analyze.last(arrays, "initial." + name)
                        for name in AXES], axis=-1)
    captured_controls = np.stack([analyze.last(arrays, "sampled.c" + name)
                                  for name in AXES], axis=-1)
    captured_scores = analyze.last(arrays, "scored.costs")
    captured_probability = analyze.last(arrays, "weighted.probability")
    computed_baseline = aggregate(captured_scores, captured_controls, initial,
                                  settings, history)
    baseline = aggregate(captured_scores, captured_controls, initial,
                         settings, history, captured_probability)
    original_output = np.asarray(analyze.event(meta, "output"))
    baseline_error = float(np.max(np.abs(
        np.asarray(baseline["returned_control"]) - original_output)))
    computed_baseline_error = float(np.max(np.abs(
        np.asarray(computed_baseline["returned_control"]) - original_output)))
    probability_error = float(np.max(np.abs(
        computed_baseline["probability"] - captured_probability)))
    if baseline_error > 1e-5 or computed_baseline_error > 2e-5 or \
            probability_error > 3e-5:
        raise ValueError(f"captured aggregation does not replay: {baseline_error}")
    full_filter_error = max(float(np.max(np.abs(
        baseline["filtered_sequence"][:, axis] - analyze.last(
            arrays, "after_filter." + name))))
        for axis, name in enumerate(AXES))
    if full_filter_error > 1e-5:
        raise ValueError(f"captured full control filter differs: {full_filter_error}")
    original_native = np.fromfile(
        score_root / "captured_native_scores.bin", dtype="<f4")
    if original_native.shape != (300,):
        raise ValueError("captured native score file is absent or malformed")
    pre_v1 = analyze.last(arrays, "critic.FollowPath.PathAngleCritic")
    residual = np.abs(original_native.astype(np.float64) - pre_v1)
    native_baseline = aggregate(original_native, captured_controls, initial,
                                settings, history)
    if float(np.max(residual)) > 2 or int(np.sum(residual > 1e-3)) > 25:
        raise ValueError("native critic baseline diverges beyond documented limit")
    prediction = analyze.event(meta, "prediction.input")
    if prediction["status"] != "accepted":
        raise ValueError("frozen prediction was not accepted")
    captured_analysis, _ = analyze.analyze(cycle, profile_path, truth_path)
    if captured_analysis["predicted_collision_rollouts"] != 300 or not \
            captured_analysis["all_batches_prediction_collision_invariant_at_first_step"] or \
            captured_analysis["prediction_score_span"] > 1e-3:
        raise ValueError("V1 hard-collision term is not a constant first-step tie")
    if captured_analysis["safe_truth_and_costmap_rollouts"] != 16:
        raise ValueError("captured safety baseline changed")
    first_pose_spread = max(float(np.ptp(analyze.last(arrays, "rollout." + axis)[:, 0]))
                            for axis in ("x", "y", "yaw"))
    if first_pose_spread > 1e-6:
        raise ValueError("first pose is not common to all samples")
    profile = yaml.safe_load(profile_path.read_text())
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    padded = meta["padded_footprint"]
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    physical = analyze.obstacle_polygon()
    actual = [analyze.placed(physical, analyze.interpolated_pose(
        truth, times, prediction["consumer_sim_s"] + (step + 1) * settings["dt"]))
        for step in range(settings["steps"])]
    original_open_loop = open_loop_geometry(
        baseline["filtered_sequence"], meta, settings, actual, body, padded)
    if abs(original_open_loop["body_min_gap_m"] - captured_analysis[
            "optimized_filtered_sequence_truth_body_min_gap_m"]) > 1e-6:
        raise ValueError("captured full control geometry differs")
    results = []
    for seed in range(4):
        controls, trajectory = sample_omni(meta, arrays, 2000, seed)
        if max(float(np.ptp(axis[:, 0])) for axis in trajectory) > 1e-6:
            raise ValueError("new sampled first poses differ")
        body_gaps, padded_gaps = geometry_labels(
            trajectory, body, padded, actual)
        safe = (body_gaps >= .05) & (padded_gaps > 0)
        static_mask = np.loadtxt(mask_dir / f"seed_{seed}_mask.txt", dtype=bool)
        if static_mask.shape != (2000,):
            raise ValueError("native costmap mask length differs")
        joint = safe & ~static_mask
        stacked = np.stack([controls[name] for name in AXES], axis=-1)
        for batch in BATCHES:
            base = score_root / f"seed{seed}_batch{batch}"
            scores = np.fromfile(base / "native_scores.bin", dtype="<f4")
            if scores.shape != (batch,):
                raise ValueError(f"native score count differs at {base}")
            result = aggregate(scores, stacked[:batch], initial, settings,
                               history)
            filtered_open_loop = open_loop_geometry(
                result["filtered_sequence"], meta, settings, actual,
                body, padded)
            unfiltered_open_loop = open_loop_geometry(
                result["unfiltered_sequence"], meta, settings, actual,
                body, padded)
            weighted = result["weighted"]
            probability = result["probability"]
            order = np.argsort(weighted)
            safe_indices = np.flatnonzero(joint[:batch])
            safe_ranks = np.flatnonzero(np.isin(order, safe_indices)) + 1
            best_safe_minus_unsafe = (float(
                np.min(weighted[safe_indices]) -
                np.min(weighted[~joint[:batch]]))
                if len(safe_indices) and len(safe_indices) < batch else None)
            elapsed_ns = [int(value) for value in
                          (base / "time_ns.txt").read_text().split()]
            score_match = re.search(r"^score_eval_ms=([0-9.]+)$",
                                    (base / "log.txt").read_text(), re.MULTILINE)
            if score_match is None:
                raise ValueError(f"native score duration missing at {base}")
            results.append({
                "seed": seed, "batch": batch,
                "dynamic_safe_count": int(np.sum(safe[:batch])),
                "joint_safe_count": int(len(safe_indices)),
                "best_true_body_gap_m": float(np.max(body_gaps[:batch])),
                "best_joint_safe_rank": int(safe_ranks[0]) if len(safe_ranks) else None,
                "best_safe_minus_unsafe_weighted_score": best_safe_minus_unsafe,
                "minimum_score_rollout_joint_safe": bool(joint[order[0]]),
                "top_10_joint_safe_count": int(np.sum(joint[order[:10]])),
                "joint_safe_probability_mass": float(np.sum(
                    probability[safe_indices])),
                "effective_sample_size": float(1 / np.sum(probability ** 2)),
                "returned_control_mps_radps": result["returned_control"],
                "filtered_open_loop": filtered_open_loop,
                "unfiltered_open_loop": unfiltered_open_loop,
                "constraint_clip_max_abs": result["constraint_clip_max_abs"],
                "native_process_wall_ms": (elapsed_ns[1] - elapsed_ns[0]) / 1e6,
                "native_score_eval_ms": float(score_match.group(1)),
                "native_score_sha256": digest(base / "native_scores.bin"),
            })
    aggregate_static = None
    if aggregate_mask is not None:
        fixture = json.loads(aggregate_fixture_record.read_text())
        if fixture["cycle_json_sha256"] != digest(cycle) or \
                fixture["cycle_bin_sha256"] != digest(cycle.with_suffix(".bin")) or \
                fixture["profile_sha256"] != digest(profile_path) or \
                fixture["history_sha256"] != digest(history_path) or \
                fixture["fixture_sha256"] != digest(
                    aggregate_fixture_record.with_suffix(".bin")):
            raise ValueError("filtered aggregate fixture inputs changed")
        mask = np.loadtxt(aggregate_mask, dtype=bool)
        if mask.shape != (len(results),) or len(fixture["rows"]) != len(results):
            raise ValueError("filtered aggregate costmap mask rows differ")
        for index, (row, record, collision) in enumerate(zip(
                results, fixture["rows"], mask)):
            if (row["seed"], row["batch"], row["native_score_sha256"]) != (
                    record["seed"], record["batch"],
                    record["native_score_sha256"]):
                raise ValueError(f"filtered aggregate row {index} changed")
            row["filtered_open_loop"]["costcritic_collision"] = bool(collision)
            row["filtered_open_loop"]["joint_clearance_gate_met"] = bool(
                row["filtered_open_loop"]["dynamic_clearance_gate_met"] and
                not collision)
        aggregate_static = {
            "fixture_record_sha256": digest(aggregate_fixture_record),
            "fixture_binary_sha256": fixture["fixture_sha256"],
            "native_mask_sha256": digest(aggregate_mask),
            "costcritic_collision_count": int(np.sum(mask)),
        }
    return {
        "schema": "rm_dynamic_prediction_native_critic_sensitivity/v1",
        "scope": "One frozen cycle. Deterministic nested Gaussian samples, original raw local costmap and path, installed Nav2 standard critics, offline MPPI regularizer and softmax, captured four-command history and full Nav2 control filter. PredictionV1Critic is a constant hard-collision term for all samples at the shared first step, so it is omitted from rankings. Gazebo truth labels and filtered open-loop dynamic geometry are offline only; filtered aggregate static collision is independently checked with the original native CostCritic mask tool when aggregate_static is present. PathAlign baseline has 22/300 residuals, so new-batch controls are exploratory. Process wall time excludes Python sampling, truth geometry and full ROS controller context.",
        "input_sha256": {"cycle_json": digest(cycle),
                         "cycle_bin": digest(cycle.with_suffix(".bin")),
                         "profile": digest(profile_path),
                         "truth": digest(truth_path),
                         "history": digest(history_path) if history_path else None,
                         "native_executable": digest(
                             score_root / "frozen_critic_score_executable")},
        "captured_baseline": {
            "aggregation_returned_control_max_abs_error": baseline_error,
            "recomputed_probability_max_abs_error": probability_error,
            "recomputed_returned_control_max_abs_error": computed_baseline_error,
            "aggregation_full_filter_max_abs_error": full_filter_error,
            "captured_filtered_open_loop": original_open_loop,
            "native_pre_v1_score_max_abs_error": float(np.max(residual)),
            "native_pre_v1_score_p95_abs_error": float(np.percentile(residual, 95)),
            "native_pre_v1_score_mismatch_gt_1e_3": int(np.sum(residual > 1e-3)),
            "native_aggregation_returned_control": native_baseline["returned_control"],
            "recorded_returned_control": original_output.tolist(),
        },
        "aggregate_static": aggregate_static,
        "results": results,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycle", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--truth", type=Path, required=True)
    parser.add_argument("--history", type=Path)
    parser.add_argument("--score-root", type=Path, required=True)
    parser.add_argument("--mask-dir", type=Path, required=True)
    parser.add_argument("--aggregate-mask", type=Path)
    parser.add_argument("--aggregate-fixture-record", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    output = run(args.cycle, args.profile, args.truth, args.history,
                 args.score_root, args.mask_dir, args.aggregate_mask,
                 args.aggregate_fixture_record)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"output": str(args.output),
                      "results": len(output["results"])}))
