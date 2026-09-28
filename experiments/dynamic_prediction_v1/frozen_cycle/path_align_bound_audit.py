#!/usr/bin/env python3
"""Audit PathAlign's findClosestPathPt end-iterator exposure on frozen traces.

The installed Nav2 1.1.20 helper dereferences `iter` after lower_bound,
without checking `iter == vec.end()`. This auditor does not execute that
undefined behavior. It identifies the captured rollouts that reach the branch
and compares their native replay residuals to the archived critic scores.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, sample_omni


def path_exposure(path, trajectory, stride):
    """Mirror the inputs to PathAlign's integrated-distance lookup."""
    path = np.asarray(path, dtype=np.float32)[:, :2]
    trajectory = np.asarray(trajectory, dtype=np.float32)
    count, steps, axes = trajectory.shape
    if axes != 2 or stride <= 0 or len(path) < 2:
        raise ValueError("invalid path or trajectory dimensions")
    endpoints = trajectory[:, -1, :]
    nearest = np.argmin(np.sum(
        (endpoints[:, None, :] - path[None, :, :]) ** 2, axis=2,
        dtype=np.float32), axis=1)
    furthest = int(np.max(nearest))
    if furthest < 2:
        raise ValueError("path too short for integrated audit")
    integrated = np.zeros(furthest, dtype=np.float32)
    for index in range(1, furthest):
        delta = path[index] - path[index - 1]
        segment = np.sqrt(np.sum(delta * delta, dtype=np.float32),
                          dtype=np.float32)
        integrated[index] = np.float32(integrated[index - 1] + segment)
    sampled_steps = list(range(stride, steps, stride))
    traveled = np.zeros(count, dtype=np.float32)
    first_exposed = np.full(count, -1, dtype=np.int32)
    for step in sampled_steps:
        delta = trajectory[:, step, :] - trajectory[:, step - stride, :]
        traveled += np.sqrt(np.sum(delta * delta, axis=1, dtype=np.float32),
                            dtype=np.float32)
        first_exposed[(first_exposed < 0) &
                      (traveled > integrated[-1])] = step
    return furthest, integrated[-1], sampled_steps, first_exposed


def audit_case(name, cycle, profile_path, native_scores_path):
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    profile = yaml.safe_load(profile_path.read_text())
    align = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PathAlignCritic"]
    stride = int(align.get("trajectory_point_step", 4))
    path = np.asarray(meta["path"], dtype=np.float32)[:, :2]
    trajectory = np.stack([analyze.last(arrays, "rollout.x"),
                           analyze.last(arrays, "rollout.y")], axis=-1)
    count, steps, _ = trajectory.shape
    if count != settings["batch"] or steps != settings["steps"] or \
            stride <= 0 or len(path) < 2:
        raise ValueError(f"invalid frozen path or trajectory shape: {name}")
    furthest, limit, sampled_steps, first_exposed = path_exposure(
        path, trajectory, stride)
    exposed = first_exposed >= 0
    deltas, _ = analyze.critic_deltas(arrays)
    key = "FollowPath.PathAlignCritic"
    if key not in deltas:
        raise ValueError(f"missing captured PathAlign score: {name}")
    applied_count = int(np.sum(np.abs(deltas[key]) > 1e-3))
    original = analyze.last(arrays, "critic.FollowPath.PathAngleCritic")
    native = np.fromfile(native_scores_path, dtype="<f4")
    if native.shape != original.shape:
        raise ValueError(f"native score count differs: {name}")
    residual = np.abs(native.astype(np.float64) - original)
    mismatched = residual > 1e-3
    goal = path[-1]
    goal_distance = float(np.hypot(
        np.float32(meta["pose"][0]) - goal[0],
        np.float32(meta["pose"][1]) - goal[1]))
    return {
        "name": name, "cycle_id": meta["cycle_id"],
        "input_sha256": {
            "cycle_json": digest(cycle),
            "cycle_bin": digest(cycle.with_suffix(".bin")),
            "profile": digest(profile_path),
            "native_scores": digest(native_scores_path),
        },
        "batch": count, "path_points": len(path),
        "furthest_reached_path_point": furthest,
        "integrated_path_vector_size": furthest,
        "integrated_path_limit_m": float(limit),
        "trajectory_point_step": stride,
        "sampled_trajectory_steps_zero_based": sampled_steps,
        "path_align_offset_from_furthest": int(
            align.get("offset_from_furthest", 20)),
        "goal_distance_m": goal_distance,
        "path_align_goal_threshold_m": float(
            align.get("threshold_to_consider", .5)),
        "path_align_nonzero_score_count": applied_count,
        "potential_end_iterator_exposure_count": int(np.sum(exposed)),
        "first_exposed_step_counts": {str(step): int(np.sum(
            first_exposed == step)) for step in sampled_steps},
        "native_pre_v1_residual_gt_1e_3_count": int(np.sum(mismatched)),
        "residual_max_abs": float(np.max(residual)),
        "residual_gt_1e_3_and_exposed_count": int(np.sum(
            mismatched & exposed)),
        "residual_gt_1e_3_without_exposure_count": int(np.sum(
            mismatched & ~exposed)),
        "mismatched_indices": np.flatnonzero(mismatched).astype(int).tolist(),
        "exposed_indices": np.flatnonzero(exposed).astype(int).tolist(),
    }


def audit_synthetic(cycle, score_root, seeds):
    meta, arrays = analyze.read_cycle(cycle)
    path = meta["path"]
    results = []
    for seed in seeds:
        _, sampled = sample_omni(meta, arrays, BATCHES[-1], seed)
        trajectory = np.stack(sampled[:2], axis=-1)
        for batch in BATCHES:
            furthest, limit, _, first = path_exposure(
                path, trajectory[:batch], 4)
            scores = score_root / f"seed{seed}_batch{batch}" / "native_scores.bin"
            score_count = np.fromfile(scores, dtype="<f4").size
            if score_count != batch:
                raise ValueError(f"native score count differs: {scores}")
            results.append({
                "seed": seed, "batch": batch,
                "furthest_reached_path_point": furthest,
                "integrated_path_limit_m": float(limit),
                "potential_end_iterator_exposure_count": int(np.sum(first >= 0)),
                "native_scores_sha256": digest(scores),
            })
    return {"cycle_json_sha256": digest(cycle),
            "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
            "rows": results}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", nargs=4, required=True,
                        metavar=("NAME", "CYCLE", "PROFILE", "NATIVE_SCORES"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--synthetic-cycle", type=Path)
    parser.add_argument("--score-root", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = {
        "schema": "rm_dynamic_prediction_path_align_bound_audit/v1",
        "scope": "Read-only frozen-cycle analysis of the installed Nav2 1.1.20 PathAlign end-iterator precondition. Exposure indicates a source-level undefined-behavior branch, not proof that the unavailable original-image binary took a particular numerical result.",
        "cases": [audit_case(name, Path(cycle), Path(profile), Path(scores))
                  for name, cycle, profile, scores in args.case],
    }
    if (args.synthetic_cycle is None) != (args.score_root is None):
        raise ValueError("synthetic cycle and score root must be paired")
    if args.synthetic_cycle is not None:
        result["synthetic_batches"] = audit_synthetic(
            args.synthetic_cycle, args.score_root, range(4))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": [row["name"] for row in result["cases"]]}))
