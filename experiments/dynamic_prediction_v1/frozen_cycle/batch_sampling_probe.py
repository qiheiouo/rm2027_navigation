#!/usr/bin/env python3
"""Measure frozen MPPI Omni sampling coverage at nested batch sizes.

The initial control, pose, speed, noise std and truth are frozen. This
reconstructs the Nav2 1.1.20 sampling/Omni integration law, but does not run
any critic or infer an MPPI output. Truth is used only for offline labels.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import yaml

import analyze


BATCHES = (300, 600, 1000, 2000)
AXES = (("vx", "cvx"), ("vy", "cvy"), ("wz", "cwz"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sample_omni(meta, arrays, count, seed):
    settings = analyze.event(meta, "settings")
    rng = np.random.default_rng(seed)
    controls = {}
    for name, _ in AXES:
        initial = analyze.last(arrays, "initial." + name).astype(np.float32)
        noise = rng.standard_normal((count, settings["steps"]),
                                    dtype=np.float32)
        controls[name] = initial[None, :] + noise * np.float32(
            settings[name + "_std"])
    velocities = {}
    for speed_index, (name, _) in enumerate(AXES):
        current = np.full((count, 1), meta["speed"][speed_index],
                          dtype=np.float32)
        velocities[name] = np.concatenate((current, controls[name][:, :-1]),
                                          axis=1)
    trajectory = analyze.integrate_omni(
        velocities["vx"], velocities["vy"], velocities["wz"],
        meta["pose"], settings["dt"])
    return controls, trajectory


def geometry_labels(trajectory, body, padded, actual_polygons):
    count, steps = trajectory[0].shape
    if len(actual_polygons) != steps:
        raise ValueError("truth horizon differs from sampled horizon")
    minimum_body = np.full(count, np.inf)
    minimum_padded = np.full(count, np.inf)
    for index in range(count):
        for step in range(steps):
            pose = tuple(float(axis[index, step]) for axis in trajectory)
            minimum_body[index] = min(
                minimum_body[index], analyze.polygon_distance(
                    analyze.placed(body, pose), actual_polygons[step]))
            minimum_padded[index] = min(
                minimum_padded[index], analyze.polygon_distance(
                    analyze.placed(padded, pose), actual_polygons[step]))
    return minimum_body, minimum_padded


def run(trial, cycle_id, seeds):
    cycle = trial / "mppi_cycles" / f"cycle_{cycle_id}.json"
    profile = trial / "profile.yaml"
    truth_file = trial / "gazebo_poses.jsonl"
    captured, original_records = analyze.analyze(cycle, profile, truth_file)
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (captured["batch_size"] != 300 or settings["iterations"] != 1 or
            settings["steps"] != 30 or settings["offset"] != 1):
        raise ValueError("expected the locked single-iteration 300x30 cycle")
    if (captured["predicted_collision_rollouts"] != 300 or not
            captured["all_batches_prediction_collision_invariant_at_first_step"]):
        raise ValueError("captured V1 first-step tie no longer holds")
    actual = analyze.rows_from_transport(truth_file)
    times = [row["t"] for row in actual]
    frozen = yaml.safe_load(profile.read_text())
    local = frozen["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    padded = meta["padded_footprint"]
    obstacle = analyze.obstacle_polygon()
    truth_polygons = [analyze.placed(obstacle, analyze.interpolated_pose(
        actual, times, captured["consumer_sim_s"] +
        (step + 1) * settings["dt"])) for step in range(settings["steps"])]
    original_safe = sum(row["truth_dynamic_safe"] for row in original_records)
    original_joint_safe = sum(row["truth_dynamic_safe"] and not
                              row["costcritic_collision"]
                              for row in original_records)
    if original_safe != 16 or original_joint_safe != 16:
        raise ValueError("captured control-cycle safety labels changed")
    trials = []
    for seed in seeds:
        start = time.perf_counter()
        _, trajectory = sample_omni(meta, arrays, BATCHES[-1], seed)
        sample_s = time.perf_counter() - start
        if any(np.ptp(axis[:, 0]) > 1e-6 for axis in trajectory):
            raise ValueError("Omni first step must be independent of sampled control")
        start = time.perf_counter()
        body_gaps, padded_gaps = geometry_labels(
            trajectory, body, padded, truth_polygons)
        geometry_s = time.perf_counter() - start
        safe = (body_gaps >= .05) & (padded_gaps > 0)
        trials.append({
            "seed": seed,
            "sampling_seconds_for_2000": sample_s,
            "truth_geometry_seconds_for_2000": geometry_s,
            "prefixes": {str(n): {
                "true_dynamic_safe_count": int(np.sum(safe[:n])),
                "best_true_body_gap_m": float(np.max(body_gaps[:n])),
                "median_true_body_gap_m": float(np.median(body_gaps[:n])),
                "safe_fraction": float(np.mean(safe[:n])),
            } for n in BATCHES},
        })
    return {
        "schema": "rm_dynamic_prediction_batch_sampling_probe/v1",
        "scope": "Nested independent Gaussian control samples around the frozen Nav2 initial sequence, with the recorded pose/speed and Omni law. Ground truth labels dynamic geometry only. New samples do not have CostCritic/other critic scores or final control. Timing excludes ROS, critics and MPPI aggregation; not a control-cycle benchmark.",
        "input_sha256": {"cycle_json": digest(cycle),
                         "cycle_bin": digest(cycle.with_suffix(".bin")),
                         "profile": digest(profile),
                         "truth": digest(truth_file)},
        "cycle_id": cycle_id,
        "captured_dynamic_safe_count": original_safe,
        "captured_dynamic_and_costcritic_safe_count": original_joint_safe,
        "captured_v1_predicted_collision_count": captured[
            "predicted_collision_rollouts"],
        "v1_predicted_collision_invariant_at_first_step": True,
        "frozen_settings": settings,
        "batches": BATCHES,
        "trials": trials,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--cycle-id", type=int, default=162)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.trial, args.cycle_id, args.seeds)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"seeds": args.seeds, "output": str(args.output)}))
