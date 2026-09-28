#!/usr/bin/env python3
"""Screen one fixed filtered-candidate V1 rule on cycles 147, 162 and 263.

Only V1 is rescored on individually filtered trajectories. Captured standard
critics and final raw-control aggregation remain unchanged, making this a
cross-cycle screening diagnostic rather than a full controller replay.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from filtered_graded_batch_probe import filtered_poses
import occupancy_rank_probe
import replay_ranking
import short_horizon_filter_probe as short


def filtered_prediction_score(meta, poses, params):
    prediction = analyze.event(meta, "prediction.input")
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("single confirmed box fixture required")
    track = tracks[0]
    settings = analyze.event(meta, "settings")
    dt = settings["dt"]
    steps = min(poses.shape[1], int(math.floor(params["horizon"] / dt + 1e-9)))
    if steps != 9:
        raise ValueError("frozen V1 horizon differs")
    has_collision = np.zeros(len(poses), dtype=bool)
    near_count = np.zeros(len(poses), dtype=np.int16)
    hits_by_step = []
    for step in range(steps):
        box = analyze.predicted_box(
            track["xy"], track["vxy"], track["size_xy"],
            (params["object_width"], params["object_height"]),
            prediction["source_age_s"], (step + 1) * dt,
            params["reference_acceleration"]).polygon()
        hits = 0
        for index in range(len(poses)):
            robot = analyze.placed(meta["padded_footprint"],
                                   tuple(float(x) for x in poses[index, step]))
            gap = analyze.polygon_distance(robot, box)
            if gap <= 1e-9:
                has_collision[index] = True
                hits += 1
            elif gap < .02:
                near_count[index] += 1
        hits_by_step.append(hits)
    alternate = [("rollout." + axis, poses[:, :, i])
                 for i, axis in enumerate(("x", "y", "yaw"))]
    overlap = occupancy_rank_probe.expected_overlap(meta, alternate, params)
    repulsive = np.where(has_collision,
                         1_000_000. * (1. + overlap),
                         300. * near_count)
    return np.asarray((3.81 / 254.) * repulsive / steps,
                      dtype=np.float32), hits_by_step, overlap


def one_cycle(trial, cycle_id, goal=None):
    cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile_path = trial / "profile.yaml"
    truth_path = trial / "gazebo_poses.jsonl"
    summary, records = analyze.analyze(cycle, profile_path, truth_path)
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if settings["batch"] != 300:
        raise ValueError("fixed cross-cycle capture differs")
    params = yaml.safe_load(profile_path.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    history_axes = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    history = np.stack([history_axes[axis] for axis in ("vx", "vy", "wz")],
                       axis=-1).astype(np.float32)
    controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                         for axis in ("vx", "vy", "wz")], axis=-1)
    poses = filtered_poses(controls, meta, settings, history)
    new_prediction, hits, overlap = filtered_prediction_score(
        meta, poses, params)
    old_prediction = analyze.critic_deltas(arrays)[0][
        "FollowPath.PredictionV1Critic"]
    old_costs = analyze.last(arrays, "weighted.costs")
    old_weights = analyze.last(arrays, "weighted.probability")
    baseline = replay_ranking.aggregate(arrays, settings, history_axes,
                                        old_weights)
    baseline_error = max(float(np.max(np.abs(
        baseline[axis] - analyze.last(arrays, "after_filter." + axis))))
        for axis in ("vx", "vy", "wz"))
    if baseline_error > 1e-5:
        raise ValueError(f"baseline output differs: {cycle_id}")
    weights = replay_ranking.probabilities(
        old_costs - old_prediction + new_prediction,
        settings["temperature"])
    changed = replay_ranking.aggregate(arrays, settings, history_axes,
                                       weights)
    truth, times, body, box, input_hashes = short.trial_input(trial)
    safe = np.asarray([record["truth_dynamic_safe"] and not
                       record["costcritic_collision"] for record in records])

    def metrics(control, probability):
        geometry = short.control_gap(
            control, meta, settings, truth, times, body, box,
            summary["consumer_sim_s"])
        item = {"filtered_body_min_gap_m": geometry[
                    "body_min_clearance_m"],
                "filtered_body_min_gap_step": geometry["step"],
                "raw_joint_safe_rollout_weight": float(probability[safe].sum()),
                "effective_sample_size": float(1. / np.sum(
                    probability * probability)),
                "returned_vx_vy_wz": [float(control[axis][settings["offset"]])
                                        for axis in ("vx", "vy", "wz")]}
        if goal is not None:
            x, y = (analyze.last(arrays, "rollout." + axis)
                    for axis in ("x", "y"))
            mask = np.hypot(x[:, -1] - goal[0], y[:, -1] - goal[1]) <= .15
            path = analyze.integrate_omni(
                *(control[axis] for axis in ("vx", "vy", "wz")),
                meta["pose"], settings["dt"])
            item["goal_position_rollout_weight"] = float(probability[mask].sum())
            item["endpoint_goal_position_distance_m"] = math.hypot(
                float(path[0][-1]) - goal[0],
                float(path[1][-1]) - goal[1])
        return item

    return {"cycle_id": cycle_id,
            "cycle_json_sha256": digest(cycle),
            "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
            "profile_sha256": input_hashes["profile_sha256"],
            "truth_sha256": input_hashes["truth_sha256"],
            "baseline_filter_error": baseline_error,
            "raw_joint_safe_count": int(safe.sum()),
            "filtered_prediction_hits_by_step": hits,
            "filtered_overlap_span": float(np.ptp(overlap)),
            "new_v1_score_span": float(np.ptp(new_prediction)),
            "baseline": metrics(baseline, old_weights),
            "candidate": metrics(changed, weights)}


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    goal = json.loads((args.goal_trial / "runtime_audit.json").read_text())[
        "navigation_result"]["goal"]
    report = {"schema": "rm_dynamic_prediction/filtered_graded_cross_cycle/v1",
              "scope": "Preselected collision-trial cycles 147 and 162 plus independent phase-4 goal cycle 263. Each sampled control individually filtered and integrated with native current-speed first step; only V1 continuous-overlap/hard score recomputed on that candidate. Other captured critic scores, raw-control aggregation and final output filter unchanged. Truth evaluates only.",
              "cycles": [one_cycle(args.collision_trial, 147),
                         one_cycle(args.collision_trial, 162),
                         one_cycle(args.goal_trial, 263, goal)]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cycles": [row["cycle_id"] for row in report["cycles"]]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collision-trial", type=Path, required=True)
    parser.add_argument("--goal-trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args())
