#!/usr/bin/env python3
"""Test a fixed, time-resolved V1 collision tie breaker on frozen rollouts.

The existing hard collision score is retained. Only an additional fraction
of V1 prediction steps in conservative occupancy is scored, in the original
collision unit. Future obstacle truth labels output geometry only.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
import replay_ranking
import short_horizon_filter_probe as short


def exposure(meta, arrays, params):
    prediction = analyze.event(meta, "prediction.input")
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("single confirmed box fixture required")
    track = tracks[0]
    settings = analyze.event(meta, "settings")
    x, y, yaw = (analyze.last(arrays, "rollout." + axis)
                 for axis in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(math.floor(float(params["horizon"]) /
                                           settings["dt"] + 1e-9)))
    if steps != 9:
        raise ValueError("fixed V1 horizon differs")
    counts = np.zeros(x.shape[0], dtype=np.int16)
    hits_by_step = []
    for step in range(steps):
        box = analyze.predicted_box(
            track["xy"], track["vxy"], track["size_xy"],
            (params["object_width"], params["object_height"]),
            prediction["source_age_s"], (step + 1) * settings["dt"],
            params["reference_acceleration"]).polygon()
        hits = np.zeros(x.shape[0], dtype=bool)
        for index in range(x.shape[0]):
            robot = analyze.placed(meta["padded_footprint"],
                                   (float(x[index, step]), float(y[index, step]),
                                    float(yaw[index, step])))
            hits[index] = analyze.polygon_distance(robot, box) <= 1e-9
        hits_by_step.append(int(hits.sum()))
        counts += hits
    return counts.astype(np.float64) / steps, hits_by_step


def one_cycle(trial, cycle_id, truth, times, body, box, goal=None):
    path = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile_path = trial / "profile.yaml"
    summary, records = analyze.analyze(path, profile_path,
                                       trial / "gazebo_poses.jsonl")
    meta, arrays = analyze.read_cycle(path)
    settings = analyze.event(meta, "settings")
    params = yaml.safe_load(profile_path.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    history = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    original_cost = analyze.last(arrays, "weighted.costs")
    original_weight = analyze.last(arrays, "weighted.probability")
    original_control = replay_ranking.aggregate(
        arrays, settings, history, original_weight)
    error = max(float(np.max(np.abs(original_control[axis] - analyze.last(
        arrays, "after_filter." + axis)))) for axis, _ in short.AXES)
    if error > 1e-5:
        raise ValueError(f"cycle {cycle_id}: baseline control differs")
    occupancy, hits_by_step = exposure(meta, arrays, params)
    unit = np.float32((3.81 / 254.) * 1_000_000. /
                      summary["prediction_steps"])
    score = original_cost + np.float32(unit * occupancy)
    weight = replay_ranking.probabilities(score, settings["temperature"])
    control = replay_ranking.aggregate(arrays, settings, history, weight)
    safe = np.array([row["truth_dynamic_safe"] and not
                     row["costcritic_collision"] for row in records])

    def result(weights, sequence):
        gap = short.control_gap(sequence, meta, settings, truth, times, body,
                                box, summary["consumer_sim_s"])
        item = {"truth_safe_rollout_weight": float(weights[safe].sum()),
                "filtered_body_min_gap_m": gap["body_min_clearance_m"],
                "filtered_body_min_gap_step": gap["step"],
                "effective_sample_size": float(1. / np.sum(weights * weights)),
                "top_rollout": int(np.argmax(weights)),
                "top_rollout_truth_safe": bool(safe[int(np.argmax(weights))]),
                "returned_vx_vy_wz": [float(sequence[axis][settings["offset"]])
                                        for axis, _ in short.AXES]}
        if goal is not None:
            x, y = (analyze.last(arrays, "rollout." + axis)
                    for axis in ("x", "y"))
            mask = np.hypot(x[:, -1] - goal[0], y[:, -1] - goal[1]) <= .15
            trajectory = analyze.integrate_omni(
                *(sequence[axis] for axis, _ in short.AXES),
                meta["pose"], settings["dt"])
            item["goal_position_rollout_weight"] = float(weights[mask].sum())
            item["endpoint_goal_position_distance_m"] = math.hypot(
                float(trajectory[0][-1]) - goal[0],
                float(trajectory[1][-1]) - goal[1])
        return item

    return {"cycle_id": cycle_id, "cycle_json_sha256": short.digest(path),
            "cycle_bin_sha256": short.digest(path.with_suffix(".bin")),
            "baseline_control_max_abs_error": error,
            "truth_safe_rollouts": int(safe.sum()),
            "prediction_hits_by_step": hits_by_step,
            "exposure_min": float(occupancy.min()),
            "exposure_max": float(occupancy.max()),
            "exposure_safe_mean": float(occupancy[safe].mean()) if safe.any()
                else None,
            "exposure_unsafe_mean": float(occupancy[~safe].mean()) if
                (~safe).any() else None,
            "baseline": result(original_weight, original_control),
            "exposure_tie_break": result(weight, control)}


def run(collision_trial, goal_trial):
    selection = json.loads((collision_trial / "selection.json").read_text())
    start, stop = selection["witness_sim_s"] - 2., selection["witness_sim_s"]
    truth, times, body, box, collision_inputs = short.trial_input(
        collision_trial)
    cycles = []
    for path in sorted((collision_trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        if start <= meta["sim_ns"] / 1e9 <= stop and any(
                event["kind"] == "prediction.input" for event in meta["events"]):
            cycles.append(one_cycle(collision_trial, meta["cycle_id"],
                                    truth, times, body, box))
    if len(cycles) != 20:
        raise ValueError("frozen 20-cycle window differs")
    goal_truth, goal_times, goal_body, goal_box, goal_inputs = short.trial_input(
        goal_trial)
    goal = json.loads((goal_trial / "runtime_audit.json").read_text())[
        "navigation_result"]["goal"]
    return {"schema": "rm_dynamic_prediction/conflict_exposure_probe/v1",
            "scope": "Fixed V1 hard score plus original collision unit times fraction of nine conservative predicted-occupancy steps intersecting the padded robot. Same frozen controls, other critics, temperature and filter. Future Gazebo poses evaluate only.",
            "collision_trial_inputs": collision_inputs,
            "goal_trial_inputs": goal_inputs,
            "selection_sha256": short.digest(collision_trial / "selection.json"),
            "window_sim_s": [start, stop],
            "selected_cycle_id": selection["selected_cycle_id"],
            "window_cycles": cycles,
            "goal_cycle_263": one_cycle(goal_trial, 263, goal_truth,
                                        goal_times, goal_body, goal_box, goal)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collision-trial", type=Path, required=True)
    parser.add_argument("--goal-trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = run(args.collision_trial, args.goal_trial)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cycles": len(report["window_cycles"]),
                      "output": str(args.output)}))
