#!/usr/bin/env python3
"""Replay a short soft risk hint and audit MPPI's output filter on frozen cycles.

This is an offline counterfactual. Gazebo future poses only evaluate geometry;
they never enter the proposed score. No controller or deployment code changes.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
import occupancy_rank_probe
import replay_ranking


HORIZONS_S = (0.4, 0.6, 0.9)
AXES = (("vx", "cvx"), ("vy", "cvy"), ("wz", "cwz"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def control_gap(controls, meta, settings, truth, truth_times, body, box, stamp):
    gap, step = replay_ranking.open_loop_gap(
        controls, meta, settings, truth, truth_times, body, box, stamp)
    return {"body_min_clearance_m": float(gap), "step": int(step)}


def clipped_sequences(arrays, settings, weights=None, rollout=None):
    if (weights is None) == (rollout is None):
        raise ValueError("provide either weights or a rollout index")
    result = {}
    for axis, sampled in AXES:
        controls = analyze.last(arrays, "sampled." + sampled)
        sequence = (np.sum(controls * weights[:, None], axis=0, dtype=np.float32)
                    if weights is not None else controls[rollout].copy())
        lower = settings["vx_min"] if axis == "vx" else -settings[
            "vy_max" if axis == "vy" else "wz_max"]
        upper = settings["vx_max" if axis == "vx" else
                         "vy_max" if axis == "vy" else "wz_max"]
        result[axis] = np.clip(sequence, lower, upper)
    return result


def filtered(sequences, history):
    return {axis: replay_ranking.smooth_axis(sequence, history[axis])
            for axis, sequence in sequences.items()}


def one_cycle(trial, cycle_id, truth, truth_times, body, box, goal=None):
    cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile = trial / "profile.yaml"
    summary, records = analyze.analyze(cycle, profile, trial / "gazebo_poses.jsonl")
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    params = yaml.safe_load(profile.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    if settings["batch"] != 300 or summary["prediction_steps"] != 9:
        raise ValueError("probe requires the frozen 300 rollout, nine step input")
    old_costs = analyze.last(arrays, "weighted.costs")
    old_weights = analyze.last(arrays, "weighted.probability")
    old_prediction = analyze.critic_deltas(arrays)[0][
        "FollowPath.PredictionV1Critic"]
    if np.max(np.abs(replay_ranking.probabilities(
            old_costs, settings["temperature"]) - old_weights)) > 1e-6:
        raise ValueError(f"cycle {cycle_id}: softmax replay mismatch")
    history = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    baseline_control = replay_ranking.aggregate(
        arrays, settings, history, old_weights)
    control_error = max(float(np.max(np.abs(
        baseline_control[axis] - analyze.last(arrays, "after_filter." + axis))))
        for axis, _ in AXES)
    if control_error > 1e-5:
        raise ValueError(f"cycle {cycle_id}: output filter replay mismatch")
    safe = np.asarray([record["truth_dynamic_safe"] and not
                       record["costcritic_collision"] for record in records])
    x, y = (analyze.last(arrays, "rollout." + axis) for axis in ("x", "y"))
    goal_mask = (np.hypot(x[:, -1] - goal[0], y[:, -1] - goal[1]) <= .15
                 if goal is not None else None)
    stamp = summary["consumer_sim_s"]

    def metrics(weights, controls):
        result = {
            "safe_rollout_weight": float(weights[safe].sum()),
            "effective_sample_size": float(1. / np.sum(weights * weights)),
            "filtered_gap": control_gap(controls, meta, settings, truth,
                                        truth_times, body, box, stamp),
            "returned_vx_vy_wz": [float(controls[axis][settings["offset"]])
                                   for axis, _ in AXES],
        }
        if goal_mask is not None:
            result["goal_position_rollout_weight"] = float(
                weights[goal_mask].sum())
            trajectory = analyze.integrate_omni(
                *(controls[axis] for axis, _ in AXES), meta["pose"],
                settings["dt"])
            result["endpoint_goal_position_distance_m"] = math.hypot(
                float(trajectory[0][-1]) - goal[0],
                float(trajectory[1][-1]) - goal[1])
        return result

    output = {
        "cycle_id": cycle_id,
        "cycle_json_sha256": digest(cycle),
        "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
        "truth_safe_rollouts": int(safe.sum()),
        "captured_predicted_collision_rollouts": summary[
            "predicted_collision_rollouts"],
        "baseline_filter_max_abs_error": control_error,
        "captured": metrics(old_weights, baseline_control),
        "soft_hint": {},
    }
    # Keep the original hard-collision score *unit* fixed. Only replace the
    # V1 binary score with mean physical overlap over a shorter time span.
    collision_unit = np.float32(
        (3.81 / 254.) * 1_000_000. / summary["prediction_steps"])
    for horizon in HORIZONS_S:
        short = dict(params)
        short["horizon"] = horizon + 1e-6  # select exact 4/6/9 float-dt steps
        risk = occupancy_rank_probe.expected_overlap(meta, arrays, short)
        scores = old_costs - old_prediction + np.float32(collision_unit * risk)
        weights = replay_ranking.probabilities(scores, settings["temperature"])
        controls = replay_ranking.aggregate(arrays, settings, history, weights)
        before_filter = clipped_sequences(arrays, settings, weights=weights)
        top = int(np.argmax(weights))
        top_before = clipped_sequences(arrays, settings, rollout=top)
        top_after = filtered(top_before, history)
        result = metrics(weights, controls)
        result.update({
            "risk_fraction_span": float(np.ptp(risk)),
            "unfiltered_aggregate_gap": control_gap(
                before_filter, meta, settings, truth, truth_times, body, box,
                stamp),
            "top_rollout": top,
            "top_weight": float(weights[top]),
            "top_rollout_truth_safe": bool(safe[top]),
            "top_rollout_raw_truth_gap_m": float(records[top][
                "truth_body_min_clearance_m"]),
            "top_rollout_unfiltered_gap": control_gap(
                top_before, meta, settings, truth, truth_times, body, box,
                stamp),
            "top_rollout_filtered_gap": control_gap(
                top_after, meta, settings, truth, truth_times, body, box,
                stamp),
        })
        output["soft_hint"][f"{horizon:.1f}s"] = result
    return output


def trial_input(trial):
    profile = trial / "profile.yaml"
    parsed = yaml.safe_load(profile.read_text())
    body = yaml.safe_load(parsed["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    truth_path = trial / "gazebo_poses.jsonl"
    truth = analyze.rows_from_transport(truth_path)
    return truth, [row["t"] for row in truth], body, analyze.obstacle_polygon(), {
        "profile_sha256": digest(profile),
        "truth_sha256": digest(truth_path),
    }


def run(collision_trial, goal_trial):
    selection_path = collision_trial / "selection.json"
    selection = json.loads(selection_path.read_text())
    start = selection["witness_sim_s"] - 2.
    stop = selection["witness_sim_s"]
    truth, times, body, box, collision_inputs = trial_input(collision_trial)
    cycles = []
    for path in sorted((collision_trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        t = meta["sim_ns"] / 1e9
        if not start <= t <= stop or not any(
                event["kind"] == "prediction.input" for event in meta["events"]):
            continue
        cycle = one_cycle(collision_trial, meta["cycle_id"], truth, times,
                          body, box)
        cycles.append(cycle)
    if selection["selected_cycle_id"] not in [c["cycle_id"] for c in cycles]:
        raise ValueError("preregistered cycle absent from saturated window")
    goal_audit_path = goal_trial / "runtime_audit.json"
    goal = json.loads(goal_audit_path.read_text())[
        "navigation_result"]["goal"]
    goal_truth, goal_times, goal_body, goal_box, goal_inputs = trial_input(
        goal_trial)
    goal_cycle = one_cycle(goal_trial, 263, goal_truth, goal_times,
                           goal_body, goal_box, goal)
    return {
        "schema": "rm_dynamic_prediction_short_horizon_filter_probe/v1",
        "scope": "Fixed captured MPPI rollouts; causal prediction score only. Future Gazebo box poses evaluate geometry offline. No runtime critic or deployment change.",
        "fixed_collision_unit": "(3.81 / 254) * 1000000 / 9; no weight change",
        "horizons_s": HORIZONS_S,
        "clearance_gate_m": .05,
        "collision_trial": {"inputs": collision_inputs,
                            "selection_sha256": digest(selection_path),
                            "selected_cycle_id": selection[
                                "selected_cycle_id"],
                            "window_sim_s": [start, stop],
                            "window_cycles": cycles,
                            "saturated_cycle_ids": [
                                cycle["cycle_id"] for cycle in cycles if
                                cycle["captured_predicted_collision_rollouts"] == 300
                                and cycle["truth_safe_rollouts"] > 0]},
        "goal_trial": {"inputs": goal_inputs,
                       "goal_audit_sha256": digest(goal_audit_path),
                       "cycle_263": goal_cycle},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collision-trial", type=Path, required=True)
    parser.add_argument("--goal-trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.collision_trial, args.goal_trial)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "selected_cycle": result["collision_trial"]["selected_cycle_id"],
        "window_cycles": len(result["collision_trial"]["window_cycles"]),
        "saturated_cycles": len(result["collision_trial"]["saturated_cycle_ids"]),
        "output": str(args.output)}, sort_keys=True))
