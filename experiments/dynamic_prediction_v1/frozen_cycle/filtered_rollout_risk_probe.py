#!/usr/bin/env python3
"""Compare V1 risk on captured versus individually filtered sample controls.

Offline diagnostic only. The per-sample filtered trajectory is an alternative
control replay, not a captured Nav2 MPPI rollout or a closed-loop prediction.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
import occupancy_rank_probe
import replay_ranking
import short_horizon_filter_probe as short


HORIZONS_S = (0.4, 0.9)


def pairwise_auc(risk, safe):
    good, bad = risk[safe], risk[~safe]
    if not len(good) or not len(bad):
        return None
    return float(((good[:, None] < bad[None, :]).sum() +
                  .5 * (good[:, None] == bad[None, :]).sum()) /
                 (len(good) * len(bad)))


def one_cycle(trial, cycle_id, truth, times, body, box, goal=None):
    cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile = trial / "profile.yaml"
    summary, records = analyze.analyze(
        cycle, profile, trial / "gazebo_poses.jsonl")
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    params = yaml.safe_load(profile.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    if settings["batch"] != 300 or summary["prediction_steps"] != 9:
        raise ValueError("probe requires frozen 300-sample, nine-step input")
    history = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    old_costs = analyze.last(arrays, "weighted.costs")
    old_prediction = analyze.critic_deltas(arrays)[0][
        "FollowPath.PredictionV1Critic"]
    old_weights = analyze.last(arrays, "weighted.probability")
    replayed = replay_ranking.aggregate(
        arrays, settings, history, old_weights)
    error = max(float(np.max(np.abs(replayed[axis] - analyze.last(
        arrays, "after_filter." + axis)))) for axis, _ in short.AXES)
    if error > 1e-5:
        raise ValueError(f"cycle {cycle_id}: baseline filter replay mismatch")

    raw_safe = np.asarray([record["truth_dynamic_safe"] and not
                           record["costcritic_collision"] for record in records])
    filtered_xyz = [[], [], []]
    filtered_gaps = []
    for index in range(settings["batch"]):
        controls = short.filtered(short.clipped_sequences(
            arrays, settings, rollout=index), history)
        xyz = analyze.integrate_omni(
            *(controls[axis] for axis, _ in short.AXES),
            meta["pose"], settings["dt"])
        for axis in range(3):
            filtered_xyz[axis].append(xyz[axis])
        gap = short.control_gap(controls, meta, settings, truth, times,
                                body, box, summary["consumer_sim_s"])
        filtered_gaps.append(gap["body_min_clearance_m"])
    filtered_safe = np.asarray(filtered_gaps) >= .05
    filtered_and_raw_costmap_clear = filtered_safe & np.asarray([
        not record["costcritic_collision"] for record in records])
    # Keep captured raw rollout arrays and append alternative trajectory
    # arrays. analyze.last() then reads the latter for prediction scoring.
    filtered_arrays = arrays + [
        ("rollout." + axis, np.stack(values))
        for axis, values in zip(("x", "y", "yaw"), filtered_xyz)]
    collision_unit = np.float32(
        (3.81 / 254.) * 1_000_000. / summary["prediction_steps"])
    goal_mask = None
    if goal is not None:
        x, y = (analyze.last(arrays, "rollout." + axis)
                for axis in ("x", "y"))
        goal_mask = np.hypot(x[:, -1] - goal[0],
                             y[:, -1] - goal[1]) <= .15

    output = {
        "cycle_id": cycle_id,
        "cycle_json_sha256": short.digest(cycle),
        "cycle_bin_sha256": short.digest(cycle.with_suffix(".bin")),
        "baseline_filter_max_abs_error": error,
        "raw_truth_and_raw_costmap_safe_count": int(raw_safe.sum()),
        "filtered_control_dynamic_safe_count": int(filtered_safe.sum()),
        "filtered_control_dynamic_safe_and_raw_costmap_clear_count": int(
            filtered_and_raw_costmap_clear.sum()),
        "safe_in_both_raw_and_filtered_count": int((
            raw_safe & filtered_and_raw_costmap_clear).sum()),
        "risks": {},
    }
    for horizon in HORIZONS_S:
        settings_copy = dict(params)
        settings_copy["horizon"] = horizon + 1e-6
        raw_risk = occupancy_rank_probe.expected_overlap(
            meta, arrays, settings_copy)
        filtered_risk = occupancy_rank_probe.expected_overlap(
            meta, filtered_arrays, settings_copy)
        scores = old_costs - old_prediction + np.float32(
            collision_unit * filtered_risk)
        weights = replay_ranking.probabilities(
            scores, settings["temperature"])
        controls = replay_ranking.aggregate(
            arrays, settings, history, weights)
        top = int(np.argmax(weights))
        result = {
            "captured_rollout_risk_auc_for_filtered_control_safety":
                pairwise_auc(raw_risk, filtered_and_raw_costmap_clear),
            "filtered_control_risk_auc_for_filtered_control_safety":
                pairwise_auc(filtered_risk,
                             filtered_and_raw_costmap_clear),
            "filtered_safe_and_raw_costmap_clear_weight": float(
                weights[filtered_and_raw_costmap_clear].sum()),
            "top_rollout": top,
            "top_weight": float(weights[top]),
            "top_filtered_control_gap_m": float(filtered_gaps[top]),
            "aggregated_filtered_gap": short.control_gap(
                controls, meta, settings, truth, times, body, box,
                summary["consumer_sim_s"]),
            "returned_vx_vy_wz": [float(controls[axis][settings["offset"]])
                                   for axis, _ in short.AXES],
        }
        if goal_mask is not None:
            result["goal_position_rollout_weight"] = float(
                weights[goal_mask].sum())
            trajectory = analyze.integrate_omni(
                *(controls[axis] for axis, _ in short.AXES),
                meta["pose"], settings["dt"])
            result["endpoint_goal_position_distance_m"] = math.hypot(
                float(trajectory[0][-1]) - goal[0],
                float(trajectory[1][-1]) - goal[1])
        output["risks"][f"{horizon:.1f}s"] = result
    return output


def run(collision_trial, goal_trial):
    selection = json.loads((collision_trial / "selection.json").read_text())
    start = selection["witness_sim_s"] - 2.
    stop = selection["witness_sim_s"]
    truth, times, body, box, collision_inputs = short.trial_input(
        collision_trial)
    cycles = []
    for path in sorted((collision_trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda path: int(path.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        t = meta["sim_ns"] / 1e9
        if start <= t <= stop and any(event["kind"] == "prediction.input"
                                      for event in meta["events"]):
            cycles.append(one_cycle(collision_trial, meta["cycle_id"],
                                    truth, times, body, box))
    if len(cycles) != 20 or selection["selected_cycle_id"] not in [
            cycle["cycle_id"] for cycle in cycles]:
        raise ValueError("frozen preregistered 20-cycle window differs")
    goal = json.loads((goal_trial / "runtime_audit.json").read_text())[
        "navigation_result"]["goal"]
    goal_truth, goal_times, goal_body, goal_box, goal_inputs = short.trial_input(
        goal_trial)
    return {
        "schema": "rm_dynamic_prediction_filtered_rollout_risk_probe/v1",
        "scope": "Same captured MPPI controls and critic costs. Individual controls are filtered before risk scoring; future Gazebo truth labels offline geometry only. Not a runtime implementation.",
        "fixed_collision_unit": "(3.81 / 254) * 1000000 / 9",
        "horizons_s": HORIZONS_S,
        "collision_trial": {"inputs": collision_inputs,
                            "selection_sha256": short.digest(
                                collision_trial / "selection.json"),
                            "selected_cycle_id": selection[
                                "selected_cycle_id"],
                            "window_sim_s": [start, stop],
                            "window_cycles": cycles},
        "goal_trial": {"inputs": goal_inputs,
                       "cycle_263": one_cycle(
                           goal_trial, 263, goal_truth, goal_times,
                           goal_body, goal_box, goal)},
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
    print(json.dumps({"window_cycles": len(result["collision_trial"][
        "window_cycles"]), "output": str(args.output)}, sort_keys=True))
