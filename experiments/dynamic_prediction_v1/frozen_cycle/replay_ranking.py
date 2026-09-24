#!/usr/bin/env python3
"""Replay a fixed MPPI cycle's aggregation after replacing only V1 scores.

All rollouts, other critic costs, sampled controls, optimizer settings, and
Savitzky-Golay history come from the same captured trial. Gazebo future poses
only label the offline open-loop result. This is not a new closed-loop run.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
import occupancy_rank_probe
import rank_probe


FILTER = np.asarray((-21, 14, 39, 54, 59, 54, 39, 14, -21),
                    dtype=np.float32) / np.float32(231.)


def smooth_axis(values, history):
    """Nav2 1.1.20's in-place 9-point filter, including its end indexing."""
    sequence = np.asarray(values, dtype=np.float32).copy()
    history = [np.float32(value) for value in history]
    last = len(sequence) - 1
    if last < 20:
        return sequence

    def filtered(points):
        if len(points) != 9:
            raise ValueError("filter window must have 9 points")
        return np.sum(np.asarray(points, dtype=np.float32) * FILTER,
                      dtype=np.float32)

    for index in range(4):
        sequence[index] = filtered(history[index:] + list(sequence[:index + 5]))
    for index in range(4, last - 4):
        sequence[index] = filtered(list(sequence[index - 4:index + 5]))
    index = last - 3
    sequence[index] = filtered(list(sequence[index - 4:index + 4]) +
                               [sequence[index + 3]])
    index += 1
    sequence[index] = filtered(list(sequence[index - 4:index + 3]) +
                               [sequence[index + 2]] * 2)
    index += 1
    sequence[index] = filtered(list(sequence[index - 4:index + 2]) +
                               [sequence[index + 1]] * 3)
    index += 1
    sequence[index] = filtered(list(sequence[index - 4:index + 1]) +
                               [sequence[index]] * 4)
    return sequence


def probabilities(costs, temperature):
    costs = np.asarray(costs, dtype=np.float32)
    exponent = np.exp(-np.float32(1. / temperature) *
                      (costs - np.min(costs)))
    return exponent / np.sum(exponent, dtype=np.float32)


def history_from_trial(cycle_dir, cycle_id, current_arrays):
    history = {name: [] for name in ("vx", "vy", "wz")}
    for predecessor_id in range(cycle_id - 4, cycle_id):
        path = cycle_dir / f"cycle_{predecessor_id}.json"
        meta, arrays = analyze.read_cycle(path)
        if meta.get("unwinding_exception") or analyze.event(meta, "scored.fail"):
            raise ValueError(f"predecessor cycle {predecessor_id} failed")
        offset = analyze.event(meta, "settings")["offset"]
        for name in history:
            history[name].append(float(analyze.last(arrays,
                                                   "after_filter." + name)[offset]))
        if predecessor_id == cycle_id - 1:
            for name in history:
                previous = analyze.last(arrays, "after_filter." + name)
                shifted = np.roll(previous, -1)
                shifted[-1] = shifted[-2]
                initial = analyze.last(current_arrays, "initial." + name)
                if not np.array_equal(shifted, initial):
                    raise ValueError("initial sequence does not follow prior cycle")
    return history


def aggregate(arrays, settings, history, weights):
    sequences = {}
    for name, sampled in (("vx", "cvx"), ("vy", "cvy"), ("wz", "cwz")):
        values = analyze.last(arrays, "sampled." + sampled)
        sequence = np.sum(values * weights[:, None], axis=0, dtype=np.float32)
        if name == "vx":
            sequence = np.clip(sequence, settings["vx_min"], settings["vx_max"])
        elif name == "vy":
            sequence = np.clip(sequence, -settings["vy_max"], settings["vy_max"])
        else:
            sequence = np.clip(sequence, -settings["wz_max"], settings["wz_max"])
        sequences[name] = smooth_axis(sequence, history[name])
    return sequences


def open_loop_gap(sequences, meta, settings, truth, truth_times, body, box,
                  consumed_t):
    trajectory = analyze.integrate_omni(
        *(sequences[key] for key in ("vx", "vy", "wz")), meta["pose"],
        settings["dt"])
    gaps = [analyze.polygon_distance(
        analyze.placed(body, tuple(float(axis[j]) for axis in trajectory)),
        analyze.placed(box, analyze.interpolated_pose(
            truth, truth_times, consumed_t + (j + 1) * settings["dt"])))
        for j in range(settings["steps"])]
    return min(gaps), gaps.index(min(gaps)) + 1


def replay(cycle_path, profile_path, truth_path, history):
    baseline, records = analyze.analyze(cycle_path, profile_path, truth_path)
    meta, arrays = analyze.read_cycle(cycle_path)
    settings = analyze.event(meta, "settings")
    original_weights = analyze.last(arrays, "weighted.probability")
    original_costs = analyze.last(arrays, "weighted.costs")
    weights_check = probabilities(original_costs, settings["temperature"])
    if np.max(np.abs(weights_check - original_weights)) > 1e-6:
        raise ValueError("softmax replay differs from capture")
    control_check = aggregate(arrays, settings, history, original_weights)
    control_error = max(float(np.max(np.abs(control_check[name] -
                                            analyze.last(arrays, "after_filter." + name))))
                        for name in control_check)
    if control_error > 1e-5:
        raise ValueError(f"control filter replay differs from capture: {control_error}")
    params = yaml.safe_load(profile_path.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    overlaps = rank_probe.overlap_fractions(meta, arrays, params)
    collisions = np.asarray([r["first_predicted_conflict_s"] is not None
                             for r in records], dtype=bool)
    safe = np.asarray([r["truth_dynamic_safe"] and not r["costcritic_collision"]
                       for r in records], dtype=bool)
    truth = analyze.rows_from_transport(truth_path)
    truth_times = [row["t"] for row in truth]
    local = yaml.safe_load(profile_path.read_text())["local_costmap"][
        "local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    box = analyze.obstacle_polygon()
    original_term = (3.81 / 254.) * 1_000_000. / baseline["prediction_steps"]
    # Two fixed, interpretable changes to the *shape* of collision scoring:
    # existing near penalty as a gentle gradient, or the existing collision
    # penalty multiplied by mean overlap. No critic weight or input changes.
    variants = {
        "baseline": np.zeros(len(overlaps), dtype=np.float32),
        "near_scale_gradient": np.asarray(
            collisions * (3.81 / 254.) * 300. * overlaps, dtype=np.float32),
        "collision_scale_gradient": np.asarray(
            collisions * original_term * overlaps, dtype=np.float32),
        "uniform_center_gradient": np.asarray(
            collisions * original_term *
            occupancy_rank_probe.expected_overlap(meta, arrays, params),
            dtype=np.float32),
    }
    outcomes = {}
    for name, delta in variants.items():
        weights = original_weights if name == "baseline" else probabilities(
            original_costs + delta, settings["temperature"])
        controls = control_check if name == "baseline" else aggregate(
            arrays, settings, history, weights)
        gap, step = open_loop_gap(controls, meta, settings, truth, truth_times,
                                  body, box, baseline["consumer_sim_s"])
        offset = settings["offset"]
        outcomes[name] = {
            "safe_rollout_probability_mass": float(np.sum(weights[safe])),
            "open_loop_body_min_gap_m": gap,
            "open_loop_body_min_gap_step": step,
            "returned_control": [float(controls[key][offset])
                                 for key in ("vx", "vy", "wz")],
        }
    if max(abs(a - b) for a, b in zip(outcomes["baseline"]["returned_control"],
                                      baseline["returned_control"])) > 1e-6:
        raise ValueError("baseline command differs from capture")
    return {"cycle_id": baseline["cycle_id"], "cycle_sha256": baseline["source"]["cycle_sha256"],
            "safe_rollouts": int(safe.sum()),
            "predicted_collision_rollouts": int(collisions.sum()),
            "baseline_replay_max_control_error": control_error,
            "overlap_fraction_span": float(np.ptp(overlaps)),
            "outcomes": outcomes}


def run_window(trial):
    selection = json.loads((trial / "selection.json").read_text())
    start, stop = selection["witness_sim_s"] - 2., selection["witness_sim_s"]
    cycle_dir = trial / "mppi_cycles"
    profile = trial / "profile.yaml"
    truth = trial / "gazebo_poses.jsonl"
    rows = []
    for path in sorted(cycle_dir.glob("cycle_*.json"),
                       key=lambda file: int(file.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        t = meta["sim_ns"] / 1e9
        if start <= t <= stop and any(e["kind"] == "prediction.input"
                                      for e in meta["events"]):
            history = history_from_trial(cycle_dir, meta["cycle_id"],
                                         analyze.read_cycle(path)[1])
            rows.append(replay(path, profile, truth, history))
    if not rows:
        raise ValueError("no complete frozen cycles in window")
    return {"schema": "rm_dynamic_prediction_frozen_ranking_replay/v1",
            "scope": "Same-run frozen rollouts/critic costs; only V1 collision rank changed. Offline future geometry is not an online input.",
            "window_sim_s": [start, stop],
            "selected_cycle_id": selection["selected_cycle_id"],
            "cycles": rows}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("replay output exists")
    result = run_window(args.trial)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    for row in result["cycles"]:
        print(json.dumps({"cycle_id": row["cycle_id"], "safe": row["safe_rollouts"],
                          "predicted_collision": row["predicted_collision_rollouts"],
                          "outcomes": row["outcomes"]}))
