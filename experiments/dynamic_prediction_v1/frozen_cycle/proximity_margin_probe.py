#!/usr/bin/env python3
"""Offline expected proximity risk on individually filtered MPPI samples.

The 0.15 m proximity scale is the unchanged 0.05 m body gate plus roughly
0.10 m stopping travel measured in the documented simulation. This scale is
not a real-robot stopping guarantee. Future box truth is evaluation only.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
from envelope import AxisBox
from heldout_geometry_audit import recorded_messages
import multi_scan_bound_probe as multi
import rank_probe
import replay_ranking
from raw_scan_support_audit import one_trial, static_map
import short_horizon_filter_probe as short


MARGIN_M = .15
QUADRATURE_ORDER = 5


def source_rows(trial, model, occupancy):
    _, rows = one_trial(trial, recorded_messages(trial), occupancy,
                        return_source_rows=True)
    return {round(row["source_t"], 6): row
            for row in multi.multi_rows(model, rows)}


def center_support(prediction, params, duration, model=None, row=None):
    extent = (float(params["object_width"]),
              float(params["object_height"]))
    if row is not None:
        bound = multi.box(model, row, duration)
        if bound is None:
            raise ValueError("empty conditional scan bound")
        center = ((bound.min_x + bound.max_x) / 2,
                  (bound.min_y + bound.max_y) / 2)
        support = ((bound.max_x - bound.min_x - extent[0]) / 2,
                   (bound.max_y - bound.min_y - extent[1]) / 2)
        if min(support) < -1e-9:
            raise ValueError("conditional support smaller than physical box")
        return center, support
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("single-box fixture required")
    track = tracks[0]
    mismatch = .5 * float(params["reference_acceleration"]) * duration ** 2
    center = tuple(track["xy"][axis] + track["vxy"][axis] * duration
                   for axis in (0, 1))
    support = tuple(track["size_xy"][axis] / 2 + extent[axis] / 2 + mismatch
                    for axis in (0, 1))
    return center, support


def proximity_risk(meta, arrays, params, model=None, row=None):
    prediction = analyze.event(meta, "prediction.input")
    dt = analyze.event(meta, "settings")["dt"]
    x, y, yaw = (analyze.last(arrays, "rollout." + axis)
                 for axis in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(math.floor(float(params["horizon"]) /
                                           dt + 1e-9)))
    if steps != 9:
        raise ValueError("frozen prediction horizon differs from nine steps")
    footprint = meta["padded_footprint"]
    extent = (float(params["object_width"]),
              float(params["object_height"]))
    nodes, weights = np.polynomial.legendre.leggauss(QUADRATURE_ORDER)
    weights /= 2.
    risk = np.zeros(x.shape[0], dtype=np.float64)
    for step in range(steps):
        duration = prediction["source_age_s"] + (step + 1) * dt
        center, support = center_support(prediction, params, duration,
                                         model=model, row=row)
        boxes = []
        for nx, wx in zip(nodes, weights):
            for ny, wy in zip(nodes, weights):
                cx, cy = (center[0] + nx * support[0],
                          center[1] + ny * support[1])
                box = AxisBox(cx - extent[0] / 2, cy - extent[1] / 2,
                              cx + extent[0] / 2, cy + extent[1] / 2)
                boxes.append((box, box.polygon(), wx * wy))
        for index in range(x.shape[0]):
            robot = analyze.placed(footprint, (
                float(x[index, step]), float(y[index, step]),
                float(yaw[index, step])))
            value = 0.
            for box, polygon, weight in boxes:
                separation = analyze.polygon_distance(robot, polygon)
                if separation <= 1e-9:
                    # sqrt(area) gives overlap a length scale; this graded
                    # continuation is a ranking hypothesis, not a probability.
                    separation = -math.sqrt(rank_probe.overlap_area(
                        robot, box))
                value += weight * max(0., MARGIN_M - separation) / MARGIN_M
            risk[index] += value / steps
    return risk


def filtered_arrays(meta, arrays, settings, history):
    xyz = [[], [], []]
    for index in range(settings["batch"]):
        controls = short.filtered(short.clipped_sequences(
            arrays, settings, rollout=index), history)
        trajectory = analyze.integrate_omni(
            *(controls[axis] for axis, _ in short.AXES),
            meta["pose"], settings["dt"])
        for axis in range(3):
            xyz[axis].append(trajectory[axis])
    return arrays + [("rollout." + name, np.stack(values))
                     for name, values in zip(("x", "y", "yaw"), xyz)]


def one_cycle(trial, cycle_id, model, sources, truth, times, body, box,
              goal=None):
    path = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile = trial / "profile.yaml"
    summary, records = analyze.analyze(
        path, profile, trial / "gazebo_poses.jsonl")
    meta, arrays = analyze.read_cycle(path)
    settings = analyze.event(meta, "settings")
    params = yaml.safe_load(profile.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    history = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    old_weights = analyze.last(arrays, "weighted.probability")
    baseline = replay_ranking.aggregate(arrays, settings, history, old_weights)
    error = max(float(np.max(np.abs(baseline[axis] - analyze.last(
        arrays, "after_filter." + axis)))) for axis, _ in short.AXES)
    if error > 1e-5:
        raise ValueError(f"cycle {cycle_id}: baseline control replay mismatch")
    alternative = filtered_arrays(meta, arrays, settings, history)
    prediction = analyze.event(meta, "prediction.input")
    row = sources.get(round(prediction["source_stamp_ns"] / 1e9, 6))
    old_costs = analyze.last(arrays, "weighted.costs")
    old_prediction = analyze.critic_deltas(arrays)[0][
        "FollowPath.PredictionV1Critic"]
    unit = np.float32((3.81 / 254.) * 1_000_000. /
                      summary["prediction_steps"])
    safe = np.asarray([record["truth_dynamic_safe"] and not
                       record["costcritic_collision"] for record in records])
    goal_mask = None
    if goal is not None:
        x, y = (analyze.last(arrays, "rollout." + axis)
                for axis in ("x", "y"))
        goal_mask = np.hypot(x[:, -1] - goal[0],
                             y[:, -1] - goal[1]) <= .15
    output = {"cycle_id": cycle_id,
              "cycle_json_sha256": short.digest(path),
              "cycle_bin_sha256": short.digest(path.with_suffix(".bin")),
              "baseline_control_max_error": error,
              "conditional_source_available": row is not None,
              "raw_safe_rollouts": int(safe.sum()),
              "variants": {}}
    for name, source in (("tracker_support", None),
                         ("conditional_scan_or_tracker_fallback", row)):
        risk = proximity_risk(meta, alternative, params,
                              model=model if source is not None else None,
                              row=source)
        weights = replay_ranking.probabilities(
            old_costs - old_prediction + np.float32(unit * risk),
            settings["temperature"])
        controls = replay_ranking.aggregate(
            arrays, settings, history, weights)
        result = {
            "risk_span": float(np.ptp(risk)),
            "raw_safe_rollout_weight": float(weights[safe].sum()),
            "effective_sample_size": float(1. / np.sum(weights * weights)),
            "top_rollout": int(np.argmax(weights)),
            "filtered_aggregate_gap": short.control_gap(
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
        output["variants"][name] = result
    return output


def run(collision_trial, goal_trial, model_evidence):
    model_record = json.loads(model_evidence.read_text())
    model = model_record["model"]
    occupancy, _ = static_map()
    collision_sources = source_rows(collision_trial, model, occupancy)
    goal_sources = source_rows(goal_trial, model, occupancy)
    selection = json.loads((collision_trial / "selection.json").read_text())
    start, stop = selection["witness_sim_s"] - 2., selection["witness_sim_s"]
    truth, times, body, box, collision_inputs = short.trial_input(
        collision_trial)
    cycles = []
    for path in sorted((collision_trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        if (start <= meta["sim_ns"] / 1e9 <= stop and
                any(event["kind"] == "prediction.input"
                    for event in meta["events"])):
            cycles.append(one_cycle(collision_trial, meta["cycle_id"], model,
                                    collision_sources, truth, times, body,
                                    box))
    if len(cycles) != 20:
        raise ValueError("frozen 20-cycle window differs")
    goal = json.loads((goal_trial / "runtime_audit.json").read_text())[
        "navigation_result"]["goal"]
    goal_truth, goal_times, goal_body, goal_box, goal_inputs = short.trial_input(
        goal_trial)
    return {
        "schema": "rm_dynamic_prediction_proximity_margin_probe/v1",
        "scope": "Offline same-input score replacement only. Uniform center interval and signed-distance continuation are ranking assumptions, not collision probabilities. Future truth evaluates but never scores.",
        "margin_m": MARGIN_M,
        "margin_basis": "0.05 m body gate + approximately 0.10 m simulated stopping travel",
        "quadrature_order_per_axis": QUADRATURE_ORDER,
        "fixed_collision_unit": "(3.81 / 254) * 1000000 / 9",
        "model_evidence_sha256": short.digest(model_evidence),
        "collision_trial": {"inputs": collision_inputs,
                            "selection_sha256": short.digest(
                                collision_trial / "selection.json"),
                            "selected_cycle_id": selection[
                                "selected_cycle_id"],
                            "window_sim_s": [start, stop],
                            "cycles": cycles},
        "goal_trial": {"inputs": goal_inputs,
                       "cycle_263": one_cycle(
                           goal_trial, 263, model, goal_sources, goal_truth,
                           goal_times, goal_body, goal_box, goal)},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collision-trial", type=Path, required=True)
    parser.add_argument("--goal-trial", type=Path, required=True)
    parser.add_argument("--model-evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.collision_trial, args.goal_trial, args.model_evidence)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"window_cycles": len(result["collision_trial"][
        "cycles"]), "output": str(args.output)}, sort_keys=True))
