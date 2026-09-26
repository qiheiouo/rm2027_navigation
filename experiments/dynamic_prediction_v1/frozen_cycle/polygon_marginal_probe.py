#!/usr/bin/env python3
"""Check whether preserving the conditional center-velocity polygon helps.

This is an offline distribution hypothesis. Neither a uniform polygon prior
nor a uniform acceleration prior is calibrated to real-world probabilities.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
from envelope import AxisBox
import multi_scan_bound_probe as multi
import proximity_margin_probe as proximity
import rank_probe
import replay_ranking
import short_horizon_filter_probe as short


CHECKS = (("collision", 147), ("collision", 162), ("goal", 263))


def area(vertices):
    return abs(sum(a[0] * b[1] - a[1] * b[0]
                   for a, b in zip(vertices, vertices[1:] + vertices[:1]))) / 2


def marginal_quantile(polygon, horizon, fraction):
    values = [center + horizon * velocity for center, velocity in polygon]
    lower, upper = min(values), max(values)
    total = area(polygon)
    if total <= 1e-12:
        raise ValueError("empty conditional center-velocity polygon")
    for _ in range(35):
        midpoint = (lower + upper) / 2
        left = multi.clip_polygon(polygon, 1., horizon, midpoint)
        if area(left) / total < fraction:
            lower = midpoint
        else:
            upper = midpoint
    return (lower + upper) / 2


def projected_samples(polygon, horizon, growth):
    # Five equal-mass projection quantiles, each convolved with a two-node
    # uniform acceleration displacement quadrature; total weight is one.
    samples = []
    for fraction in (.1, .3, .5, .7, .9):
        nominal = marginal_quantile(polygon, horizon, fraction)
        for acceleration_node in (-1 / math.sqrt(3), 1 / math.sqrt(3)):
            samples.append((nominal + acceleration_node * growth, .1))
    return samples


def polygon_risk(meta, arrays, params, model, source):
    prediction = analyze.event(meta, "prediction.input")
    dt = analyze.event(meta, "settings")["dt"]
    x, y, yaw = (analyze.last(arrays, "rollout." + axis)
                 for axis in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(math.floor(float(params["horizon"]) /
                                           dt + 1e-9)))
    if steps != 9:
        raise ValueError("frozen V1 horizon differs from nine steps")
    extent = (float(params["object_width"]),
              float(params["object_height"]))
    risk = np.zeros(x.shape[0], dtype=np.float64)
    footprint = meta["padded_footprint"]
    acceleration = model["stipulated_acceleration_mps2"]
    for step in range(steps):
        horizon = prediction["source_age_s"] + (step + 1) * dt
        growth = .5 * acceleration * horizon * horizon
        xs, ys = (projected_samples(polygon, horizon, growth)
                  for polygon in source["multi_polygons"])
        boxes = []
        for cx, wx in xs:
            for cy, wy in ys:
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
                    separation = -math.sqrt(rank_probe.overlap_area(
                        robot, box))
                value += (weight * max(0., proximity.MARGIN_M - separation) /
                          proximity.MARGIN_M)
            risk[index] += value / steps
    return risk


def one_check(trial, cycle_id, model, sources):
    path = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile = trial / "profile.yaml"
    summary, records = analyze.analyze(
        path, profile, trial / "gazebo_poses.jsonl")
    meta, arrays = analyze.read_cycle(path)
    settings = analyze.event(meta, "settings")
    params = yaml.safe_load(profile.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    prediction = analyze.event(meta, "prediction.input")
    source = sources.get(round(prediction["source_stamp_ns"] / 1e9, 6))
    if source is None:
        raise ValueError(f"cycle {cycle_id}: no conditional polygon")
    history = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    alternative = proximity.filtered_arrays(meta, arrays, settings, history)
    truth, times, body, box, inputs = short.trial_input(trial)
    old_costs = analyze.last(arrays, "weighted.costs")
    old_prediction = analyze.critic_deltas(arrays)[0][
        "FollowPath.PredictionV1Critic"]
    unit = np.float32((3.81 / 254.) * 1_000_000. /
                      summary["prediction_steps"])
    safe = np.asarray([record["truth_dynamic_safe"] and not
                       record["costcritic_collision"] for record in records])
    goal = None
    goal_mask = None
    if (trial / "runtime_audit.json").exists():
        goal = json.loads((trial / "runtime_audit.json").read_text())[
            "navigation_result"]["goal"]
        x, y = (analyze.last(arrays, "rollout." + axis)
                for axis in ("x", "y"))
        goal_mask = np.hypot(x[:, -1] - goal[0],
                             y[:, -1] - goal[1]) <= .15
    output = {"cycle_id": cycle_id,
              "cycle_json_sha256": short.digest(path),
              "cycle_bin_sha256": short.digest(path.with_suffix(".bin")),
              "source_stamp_ns": prediction["source_stamp_ns"],
              "conditional_history_count": source["multi_history_count"],
              "raw_safe_rollouts": int(safe.sum()),
              "inputs": inputs,
              "variants": {}}
    for name, risk in (
            ("uniform_projected_interval", proximity.proximity_risk(
                meta, alternative, params, model=model, row=source)),
            ("uniform_feasible_polygon_marginal", polygon_risk(
                meta, alternative, params, model, source))):
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
    model = json.loads(model_evidence.read_text())["model"]
    occupancy, _ = proximity.static_map()
    sources = {
        "collision": proximity.source_rows(
            collision_trial, model, occupancy),
        "goal": proximity.source_rows(goal_trial, model, occupancy),
    }
    trials = {"collision": collision_trial, "goal": goal_trial}
    return {
        "schema": "rm_dynamic_prediction_polygon_marginal_probe/v1",
        "scope": "Fixed three previously identified diagnostic cycles; no score parameter fitted to truth. Uniform feasible-polygon and acceleration assumptions are uncalibrated.",
        "margin_m": proximity.MARGIN_M,
        "model_evidence_sha256": short.digest(model_evidence),
        "checks": {f"{key}_{cycle}": one_check(
            trials[key], cycle, model, sources[key])
                   for key, cycle in CHECKS},
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
    print(json.dumps({"checks": sorted(result["checks"]),
                      "output": str(args.output)}, sort_keys=True))
