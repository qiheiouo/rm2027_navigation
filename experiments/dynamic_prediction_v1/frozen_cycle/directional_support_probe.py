#!/usr/bin/env python3
"""Offline oracle bound on view-conditioned box-center support.

The residual interval is fitted with future Gazebo truth from this same trial.
It is a feasibility diagnostic, never a runtime prediction or safety bound.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

import numpy as np
import yaml

from analyze import (critic_deltas, event, integrate_omni, interpolated_pose,
                     last, placed, polygon_distance, read_cycle,
                     rows_from_transport, sha)
from envelope import AxisBox, predicted_box
import replay_ranking


def probe(trial):
    trial = Path(trial)
    plan = json.loads((trial.parent / "plan.json").read_text())
    profile_file = trial / "profile.yaml"
    if sha(profile_file) != plan["derived_profile_sha256"]:
        raise ValueError("trial profile differs from pre-registration")
    profile = yaml.safe_load(profile_file.read_text())
    follow = profile["controller_server"]["ros__parameters"]["FollowPath"]
    params = follow["PredictionV1Critic"]
    extent = (float(params["object_width"]), float(params["object_height"]))
    audit = json.loads((trial / "runtime_audit.json").read_text())
    first_near_goal = audit["first_cycle_within_0_5m_goal"]["sim_s"]
    goal = tuple(audit["navigation_result"]["goal"])
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in truth]
    residuals = defaultdict(list)
    cycles = defaultdict(list)
    current_coverage_misses = defaultdict(list)
    worst_details = None
    worst_deficit = 0.
    # The 0.6 m side threshold is an exploratory geometric partition,
    # exceeding the known object half-width plus padded robot half-width.
    side_threshold = .6
    for file in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta = json.loads(file.read_text())
        inputs = [entry["value"] for entry in meta["events"]
                  if entry["kind"] == "prediction.input"]
        if not inputs or inputs[0]["status"] != "accepted":
            continue
        prediction = inputs[0]
        tracks = [track for track in prediction["tracks"] if track["state"] == 2]
        if len(tracks) != 1:
            raise ValueError("this fixture requires one confirmed moving box")
        track = tracks[0]
        dt = next(entry["value"]["dt"] for entry in meta["events"]
                  if entry["kind"] == "settings")
        steps = min(int(follow["time_steps"]),
                    int(math.floor(float(params["horizon"]) / dt + 1e-9)))
        side = meta["pose"][0] - track["xy"][0]
        group = "east" if side > side_threshold else (
            "west" if side < -side_threshold else "middle")
        accepted = []
        for j in range(steps):
            duration = prediction["source_age_s"] + (j + 1) * dt
            t = prediction["consumer_sim_s"] + (j + 1) * dt
            if t >= times[-1]:
                continue
            actual = interpolated_pose(truth, times, t)
            if abs(actual[2]) > 1e-4:
                raise ValueError("rotated obstacle needs an oriented-box oracle")
            nominal = tuple(track["xy"][axis] + track["vxy"][axis] * duration
                            for axis in (0, 1))
            delta = tuple(actual[axis] - nominal[axis] for axis in (0, 1))
            current_box = predicted_box(
                track["xy"], track["vxy"], track["size_xy"], extent,
                prediction["source_age_s"], (j + 1) * dt,
                float(params["reference_acceleration"]))
            deficit = max(0.,
                          current_box.min_x - (actual[0] - extent[0] / 2),
                          (actual[0] + extent[0] / 2) - current_box.max_x,
                          current_box.min_y - (actual[1] - extent[1] / 2),
                          (actual[1] + extent[1] / 2) - current_box.max_y)
            if deficit > 1e-9:
                current_coverage_misses[group].append((meta["cycle_id"], j + 1,
                                                        t, deficit))
                if deficit > worst_deficit:
                    worst_deficit = deficit
                    worst_details = {
                        "cycle_id": meta["cycle_id"], "future_step": j + 1,
                        "future_sim_s": t, "uncovered_edge_m": deficit,
                        "robot_pose": meta["pose"],
                        "prediction_source_age_s": prediction["source_age_s"],
                        "track_xy": track["xy"], "track_vxy": track["vxy"],
                        "visible_size_xy": track["size_xy"],
                        "predicted_box": [current_box.min_x, current_box.min_y,
                                          current_box.max_x, current_box.max_y],
                        "actual_box": [actual[0] - extent[0] / 2,
                                       actual[1] - extent[1] / 2,
                                       actual[0] + extent[0] / 2,
                                       actual[1] + extent[1] / 2],
                    }
            residuals[group].append(delta)
            accepted.append((actual, nominal))
        if accepted:
            cycles[group].append((meta, prediction, track, dt, accepted))
    east = np.asarray(residuals["east"], dtype=np.float64)
    if east.shape[0] < 2:
        raise ValueError("no east-side residual evidence")
    minimum, maximum = east.min(axis=0), east.max(axis=0)
    late = [cycle for cycle in cycles["east"]
            if cycle[0]["sim_ns"] / 1e9 >= first_near_goal]
    observations = []
    for meta, prediction, track, dt, future in late:
        actual, nominal = future[0]
        goal_shape = placed(meta["padded_footprint"], goal)
        current = predicted_box(track["xy"], track["vxy"], track["size_xy"],
                                extent, prediction["source_age_s"], dt,
                                float(params["reference_acceleration"]))
        row = {"cycle_id": meta["cycle_id"],
               "current_goal_gap_m": polygon_distance(goal_shape, current.polygon()),
               "actual_goal_gap_m": polygon_distance(goal_shape, AxisBox(
                   actual[0] - extent[0] / 2, actual[1] - extent[1] / 2,
                   actual[0] + extent[0] / 2, actual[1] + extent[1] / 2).polygon())}
        for margin in (0., .02, .05):
            hypothetical = AxisBox(
                nominal[0] + minimum[0] - margin - extent[0] / 2,
                nominal[1] + minimum[1] - margin - extent[1] / 2,
                nominal[0] + maximum[0] + margin + extent[0] / 2,
                nominal[1] + maximum[1] + margin + extent[1] / 2)
            row[f"oracle_margin_{margin:.2f}_goal_gap_m"] = polygon_distance(
                goal_shape, hypothetical.polygon())
        observations.append(row)
    by_side = {}
    for group, values in residuals.items():
        array = np.asarray(values, dtype=np.float64)
        by_side[group] = {
            "cycles": len(cycles[group]), "future_samples": len(values),
            "residual_xy_min_m": array.min(axis=0).tolist(),
            "residual_xy_max_m": array.max(axis=0).tolist(),
            "positive_x_residual_samples": int(np.count_nonzero(array[:, 0] > 0)),
            "current_v1_uncovered_future_samples": len(current_coverage_misses[group]),
            "current_v1_max_uncovered_edge_m": max(
                (row[3] for row in current_coverage_misses[group]), default=0.),
        }
    goal_overlap = {}
    for name in ("current", "oracle_margin_0.00", "oracle_margin_0.02",
                 "oracle_margin_0.05", "actual"):
        gaps = [row[f"{name}_goal_gap_m"] for row in observations]
        goal_overlap[name] = {
            "overlap_cycles": sum(gap <= 1e-9 for gap in gaps),
            "minimum_gap_m": min(gaps),
            "median_gap_m": float(np.median(gaps)),
            "at_least_0_02m_gap_cycles": sum(gap >= .02 for gap in gaps),
        }
    selected = next(row for row in observations
                    if row["cycle_id"] == audit["first_cycle_within_0_5m_goal"]["cycle_id"])
    selected_id = selected["cycle_id"]
    meta, arrays = read_cycle(trial / "mppi_cycles" / f"cycle_{selected_id}.json")
    prediction = event(meta, "prediction.input")
    track = next(track for track in prediction["tracks"] if track["state"] == 2)
    settings = event(meta, "settings")
    dt = settings["dt"]
    steps = min(int(follow["time_steps"]),
                int(math.floor(float(params["horizon"]) / dt + 1e-9)))
    x, y, yaw = (last(arrays, f"rollout.{axis}") for axis in ("x", "y", "yaw"))
    # Even with 0.05 m extra oracle-center uncertainty on all sides, every
    # selected-cycle rollout has positive predicted margin. This is only an
    # offline upper bound on what a better observation model could enable.
    margin = .05
    oracle_gaps = []
    for i in range(x.shape[0]):
        minimum_gap = math.inf
        for j in range(steps):
            duration = prediction["source_age_s"] + (j + 1) * dt
            nominal = [track["xy"][axis] + track["vxy"][axis] * duration
                       for axis in (0, 1)]
            box = AxisBox(nominal[0] + minimum[0] - margin - extent[0] / 2,
                          nominal[1] + minimum[1] - margin - extent[1] / 2,
                          nominal[0] + maximum[0] + margin + extent[0] / 2,
                          nominal[1] + maximum[1] + margin + extent[1] / 2)
            footprint = placed(meta["padded_footprint"],
                               (float(x[i, j]), float(y[i, j]), float(yaw[i, j])))
            minimum_gap = min(minimum_gap, polygon_distance(footprint, box.polygon()))
        oracle_gaps.append(minimum_gap)
    if min(oracle_gaps) < .02:
        raise ValueError("selected cycle has oracle near/collision score to recompute")
    original_weights = last(arrays, "weighted.probability")
    original_costs = last(arrays, "weighted.costs")
    terms, _ = critic_deltas(arrays)
    old_prediction_term = terms["FollowPath.PredictionV1Critic"].astype(np.float32)
    new_weights = replay_ranking.probabilities(
        original_costs.astype(np.float32) - old_prediction_term,
        settings["temperature"])
    history = replay_ranking.history_from_trial(trial / "mppi_cycles", selected_id, arrays)
    baseline_control = replay_ranking.aggregate(arrays, settings, history, original_weights)
    baseline_replay_error = max(float(np.max(np.abs(
        baseline_control[axis] - last(arrays, f"after_filter.{axis}"))))
        for axis in ("vx", "vy", "wz"))
    if baseline_replay_error > 1e-5:
        raise ValueError("captured control filter does not replay")
    oracle_control = replay_ranking.aggregate(arrays, settings, history, new_weights)
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])

    def control_result(control, weights):
        trajectory = integrate_omni(*(control[axis] for axis in ("vx", "vy", "wz")),
                                    meta["pose"], dt)
        end_distance = math.hypot(float(trajectory[0][-1]) - goal[0],
                                  float(trajectory[1][-1]) - goal[1])
        body_gap, _ = replay_ranking.open_loop_gap(
            control, meta, settings, truth, times, body,
            [(-extent[0] / 2, -extent[1] / 2),
             (extent[0] / 2, -extent[1] / 2),
             (extent[0] / 2, extent[1] / 2),
             (-extent[0] / 2, extent[1] / 2)], prediction["consumer_sim_s"])
        goal_end = np.hypot(x[:, -1] - goal[0], y[:, -1] - goal[1]) <= .15
        return {"returned_control": [float(control[axis][settings["offset"]])
                                     for axis in ("vx", "vy", "wz")],
                "open_loop_endpoint_goal_distance_m": end_distance,
                "open_loop_truth_body_min_gap_m": body_gap,
                "goal_position_rollout_probability_mass": float(np.sum(weights[goal_end]))}

    counterfactual = {
        "fixed_rollout_count": int(x.shape[0]),
        "oracle_0_05m_support_rollouts_with_predicted_gap_at_least_0_02m":
            sum(gap >= .02 for gap in oracle_gaps),
        "baseline_filter_replay_max_error": baseline_replay_error,
        "baseline": control_result(baseline_control, original_weights),
        "oracle_input_zero_v1_score": control_result(oracle_control, new_weights),
    }
    return {
        "schema": "rm_dynamic_prediction_directional_oracle_feasibility/v1",
        "scope": "Single phase-4 trial, original accepted predictions, exact V1 effective horizon. Gazebo truth determines residual bounds and evaluation in the same trial; no independent validation or online candidate.",
        "trial": str(trial),
        "source_sha256": {"plan": sha(trial.parent / "plan.json"),
                          "profile": sha(profile_file),
                          "truth": sha(trial / "gazebo_poses.jsonl"),
                          "runtime_audit": sha(trial / "runtime_audit.json"),
                          "probe_script": sha(__file__)},
        "east_west_threshold_m": side_threshold,
        "residual_by_side": by_side,
        "current_v1_coverage": {
            "future_samples": sum(len(values) for values in residuals.values()),
            "uncovered_future_samples": sum(len(values) for values in
                                            current_coverage_misses.values()),
            "uncovered_cycles": len({row[0] for values in current_coverage_misses.values()
                                     for row in values}),
            "uncovered_cycle_id_range": [
                min((row[0] for values in current_coverage_misses.values()
                     for row in values), default=None),
                max((row[0] for values in current_coverage_misses.values()
                     for row in values), default=None)],
            "uncovered_future_time_range_s": [
                min((row[2] for values in current_coverage_misses.values()
                     for row in values), default=None),
                max((row[2] for values in current_coverage_misses.values()
                     for row in values), default=None)],
            "maximum_uncovered_edge_m": max(
                (row[3] for values in current_coverage_misses.values()
                 for row in values), default=0.),
            "worst_witness": worst_details,
        },
        "east_oracle_center_residual_min_m": minimum.tolist(),
        "east_oracle_center_residual_max_m": maximum.tolist(),
        "late_east_cycle_count": len(observations),
        "late_east_goal_overlap": goal_overlap,
        "first_near_goal_cycle": selected,
        "selected_cycle_fixed_rollout_counterfactual": counterfactual,
        "limits": "An empirical residual interval fitted to future truth has no safety guarantee for unseen views, occlusions, target shapes, or motion. Shrinking V1's hard envelope from these numbers is not authorized by this probe. Robot footprint, padding, and 0.05 m gate are unchanged."
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("output exists")
    result = probe(args.trial)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ("source_sha256", "limits")}, indent=2))
