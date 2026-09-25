#!/usr/bin/env python3
"""Offline set-membership scan-center bound from multiple past observations."""
import argparse
import json
import math
from pathlib import Path

import yaml

import analyze
from analyze import event, interpolated_pose, last, placed, polygon_distance, read_cycle, rows_from_transport
from causal_scan_bound_probe import fit_source_error, deficit
from envelope import AxisBox
from heldout_geometry_audit import digest, recorded_messages
from raw_scan_support_audit import EXTENT, near_full_span, one_trial, static_map, training_messages


def clip_polygon(vertices, a, b, limit):
    result = []
    if not vertices:
        return result
    previous = vertices[-1]
    previous_value = a * previous[0] + b * previous[1] - limit
    for current in vertices:
        current_value = a * current[0] + b * current[1] - limit
        if (previous_value <= 1e-10) != (current_value <= 1e-10):
            weight = previous_value / (previous_value - current_value)
            result.append((previous[0] + weight * (current[0] - previous[0]),
                           previous[1] + weight * (current[1] - previous[1])))
        if current_value <= 1e-10:
            result.append(current)
        previous, previous_value = current, current_value
    return result


def feasible_polygon(model, row, past, axis):
    observed = row["raw_bbox_mid"][axis]
    bound = model["source_error_xy_m"][axis]
    acceleration = model["stipulated_acceleration_mps2"]
    anchor = past[0]
    anchor_delta = row["source_t"] - anchor["source_t"]
    anchor_center = anchor["raw_bbox_mid"][axis]
    anchor_slack = bound + .5 * acceleration * anchor_delta * anchor_delta
    velocity_min = (observed - bound - anchor_center - anchor_slack) / anchor_delta
    velocity_max = (observed + bound - anchor_center + anchor_slack) / anchor_delta
    polygon = [(observed - bound, velocity_min),
               (observed + bound, velocity_min),
               (observed + bound, velocity_max),
               (observed - bound, velocity_max)]
    for previous in past:
        delta = row["source_t"] - previous["source_t"]
        center = previous["raw_bbox_mid"][axis]
        slack = bound + .5 * acceleration * delta * delta
        polygon = clip_polygon(polygon, 1.0, -delta, center + slack)
        polygon = clip_polygon(polygon, -1.0, delta, -(center - slack))
        if not polygon:
            return []
    for center, velocity in polygon:
        if abs(center - observed) > bound + 1e-8:
            raise ValueError("current-center vertex escaped source error bound")
        for previous in past:
            delta = row["source_t"] - previous["source_t"]
            slack = bound + .5 * acceleration * delta * delta
            if abs(center - velocity * delta -
                   previous["raw_bbox_mid"][axis]) > slack + 1e-8:
                raise ValueError("feasible vertex escaped past-scan bound")
    return polygon


def multi_rows(model, rows):
    recent = []
    result = []
    for row in sorted(rows, key=lambda item: item["source_t"]):
        if not near_full_span(row):
            continue
        recent = [past for past in recent if
                  row["source_t"] - past["source_t"] <= 1.0]
        past = [previous for previous in recent if
                row["source_t"] - previous["source_t"] >= .05]
        if any(row["source_t"] - previous["source_t"] >= .2 for previous in past):
            row["multi_polygons"] = [
                feasible_polygon(model, row, past, axis) for axis in (0, 1)]
            row["multi_history_count"] = len(past)
            result.append(row)
        recent.append(row)
    return result


def box(model, row, horizon):
    extent = []
    acceleration = model["stipulated_acceleration_mps2"]
    for axis, polygon in enumerate(row["multi_polygons"]):
        if not polygon:
            return None
        values = [center + horizon * velocity for center, velocity in polygon]
        growth = .5 * acceleration * horizon * horizon
        extent.append((min(values) - growth - EXTENT[axis] / 2,
                       max(values) + growth + EXTENT[axis] / 2))
    return AxisBox(extent[0][0], extent[1][0], extent[0][1], extent[1][1])


def evaluate(model, sources, truth):
    times = [row["t"] for row in truth]
    output = {"sources": len(sources), "empty_source_sets": 0,
              "future_samples": 0, "uncovered_samples": 0,
              "worst_miss": None}
    for row in sources:
        if any(not polygon for polygon in row["multi_polygons"]):
            output["empty_source_sets"] += 1
            continue
        for step in range(11):
            t = row["source_t"] + step * .1
            if t >= times[-1]:
                break
            actual = interpolated_pose(truth, times, t)
            miss = deficit(box(model, row, step * .1), actual)
            output["future_samples"] += 1
            if miss > 1e-9:
                output["uncovered_samples"] += 1
                if output["worst_miss"] is None or miss > output["worst_miss"]["uncovered_edge_m"]:
                    output["worst_miss"] = {"source_t": row["source_t"],
                                            "step": step, "uncovered_edge_m": miss}
    return output


def consumed_coverage(model, trial, sources):
    by_source = {round(row["source_t"], 6): row for row in sources}
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in truth]
    output = {"cycles": 0, "samples": 0, "uncovered_samples": 0,
              "empty_source_sets": 0, "worst_miss": None}
    for path in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda entry: int(entry.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        candidates = [entry["value"] for entry in meta["events"]
                      if entry["kind"] == "prediction.input"]
        if len(candidates) != 1 or candidates[0].get("status") != "accepted":
            continue
        prediction = candidates[0]
        row = by_source.get(round(prediction["source_stamp_ns"] / 1e9, 6))
        if row is None:
            continue
        output["cycles"] += 1
        if any(not polygon for polygon in row["multi_polygons"]):
            output["empty_source_sets"] += 1
            continue
        dt = event(meta, "settings")["dt"]
        steps = min(30, int(1. / dt + 1e-8))
        for step in range(1, steps + 1):
            duration = prediction["source_age_s"] + step * dt
            t = prediction["consumer_sim_s"] + step * dt
            if t >= times[-1]:
                break
            actual = interpolated_pose(truth, times, t)
            miss = deficit(box(model, row, duration), actual)
            output["samples"] += 1
            if miss > 1e-9:
                output["uncovered_samples"] += 1
                if (output["worst_miss"] is None or
                        miss > output["worst_miss"]["uncovered_edge_m"]):
                    output["worst_miss"] = {
                        "cycle_id": meta["cycle_id"], "step": step,
                        "source_t": row["source_t"], "uncovered_edge_m": miss}
    return output


def key_cycle(model, trial, sources, cycle_id):
    cycle_path = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    meta, arrays = read_cycle(cycle_path)
    prediction = event(meta, "prediction.input")
    settings = event(meta, "settings")
    row = next(row for row in sources if row["source_t"] == prediction["source_stamp_ns"] / 1e9)
    profile = yaml.safe_load((trial / "profile.yaml").read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    goal_path = trial / "runtime_audit.json"
    goal = (tuple(json.loads(goal_path.read_text())["navigation_result"]["goal"])
            if goal_path.exists() else None)
    goal_shape = placed(meta["padded_footprint"], goal) if goal else None
    dt = settings["dt"]
    age = prediction["source_age_s"]
    x, y, yaw = (last(arrays, f"rollout.{axis}") for axis in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(float(params["horizon"]) / dt + 1e-9))
    overlaps = [0] * steps
    first = []
    minimum_gaps = []
    for i in range(x.shape[0]):
        first_step = None
        minimum_gap = math.inf
        for j in range(steps):
            shape = placed(meta["padded_footprint"], (float(x[i, j]), float(y[i, j]), float(yaw[i, j])))
            gap = polygon_distance(shape, box(model, row, age + (j + 1) * dt).polygon())
            minimum_gap = min(minimum_gap, gap)
            if gap <= 1e-9:
                overlaps[j] += 1
                if first_step is None:
                    first_step = j + 1
        first.append(first_step)
        minimum_gaps.append(minimum_gap)
    _, truth_records = analyze.analyze(
        cycle_path, trial / "profile.yaml",
        trial / "gazebo_poses.jsonl")
    if len(truth_records) != len(first):
        raise ValueError("truth rollout count differs from captured batch")
    truth_safe = [record["truth_dynamic_safe"] and
                  not record["costcritic_collision"]
                  for record in truth_records]
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    truth_times = [entry["t"] for entry in truth]
    body_shape = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    actual_box = analyze.obstacle_polygon()
    short_truth_safe = []
    for i in range(x.shape[0]):
        true_gap = min(polygon_distance(
            placed(body_shape, (float(x[i, j]), float(y[i, j]), float(yaw[i, j]))),
            placed(actual_box, interpolated_pose(
                truth, truth_times, prediction["consumer_sim_s"] + (j + 1) * dt)))
            for j in range(steps))
        short_truth_safe.append(true_gap >= .05)
    first_truth_pose = interpolated_pose(
        truth, truth_times, prediction["consumer_sim_s"] + dt)
    first_candidate = box(model, row, age + dt)
    first_pose = (float(x[0, 0]), float(y[0, 0]), float(yaw[0, 0]))
    original_no_conflict = [record["first_predicted_conflict_s"] is None
                            for record in truth_records]
    near_goal_position = ([math.hypot(float(x[i, -1]) - goal[0],
                                      float(y[i, -1]) - goal[1]) <= .15
                           for i in range(x.shape[0])] if goal else None)
    output = {"cycle_id": cycle_id, "source_t": row["source_t"],
            "cycle_sha256": {
                "metadata": digest(cycle_path),
                "arrays": digest(cycle_path.with_suffix(".bin"))},
            "history_count": row["multi_history_count"],
            "set_vertices_xy": [len(p) for p in row["multi_polygons"]],
            "first_step_candidate_box_xyxy_m": [
                first_candidate.min_x, first_candidate.min_y,
                first_candidate.max_x, first_candidate.max_y],
            "first_step_actual_center_xy_m": first_truth_pose[:2],
            "first_step_candidate_padded_gap_m": polygon_distance(
                placed(meta["padded_footprint"], first_pose),
                first_candidate.polygon()),
            "first_step_truth_body_gap_m": polygon_distance(
                placed(body_shape, first_pose),
                placed(actual_box, first_truth_pose)),
            "per_step_overlap_counts": overlaps,
            "rollouts_without_conflict": first.count(None),
            "rollouts_with_predicted_gap_at_least_0_02m": sum(
                gap >= .02 for gap in minimum_gaps),
            "truth_safe_within_prediction_horizon": sum(short_truth_safe),
            "truth_safe_within_horizon_and_predicted_gap_at_least_0_02m": sum(
                safe and gap >= .02 for safe, gap in zip(short_truth_safe, minimum_gaps)),
            "truth_unsafe_within_horizon_and_predicted_gap_at_least_0_02m": sum(
                not safe and gap >= .02 for safe, gap in zip(short_truth_safe, minimum_gaps)),
            "full_3s_truth_safe_rollouts": sum(truth_safe),
            "original_rollouts_without_conflict": sum(original_no_conflict),
            "original_full_3s_truth_safe_without_predicted_conflict": sum(
                safe and clear for safe, clear in zip(truth_safe, original_no_conflict)),
            "full_3s_truth_safe_without_predicted_conflict": sum(
                safe and conflict is None for safe, conflict in zip(truth_safe, first)),
            "full_3s_truth_safe_with_predicted_gap_at_least_0_02m": sum(
                safe and gap >= .02 for safe, gap in zip(truth_safe, minimum_gaps)),
            "full_3s_truth_unsafe_with_predicted_gap_at_least_0_02m": sum(
                not safe and gap >= .02 for safe, gap in zip(truth_safe, minimum_gaps)),
            "full_3s_truth_unsafe_with_predicted_conflict": sum(
                not safe and conflict is not None for safe, conflict in zip(truth_safe, first)),
            "first_conflict_step_histogram": {
                str(j): first.count(j) for j in range(1, steps + 1) if j in first}}
    if goal:
        output.update({
            "goal_position_rollouts": sum(near_goal_position),
            "original_goal_position_without_predicted_conflict": sum(
                near_goal and clear for near_goal, clear in zip(near_goal_position, original_no_conflict)),
            "goal_position_without_predicted_conflict": sum(
                near_goal and conflict is None
                for near_goal, conflict in zip(near_goal_position, first)),
            "goal_position_full_3s_truth_safe_without_predicted_conflict": sum(
                near_goal and safe and conflict is None
                for near_goal, safe, conflict in zip(near_goal_position, truth_safe, first)),
            "goal_padded_gap_by_step_m": [
                polygon_distance(goal_shape, box(model, row, age + (j + 1) * dt).polygon())
                for j in range(steps)]})
    return output


def run(historical_root, phase_trial, collision_trial=None):
    occupancy, _ = static_map()
    trials = []
    for path in sorted(historical_root.rglob("predictions.jsonl")):
        trial = path.parent
        if not path.stat().st_size or not (trial / "profile.yaml").exists() or not (trial / "observation/scans.jsonl").exists():
            continue
        profile = yaml.safe_load((trial / "profile.yaml").read_text())
        if "PredictionV1Critic" not in profile["controller_server"]["ros__parameters"]["FollowPath"]:
            continue
        audit, rows = one_trial(trial, recorded_messages(trial), occupancy,
                                return_source_rows=True)
        trials.append((str(trial), rows,
                       rows_from_transport(trial / "gazebo_poses.jsonl"),
                       audit["source_sha256"]))
    model = fit_source_error(trials)
    historical = [{"trial": name, "source_sha256": sha,
                   "evaluation": evaluate(model, multi_rows(model, rows), truth)}
                  for name, rows, truth, sha in trials]
    heldout = []
    for i, (name, rows, truth, sha) in enumerate(trials):
        fitted = fit_source_error([item for j, item in enumerate(trials) if i != j])
        heldout.append({"trial": name, "source_sha256": sha,
                        "model": fitted,
                        "evaluation": evaluate(fitted, multi_rows(fitted, rows), truth)})
    phase_audit, rows = one_trial(phase_trial, training_messages(phase_trial),
                                  occupancy, return_source_rows=True, pose_source="recorded_odom")
    sources = multi_rows(model, rows)
    result = {"schema": "rm_multi_scan_set_membership_bound/v1",
            "scope": "Offline conditional bound; empirical source error and ideal fixture acceleration are not certified. Historical scan projection uses Gazebo robot pose; phase4 uses recorded simulation odometry.",
            "model": model, "historical": historical, "historical_leave_one_trial_out": heldout,
            "phase4": {"source_sha256": phase_audit["source_sha256"],
                       "source_anchored": evaluate(model, sources, rows_from_transport(phase_trial / "gazebo_poses.jsonl")),
                       "actual_consumer": consumed_coverage(model, phase_trial, sources),
                       "key_cycle": key_cycle(model, phase_trial, sources, 263)}}
    if collision_trial is not None:
        collision_audit, collision_rows = one_trial(
            collision_trial, training_messages(collision_trial), occupancy,
            return_source_rows=True, pose_source="recorded_odom")
        collision_sources = multi_rows(model, collision_rows)
        result["collision_trial"] = {
            "source_sha256": collision_audit["source_sha256"],
            "source_anchored": evaluate(model, collision_sources,
                                        rows_from_transport(collision_trial / "gazebo_poses.jsonl")),
            "actual_consumer": consumed_coverage(model, collision_trial,
                                                  collision_sources),
            "key_cycle": key_cycle(model, collision_trial, collision_sources, 162)}
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--historical-root", type=Path, required=True)
    parser.add_argument("--phase-trial", type=Path, required=True)
    parser.add_argument("--collision-trial", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.historical_root, args.phase_trial,
                 args.collision_trial)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, sort_keys=True, indent=2) + "\n")
    print(json.dumps({"model": result["model"],
                      "phase4_key_cycle": result["phase4"]["key_cycle"],
                      "collision_key_cycle": result.get("collision_trial", {}).get("key_cycle")},
                     indent=2))


if __name__ == "__main__":
    main()
