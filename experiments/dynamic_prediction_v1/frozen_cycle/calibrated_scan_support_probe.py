#!/usr/bin/env python3
"""Historical-fit scan-center support, evaluated on a separate frozen trial.

This is an offline empirical-feasibility test, not a runtime safety bound.
Only old trial truth fits the envelope; phase-4 truth labels its heldout errors.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
from analyze import (critic_deltas, event, integrate_omni,
                     interpolated_pose, last, placed, polygon_distance,
                     read_cycle, rows_from_transport)
from envelope import AxisBox, predicted_box
from heldout_geometry_audit import digest, recorded_messages
import replay_ranking
from raw_scan_support_audit import (EXTENT, REFERENCE_ACCELERATION,
                                    near_full_span, one_trial, static_map,
                                    training_messages)


def eligible(row):
    return near_full_span(row) and "scan_center_velocity" in row


def nominal(row, dt):
    return tuple(row["raw_bbox_mid"][axis] +
                 row["scan_center_velocity"][axis] * dt
                 for axis in (0, 1))


def samples(rows, truth):
    times = [entry["t"] for entry in truth]
    for row in rows:
        if not eligible(row):
            continue
        for step in range(11):
            dt = step * .1
            t = row["source_t"] + dt
            if t >= times[-1]:
                break
            actual = interpolated_pose(truth, times, t)
            center = nominal(row, dt)
            yield row, step, tuple(actual[axis] - center[axis]
                                   for axis in (0, 1))


def fit(records):
    at_source = [residual for _, step, residual in records if step == 0]
    if len(at_source) < 30:
        raise ValueError("insufficient independent source observations")
    # The extra 0.03 m is an exploratory source-error reserve. The simulated
    # Gaussian scan noise has no deterministic finite bound.
    source_error = [max(abs(error[axis]) for error in at_source) + .03
                    for axis in (0, 1)]
    velocity_error = [max(0., max((
        (abs(residual[axis]) - source_error[axis] -
         .5 * REFERENCE_ACCELERATION * (step * .1) ** 2) / (step * .1)
        for _, step, residual in records if step > 0), default=0.))
        for axis in (0, 1)]
    return {"source_error_xy_m": source_error,
            "velocity_error_xy_mps": velocity_error,
            "reference_acceleration_mps2": REFERENCE_ACCELERATION,
            "training_sources": len(at_source),
            "training_samples": len(records)}


def allowance(model, axis, dt):
    return (model["source_error_xy_m"][axis] +
            model["velocity_error_xy_mps"][axis] * dt +
            .5 * model["reference_acceleration_mps2"] * dt * dt)


def evaluate(model, records):
    missed = []
    uncovered_source_ids = set()
    for row, step, residual in records:
        dt = step * .1
        deficit = max(0., *(abs(residual[axis]) - allowance(model, axis, dt)
                           for axis in (0, 1)))
        if deficit > 1e-9:
            uncovered_source_ids.add(id(row))
            missed.append({"source_t": row["source_t"], "step": step,
                           "side": row["side"], "residual_xy_m": residual,
                           "uncovered_edge_m": deficit})
    return {"sources": sum(step == 0 for _, step, _ in records),
            "future_samples": len(records),
            "uncovered_samples": len(missed),
            "uncovered_sources": len(uncovered_source_ids),
            "maximum_uncovered_edge_m": max(
                (row["uncovered_edge_m"] for row in missed), default=0.),
            "first_miss": missed[0] if missed else None,
            "worst_miss": max(missed, key=lambda row: row["uncovered_edge_m"])
            if missed else None}


def box(model, row, dt):
    center = nominal(row, dt)
    return AxisBox(center[0] - allowance(model, 0, dt) - EXTENT[0] / 2,
                   center[1] - allowance(model, 1, dt) - EXTENT[1] / 2,
                   center[0] + allowance(model, 0, dt) + EXTENT[0] / 2,
                   center[1] + allowance(model, 1, dt) + EXTENT[1] / 2)


def consumed_coverage(trial, phase_rows, models):
    by_source = {round(row["source_t"], 6): row for row in phase_rows
                 if eligible(row)}
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    truth_times = [entry["t"] for entry in truth]
    outputs = {label: {"cycles": 0, "samples": 0,
                       "uncovered_samples": 0, "uncovered_cycles": set(),
                       "maximum_uncovered_edge_m": 0., "first_miss": None}
               for label in models}
    for path in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        values = [event_row["value"] for event_row in meta["events"]
                  if event_row["kind"] == "prediction.input"]
        if len(values) != 1 or values[0].get("status") != "accepted":
            continue
        prediction = values[0]
        row = by_source.get(round(prediction["source_stamp_ns"] / 1e9, 6))
        if row is None:
            continue
        settings = event(meta, "settings")
        step_dt = float(settings["dt"])
        steps = min(30, int(1. / step_dt + 1e-8))
        for label, model in models.items():
            if label == "east" and row["side"] != "east":
                continue
            result = outputs[label]
            result["cycles"] += 1
            for step in range(1, steps + 1):
                duration = prediction["source_age_s"] + step * step_dt
                actual_t = prediction["consumer_sim_s"] + step * step_dt
                if actual_t >= truth_times[-1]:
                    break
                actual = interpolated_pose(truth, truth_times, actual_t)
                candidate = box(model, row, duration)
                deficit = max(0.,
                              candidate.min_x - (actual[0] - EXTENT[0] / 2),
                              (actual[0] + EXTENT[0] / 2) - candidate.max_x,
                              candidate.min_y - (actual[1] - EXTENT[1] / 2),
                              (actual[1] + EXTENT[1] / 2) - candidate.max_y)
                result["samples"] += 1
                if deficit > 1e-9:
                    result["uncovered_samples"] += 1
                    result["uncovered_cycles"].add(meta["cycle_id"])
                    result["maximum_uncovered_edge_m"] = max(
                        result["maximum_uncovered_edge_m"], deficit)
                    if result["first_miss"] is None:
                        result["first_miss"] = {
                            "cycle_id": meta["cycle_id"], "step": step,
                            "source_t": row["source_t"],
                            "future_t": actual_t,
                            "uncovered_edge_m": deficit}
    for result in outputs.values():
        result["uncovered_cycles"] = len(result["uncovered_cycles"])
    return outputs


def key_cycle(trial, phase_rows, models):
    meta, arrays = read_cycle(trial / "mppi_cycles/cycle_263.json")
    prediction = event(meta, "prediction.input")
    settings = event(meta, "settings")
    track = next(track for track in prediction["tracks"] if track["state"] == 2)
    source_t = prediction["source_stamp_ns"] / 1e9
    row = next(row for row in phase_rows if row["source_t"] == source_t)
    if not eligible(row) or row["side"] != "east":
        raise ValueError("key cycle does not meet the observed east-side condition")
    profile = yaml.safe_load((trial / "profile.yaml").read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    goal = tuple(json.loads((trial / "runtime_audit.json").read_text())[
        "navigation_result"]["goal"])
    goal_shape = placed(meta["padded_footprint"], goal)
    dt = float(settings["dt"])
    age = float(prediction["source_age_s"])
    original = predicted_box(track["xy"], track["vxy"], track["size_xy"],
                             EXTENT, age, dt,
                             float(params["reference_acceleration"]))
    x, y, yaw = (last(arrays, f"rollout.{axis}")
                 for axis in ("x", "y", "yaw"))
    steps = min(int(params["horizon"] / dt + 1e-8), x.shape[1])
    variants = {}
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    truth_times = [entry["t"] for entry in truth]
    _, rollout_records = analyze.analyze(
        trial / "mppi_cycles/cycle_263.json", trial / "profile.yaml",
        trial / "gazebo_poses.jsonl")
    truth_safe = np.asarray([
        record["truth_dynamic_safe"] and not record["costcritic_collision"]
        for record in rollout_records], dtype=bool)
    original_weights = last(arrays, "weighted.probability")
    original_costs = last(arrays, "weighted.costs")
    terms, _ = critic_deltas(arrays)
    old_prediction_term = terms["FollowPath.PredictionV1Critic"].astype(np.float32)
    history = replay_ranking.history_from_trial(
        trial / "mppi_cycles", meta["cycle_id"], arrays)
    baseline_control = replay_ranking.aggregate(
        arrays, settings, history, original_weights)
    replay_error = max(float(np.max(np.abs(
        baseline_control[axis] - last(arrays, f"after_filter.{axis}"))))
        for axis in ("vx", "vy", "wz"))
    if replay_error > 1e-5:
        raise ValueError("baseline MPPI filter did not replay")
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    physical_box = [(-EXTENT[0] / 2, -EXTENT[1] / 2),
                    (EXTENT[0] / 2, -EXTENT[1] / 2),
                    (EXTENT[0] / 2, EXTENT[1] / 2),
                    (-EXTENT[0] / 2, EXTENT[1] / 2)]

    def control_metrics(control, weights):
        trajectory = integrate_omni(
            *(control[axis] for axis in ("vx", "vy", "wz")),
            meta["pose"], dt)
        body_gap, _ = replay_ranking.open_loop_gap(
            control, meta, settings, truth, truth_times, body,
            physical_box, prediction["consumer_sim_s"])
        goal_rollouts = np.hypot(x[:, -1] - goal[0],
                                 y[:, -1] - goal[1]) <= .15
        return {"returned_control": [float(control[axis][settings["offset"]])
                                     for axis in ("vx", "vy", "wz")],
                "truth_safe_rollout_probability_mass": float(
                    np.sum(weights[truth_safe])),
                "goal_position_rollout_probability_mass": float(
                    np.sum(weights[goal_rollouts])),
                "open_loop_endpoint_goal_distance_m": math.hypot(
                    float(trajectory[0][-1]) - goal[0],
                    float(trajectory[1][-1]) - goal[1]),
                "open_loop_truth_body_min_gap_m": body_gap}

    for label, model in models.items():
        first = box(model, row, age + dt)
        gaps = []
        near_counts = []
        for rollout in range(x.shape[0]):
            minimum_gap = math.inf
            near_count = 0
            for step in range(steps):
                predicted = box(model, row, age + (step + 1) * dt)
                footprint = placed(meta["padded_footprint"],
                                   (float(x[rollout, step]),
                                    float(y[rollout, step]),
                                    float(yaw[rollout, step])))
                gap = polygon_distance(footprint, predicted.polygon())
                minimum_gap = min(minimum_gap, gap)
                near_count += 1e-9 < gap < .02
            gaps.append(minimum_gap)
            near_counts.append(near_count)
        future_coverage_misses = []
        for step in range(steps):
            duration = age + (step + 1) * dt
            future_t = source_t + duration
            actual = interpolated_pose(truth, truth_times, future_t)
            candidate = box(model, row, duration)
            if not candidate.contains(actual, EXTENT):
                future_coverage_misses.append(step + 1)
        if min(gaps) <= 1e-9:
            raise ValueError("candidate has collisions; replay needs hard rank term")
        new_term = np.asarray(near_counts, dtype=np.float32) * np.float32(
            (3.81 / 254.) * 300. / steps)
        new_weights = replay_ranking.probabilities(
            original_costs.astype(np.float32) - old_prediction_term + new_term,
            settings["temperature"])
        new_control = replay_ranking.aggregate(
            arrays, settings, history, new_weights)
        variants[label] = {
            "first_step_goal_padded_gap_m": polygon_distance(
                goal_shape, first.polygon()),
            "rollouts_with_predicted_gap_at_least_0_02m": sum(
                gap >= .02 for gap in gaps),
            "rollouts_with_predicted_overlap": sum(
                gap <= 1e-9 for gap in gaps),
            "rollouts_with_near_penalty": sum(count > 0 for count in near_counts),
            "candidate_prediction_score_max": float(np.max(new_term)),
            "actual_box_uncovered_steps": future_coverage_misses,
            "fixed_rollout_replay": control_metrics(new_control, new_weights),
        }
    return {"cycle_id": meta["cycle_id"], "source_t": source_t,
            "cycle_json_sha256": digest(trial / "mppi_cycles/cycle_263.json"),
            "source_age_s": age, "raw_span_xy_m": row["raw_span"],
            "original_first_step_goal_padded_gap_m": polygon_distance(
                goal_shape, original.polygon()),
            "baseline_replay_max_control_error": replay_error,
            "baseline_fixed_rollout_replay": control_metrics(
                baseline_control, original_weights),
            "variants": variants}


def run(historical_root, phase_trial, phase_pose_source="gazebo"):
    occupancy, _ = static_map()
    historical = []
    historical_records = []
    historical_by_trial = []
    for path in sorted(historical_root.rglob("predictions.jsonl")):
        trial = path.parent
        if not path.stat().st_size or not (trial / "profile.yaml").exists():
            continue
        if not (trial / "observation/scans.jsonl").exists():
            continue
        profile = yaml.safe_load((trial / "profile.yaml").read_text())
        if "PredictionV1Critic" not in profile["controller_server"][
                "ros__parameters"]["FollowPath"]:
            continue
        audit, rows = one_trial(trial, recorded_messages(trial), occupancy,
                                return_source_rows=True)
        truth = rows_from_transport(trial / "gazebo_poses.jsonl")
        records = list(samples(rows, truth))
        historical.append({"trial": str(trial), "record_count": len(records),
                           "source_sha256": audit["source_sha256"]})
        historical_records.extend(records)
        historical_by_trial.append((str(trial), records))
    global_model = fit(historical_records)
    east_model = fit([record for record in historical_records
                      if record[0]["side"] == "east"])
    models = {"global": global_model, "east": east_model}
    phase_audit, phase_rows = one_trial(phase_trial,
                                        training_messages(phase_trial),
                                        occupancy, return_source_rows=True,
                                        pose_source=phase_pose_source)
    phase_truth = rows_from_transport(phase_trial / "gazebo_poses.jsonl")
    phase_records = list(samples(phase_rows, phase_truth))
    evaluations = {
        "global": {"historical_fit": evaluate(global_model, historical_records),
                   "phase4_heldout": evaluate(global_model, phase_records)},
        "east": {"historical_fit": evaluate(east_model, [
                    item for item in historical_records if item[0]["side"] == "east"]),
                 "phase4_heldout": evaluate(east_model, [
                    item for item in phase_records if item[0]["side"] == "east"])}
    }
    leave_one_trial_out = []
    for index, (name, test_records) in enumerate(historical_by_trial):
        fit_records = [item for other_index, (_, records) in
                       enumerate(historical_by_trial) if other_index != index
                       for item in records]
        model = fit(fit_records)
        leave_one_trial_out.append({"trial": name, "model": model,
                                    "heldout": evaluate(model, test_records)})
    return {"schema": "rm_historical_scan_support_phase4_holdout/v1",
            "scope": "Old trials only fit empirical center/velocity error envelopes using Gazebo robot pose. Phase-4 trial is held out and its scan pose source is selected explicitly. Gazebo obstacle truth only validates future coverage. Not a runtime safety bound.",
            "historical": historical,
            "phase4": {"trial": str(phase_trial),
                       "scan_pose_source": phase_pose_source,
                       "pose_source_vs_gazebo": phase_audit[
                           "pose_source_vs_gazebo"],
                       "source_sha256": phase_audit["source_sha256"]},
            "models": models, "evaluations": evaluations,
            "historical_leave_one_trial_out": leave_one_trial_out,
            "phase4_actual_consumer_coverage": consumed_coverage(
                phase_trial, phase_rows, models),
            "key_cycle": key_cycle(phase_trial, phase_rows, models)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--historical-root", type=Path, required=True)
    parser.add_argument("--phase-trial", type=Path, required=True)
    parser.add_argument("--phase-pose-source", choices=("gazebo", "recorded_odom"),
                        default="gazebo")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.historical_root, args.phase_trial,
                 args.phase_pose_source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"models": result["models"],
                      "evaluations": result["evaluations"],
                      "phase4_actual_consumer_coverage": result[
                          "phase4_actual_consumer_coverage"],
                      "key_cycle": result["key_cycle"]}, indent=2))


if __name__ == "__main__":
    main()
