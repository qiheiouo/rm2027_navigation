#!/usr/bin/env python3
"""Conditional scan-center motion bound using only past observations.

The position error is calibrated from historical source-time truth. Future
motion uses a stipulated fixture acceleration cap, never future truth for fit.
This is an offline diagnostic and is not a runtime safety guarantee.
"""
import argparse
import json
import math
from pathlib import Path

import yaml

from analyze import (event, interpolated_pose, last, placed,
                     polygon_distance, read_cycle, rows_from_transport)
from envelope import AxisBox, predicted_box
from heldout_geometry_audit import recorded_messages
from raw_scan_support_audit import (EXTENT, REFERENCE_ACCELERATION,
                                    near_full_span, one_trial, static_map,
                                    training_messages)


def secant_rows(rows):
    recent = []
    result = []
    for row in sorted(rows, key=lambda item: item["source_t"]):
        if not near_full_span(row):
            continue
        recent = [prior for prior in recent if
                  row["source_t"] - prior["source_t"] <= .4]
        prior = next((item for item in recent if
                      row["source_t"] - item["source_t"] >= .2), None)
        if prior is not None:
            delta = row["source_t"] - prior["source_t"]
            row["secant_delta_s"] = delta
            row["secant_velocity"] = tuple(
                (row["raw_bbox_mid"][axis] -
                 prior["raw_bbox_mid"][axis]) / delta for axis in (0, 1))
            result.append(row)
        recent.append(row)
    return result


def fit_source_error(trials):
    rows = [row for trial in trials for row in trial[1]
            if near_full_span(row)]
    if len(rows) < 30:
        raise ValueError("insufficient source-time measurements")
    return {"source_error_xy_m": [
        max(abs(row["actual_center"][axis] -
                row["raw_bbox_mid"][axis]) for row in rows) + .03
        for axis in (0, 1)],
        "stipulated_acceleration_mps2": REFERENCE_ACCELERATION,
        "source_fit_count": len(rows),
        "source_extra_reserve_m": .03}


def box(model, row, horizon):
    center = [row["raw_bbox_mid"][axis] +
              row["secant_velocity"][axis] * horizon for axis in (0, 1)]
    bound = center_half_width(model, row, horizon)
    return AxisBox(center[0] - bound[0] - EXTENT[0] / 2,
                   center[1] - bound[1] - EXTENT[1] / 2,
                   center[0] + bound[0] + EXTENT[0] / 2,
                   center[1] + bound[1] + EXTENT[1] / 2)


def center_half_width(model, row, horizon):
    delta = row["secant_delta_s"]
    return [model["source_error_xy_m"][axis] +
            (2 * model["source_error_xy_m"][axis] / delta +
             .5 * model["stipulated_acceleration_mps2"] * delta) * horizon +
            .5 * model["stipulated_acceleration_mps2"] * horizon * horizon
            for axis in (0, 1)]


def deficit(candidate, actual):
    return max(0., candidate.min_x - (actual[0] - EXTENT[0] / 2),
               (actual[0] + EXTENT[0] / 2) - candidate.max_x,
               candidate.min_y - (actual[1] - EXTENT[1] / 2),
               (actual[1] + EXTENT[1] / 2) - candidate.max_y)


def evaluate(model, sources, truth):
    times = [row["t"] for row in truth]
    count = misses = 0
    missed_sources = set()
    worst = None
    for row in sources:
        for step in range(11):
            duration = step * .1
            future_t = row["source_t"] + duration
            if future_t >= times[-1]:
                break
            actual = interpolated_pose(truth, times, future_t)
            miss = deficit(box(model, row, duration), actual)
            count += 1
            if miss > 1e-9:
                misses += 1
                missed_sources.add(id(row))
                if worst is None or miss > worst["uncovered_edge_m"]:
                    worst = {"source_t": row["source_t"], "step": step,
                             "uncovered_edge_m": miss}
    return {"sources": len(sources), "future_samples": count,
            "uncovered_samples": misses,
            "uncovered_sources": len(missed_sources),
            "worst_miss": worst}


def consumed_coverage(model, trial, sources):
    by_source = {round(row["source_t"], 6): row for row in sources}
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in truth]
    result = {"cycles": 0, "samples": 0, "uncovered_samples": 0,
              "uncovered_cycles": set(), "worst_miss": None}
    for path in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        candidates = [entry["value"] for entry in meta["events"]
                      if entry["kind"] == "prediction.input"]
        if len(candidates) != 1 or candidates[0].get("status") != "accepted":
            continue
        prediction = candidates[0]
        row = by_source.get(round(prediction["source_stamp_ns"] / 1e9, 6))
        if row is None:
            continue
        settings = event(meta, "settings")
        dt = settings["dt"]
        steps = min(30, int(1. / dt + 1e-8))
        result["cycles"] += 1
        for step in range(1, steps + 1):
            duration = prediction["source_age_s"] + step * dt
            t = prediction["consumer_sim_s"] + step * dt
            if t >= times[-1]:
                break
            actual = interpolated_pose(truth, times, t)
            miss = deficit(box(model, row, duration), actual)
            result["samples"] += 1
            if miss > 1e-9:
                result["uncovered_samples"] += 1
                result["uncovered_cycles"].add(meta["cycle_id"])
                if (result["worst_miss"] is None or
                        miss > result["worst_miss"]["uncovered_edge_m"]):
                    result["worst_miss"] = {
                        "cycle_id": meta["cycle_id"], "step": step,
                        "source_t": row["source_t"],
                        "uncovered_edge_m": miss}
    result["uncovered_cycles"] = len(result["uncovered_cycles"])
    return result


def key_cycle(model, trial, sources):
    meta, arrays = read_cycle(trial / "mppi_cycles/cycle_263.json")
    prediction = event(meta, "prediction.input")
    settings = event(meta, "settings")
    source_t = prediction["source_stamp_ns"] / 1e9
    row = next(row for row in sources if row["source_t"] == source_t)
    profile = yaml.safe_load((trial / "profile.yaml").read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    track = next(track for track in prediction["tracks"] if track["state"] == 2)
    goal = tuple(json.loads((trial / "runtime_audit.json").read_text())[
        "navigation_result"]["goal"])
    goal_shape = placed(meta["padded_footprint"], goal)
    dt = settings["dt"]
    age = prediction["source_age_s"]
    original = predicted_box(track["xy"], track["vxy"], track["size_xy"],
                             EXTENT, age, dt,
                             float(params["reference_acceleration"]))
    first = box(model, row, age + dt)
    x, y, yaw = (last(arrays, f"rollout.{axis}")
                 for axis in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(1. / dt + 1e-8))
    min_gaps = []
    first_conflict_steps = []
    step_overlap_counts = [0] * steps
    for rollout in range(x.shape[0]):
        gap = math.inf
        first_conflict = None
        for step in range(steps):
            shape = placed(meta["padded_footprint"],
                           (float(x[rollout, step]),
                            float(y[rollout, step]),
                            float(yaw[rollout, step])))
            step_gap = polygon_distance(
                shape, box(model, row, age + (step + 1) * dt).polygon())
            gap = min(gap, step_gap)
            if step_gap <= 1e-9:
                step_overlap_counts[step] += 1
                if first_conflict is None:
                    first_conflict = step + 1
        min_gaps.append(gap)
        first_conflict_steps.append(first_conflict)
    first_conflict_histogram = {
        str(step): first_conflict_steps.count(step)
        for step in range(1, steps + 1)
        if step in first_conflict_steps}
    return {"cycle_id": 263, "source_t": source_t,
            "secant_delta_s": row["secant_delta_s"],
            "source_age_s": age,
            "secant_velocity_xy_mps": row["secant_velocity"],
            "center_half_width_first_step_xy_m": center_half_width(
                model, row, age + dt),
            "center_half_width_last_step_xy_m": center_half_width(
                model, row, age + steps * dt),
            "original_first_step_goal_padded_gap_m": polygon_distance(
                goal_shape, original.polygon()),
            "candidate_first_step_goal_padded_gap_m": polygon_distance(
                goal_shape, first.polygon()),
            "rollouts_with_predicted_overlap": sum(gap <= 1e-9 for gap in min_gaps),
            "rollouts_with_gap_at_least_0_02m": sum(gap >= .02 for gap in min_gaps),
            "minimum_rollout_predicted_gap_m": min(min_gaps),
            "per_step_overlap_counts": step_overlap_counts,
            "first_conflict_step_histogram": first_conflict_histogram,
            "rollouts_without_conflict": first_conflict_steps.count(None),
            "goal_padded_gap_by_step_m": [polygon_distance(
                goal_shape, box(model, row, age + (step + 1) * dt).polygon())
                for step in range(steps)]}


def run(historical_root, phase_trial):
    occupancy, _ = static_map()
    trials = []
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
        trials.append((str(trial), rows,
                       rows_from_transport(trial / "gazebo_poses.jsonl"),
                       audit["source_sha256"]))
    model = fit_source_error(trials)
    historical = [{"trial": name, "source_sha256": sha,
                   "fit": evaluate(model, secant_rows(rows), truth)}
                  for name, rows, truth, sha in trials]
    leave_out = []
    for index, (name, rows, truth, _) in enumerate(trials):
        other = [item[:3] for j, item in enumerate(trials) if j != index]
        fitted = fit_source_error(other)
        leave_out.append({"trial": name, "model": fitted,
                          "heldout": evaluate(fitted, secant_rows(rows), truth)})
    phase_audit, phase_rows = one_trial(phase_trial,
                                        training_messages(phase_trial),
                                        occupancy, return_source_rows=True,
                                        pose_source="recorded_odom")
    phase_sources = secant_rows(phase_rows)
    phase_truth = rows_from_transport(phase_trial / "gazebo_poses.jsonl")
    return {"schema": "rm_causal_scan_center_motion_bound/v1",
            "scope": "Historical source-time truth only calibrates center position error. Motion uncertainty follows a conditional acceleration and two-position bound; future truth is used solely for validation. No certified lidar error or real-object acceleration bound.",
            "model": model,
            "historical": historical,
            "historical_leave_one_trial_out": leave_out,
            "phase4": {"trial": str(phase_trial),
                       "source_sha256": phase_audit["source_sha256"],
                       "source_anchored": evaluate(model, phase_sources,
                                                   phase_truth),
                       "actual_consumer": consumed_coverage(
                           model, phase_trial, phase_sources),
                       "key_cycle": key_cycle(model, phase_trial, phase_sources)}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--historical-root", type=Path, required=True)
    parser.add_argument("--phase-trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.historical_root, args.phase_trial)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"model": result["model"],
                      "phase4": {key: result["phase4"][key] for key in
                                 ("source_anchored", "actual_consumer", "key_cycle")}},
                     indent=2))


if __name__ == "__main__":
    main()
