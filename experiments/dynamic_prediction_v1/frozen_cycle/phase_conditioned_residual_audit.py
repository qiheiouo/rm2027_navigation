#!/usr/bin/env python3
"""Audit held-out source-time residual against a fixed conditional cloud."""
import argparse
import json
from pathlib import Path

import numpy as np

import analyze
from batch_sampling_probe import digest
from reflected_residual_rank_probe import historical_clouds
from raw_scan_support_audit import static_map


def midrank(values, target):
    return float((np.count_nonzero(values < target) +
                  .5 * np.count_nonzero(values == target)) / len(values))


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    score = json.loads(args.score_evidence.read_text())
    if score["phase_band_y_m_and_vy_mps"] is None:
        raise ValueError("requires a fixed phase-conditioned score probe")
    target = next(case for case in score["cases"]
                  if case["name"] == "intrusion_162")
    cycle = args.trial / "mppi_cycles/cycle_162.json"
    truth_path = args.trial / "gazebo_poses.jsonl"
    meta, _ = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    prediction = analyze.event(meta, "prediction.input")
    if meta["cycle_id"] != 162 or settings["steps"] != 30:
        raise ValueError("frozen cycle changed")
    source = prediction["source_stamp_ns"] / 1e9
    if abs(source - target["scan_source_t"]) > 1e-6:
        raise ValueError("source time differs from score evidence")
    durations = tuple(prediction["source_age_s"] +
                      (step + 1) * settings["dt"] for step in range(9))
    condition = (target["scan_center_xy"][1],
                 target["scan_velocity_xy"][1],
                 target["observer_view_y"],
                 score["phase_band_y_m_and_vy_mps"])
    occupancy, _ = static_map()
    training_paths = [Path(path) for path in score["training_trials"]]
    clouds, counts = historical_clouds(training_paths, occupancy, durations,
                                       score["training_message_source"],
                                       condition)
    if counts != target["historical_cloud_counts_by_trial_and_step"]:
        raise ValueError("conditional training support changed")
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    center = np.asarray(target["scan_center_xy"])
    velocity = np.asarray(target["scan_velocity_xy"])
    sign = -1. if target["observer_view_y"] == "north" else 1.
    steps = []
    for index, duration in enumerate(durations):
        actual = np.asarray(analyze.interpolated_pose(
            truth, times, source + duration)[:2])
        residual = actual - center - velocity * duration
        residual[1] *= sign
        per_trial = []
        for path, trial in zip(training_paths, clouds):
            row = trial[index]
            per_trial.append({"trial": str(path), "n": len(row),
                              "x_min_m": float(row[:, 0].min()),
                              "x_max_m": float(row[:, 0].max()),
                              "y_min_m": float(row[:, 1].min()),
                              "y_max_m": float(row[:, 1].max()),
                              "x_midrank": midrank(row[:, 0], residual[0]),
                              "y_midrank": midrank(row[:, 1], residual[1])})
        upper_y = max(row["y_max_m"] for row in per_trial)
        steps.append({"step": index + 1, "source_horizon_s": duration,
                      "canonical_true_residual_xy_m": residual.tolist(),
                      "equal_trial_x_midrank": float(np.mean(
                          [row["x_midrank"] for row in per_trial])),
                      "equal_trial_y_midrank": float(np.mean(
                          [row["y_midrank"] for row in per_trial])),
                      "above_all_training_y_max_m": max(
                          0., float(residual[1]) - upper_y),
                      "per_trial": per_trial})
    report = {
        "schema": "rm_dynamic_prediction/phase_conditioned_residual_audit/v1",
        "scope": "One preselected held-out collision-trial source at frozen cycle 162. Conditional historical clouds are rebuilt from the already fixed two non-target Navfn trials and causal source-state selection. The target future Gazebo box center is used only here as a residual label; no scoring parameter is fitted. North-view y residual is sign-reflected into the training canonical frame. Source samples within a trial are correlated; midranks are descriptive only.",
        "source_sha256": {"score_evidence": digest(args.score_evidence),
                          "cycle": digest(cycle),
                          "truth": digest(truth_path)},
        "source_t": source,
        "observer_view_y": target["observer_view_y"],
        "training_sample_counts_by_trial_and_step": counts,
        "steps": steps}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"steps": len(steps),
                      "y_above_all_training_steps": sum(
                          row["above_all_training_y_max_m"] > 0
                          for row in steps),
                      "output": str(args.output)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--score-evidence", type=Path, required=True)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args())
