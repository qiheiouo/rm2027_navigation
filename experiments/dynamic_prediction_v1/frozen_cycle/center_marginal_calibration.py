#!/usr/bin/env python3
"""Audit an uncalibrated polygon-center prior on six held-out scan trials.

Each trial uses the source-error bound fitted from the other five trials.
Gazebo future centers are evaluation labels only and never alter the model.
Samples from a single trial are correlated; no confidence level is claimed.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from analyze import interpolated_pose, rows_from_transport
from heldout_geometry_audit import recorded_messages
import multi_scan_bound_probe as multi
from polygon_marginal_probe import area
from raw_scan_support_audit import one_trial, static_map
import short_horizon_filter_probe as short


HORIZONS_S = (.1, .3, .6, .9)


def bucket(history_count):
    if history_count == 2:
        return "2"
    if history_count <= 5:
        return "3-5"
    if history_count <= 9:
        return "6-9"
    return "10+"


def polygon_cdf(polygon, horizon, position):
    total = area(polygon)
    if total <= 1e-12:
        raise ValueError("empty conditional feasible polygon")
    return min(1., max(0., area(multi.clip_polygon(
        polygon, 1., horizon, position)) / total))


def accelerated_cdf(polygon, horizon, position, acceleration):
    # Uniform acceleration displacement over the existing ±A*h²/2 bound.
    # Five-point Gaussian quadrature integrates that *assumption*, not truth.
    growth = .5 * acceleration * horizon * horizon
    nodes, weights = np.polynomial.legendre.leggauss(5)
    return float(sum(weight * polygon_cdf(
        polygon, horizon, position - node * growth)
        for node, weight in zip(nodes, weights)) / 2)


def describe(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        "n": len(ordered),
        "mean": float(np.mean(ordered)),
        "p10": float(np.quantile(ordered, .1)),
        "p50": float(np.quantile(ordered, .5)),
        "p90": float(np.quantile(ordered, .9)),
        "below_0_1_fraction": sum(value < .1 for value in ordered) /
                                len(ordered),
        "above_0_9_fraction": sum(value > .9 for value in ordered) /
                                len(ordered),
        "outer_decile_fraction": sum(value < .1 or value > .9
                                      for value in ordered) / len(ordered),
        "max_empirical_cdf_gap_to_uniform": max(
            max(abs((index + 1) / len(ordered) - value),
                abs(index / len(ordered) - value))
            for index, value in enumerate(ordered)),
    }


def trial_pits(trial, model, occupancy):
    audit, rows = one_trial(trial, recorded_messages(trial), occupancy,
                            return_source_rows=True)
    sources = multi.multi_rows(model, rows)
    truth = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in truth]
    values = {}
    source_counts = {}
    for row in sources:
        history = row["multi_history_count"]
        group = bucket(history)
        source_counts[group] = source_counts.get(group, 0) + 1
        for horizon in HORIZONS_S:
            if row["source_t"] + horizon >= times[-1]:
                continue
            actual = interpolated_pose(
                truth, times, row["source_t"] + horizon)
            for axis, name in ((0, "x"), (1, "y")):
                pit = accelerated_cdf(
                    row["multi_polygons"][axis], horizon,
                    actual[axis], model["stipulated_acceleration_mps2"])
                key = f"{group}/{horizon:.1f}s/{name}"
                values.setdefault(key, []).append(pit)
    return audit, source_counts, values


def run(model_evidence):
    record = json.loads(model_evidence.read_text())
    heldout = record["historical_leave_one_trial_out"]
    if len(heldout) != 6:
        raise ValueError("expected six documented leave-one-out trials")
    occupancy, _ = static_map()
    trial_results = []
    combined = {}
    for item in heldout:
        trial = Path(item["trial"])
        audit, source_counts, values = trial_pits(
            trial, item["model"], occupancy)
        if audit["source_sha256"] != item["source_sha256"]:
            raise ValueError(f"source digest differs: {trial}")
        for key, sequence in values.items():
            combined.setdefault(key, []).extend(sequence)
        trial_results.append({
            "trial": str(trial),
            "source_sha256": item["source_sha256"],
            "model": item["model"],
            "source_counts_by_history": source_counts,
            "pit_summary": {key: describe(sequence)
                            for key, sequence in sorted(values.items())},
        })
    return {
        "schema": "rm_dynamic_prediction_center_marginal_calibration/v1",
        "scope": "Descriptive leave-one-trial-out probability-integral-transform diagnostic. Uniform feasible polygon and acceleration displacement are hypotheses, not calibrated online probabilities. Highly correlated samples; no statistical confidence interval.",
        "model_evidence_sha256": short.digest(model_evidence),
        "horizons_s": HORIZONS_S,
        "historical_trials": trial_results,
        "combined": {key: describe(sequence)
                     for key, sequence in sorted(combined.items())},
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.model_evidence)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"trials": len(result["historical_trials"]),
                      "output": str(args.output)}, sort_keys=True))
