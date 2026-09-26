#!/usr/bin/env python3
"""Decompose captured MPPI ranking on the fixed pre-violation window.

Positive label: physical dynamic clearance gate and no original CostCritic
collision. Negative comparison group: dynamic-unsafe but CostCritic-clear.
This isolates the dynamic information that the existing static critic missed.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

import analyze


WINDOW = tuple(range(145, 165))
TERMS = ("FollowPath.CostCritic", "FollowPath.GoalCritic",
         "FollowPath.PathAlignCritic", "FollowPath.PredictionV1Critic")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rank_auc(costs, good, bad, tie_tolerance=1e-3):
    """Probability a good trajectory has lower score, with ties split."""
    if not np.any(good) or not np.any(bad):
        return None
    difference = (np.asarray(costs, dtype=np.float64)[good, None] -
                  np.asarray(costs, dtype=np.float64)[None, bad])
    return float(np.mean(difference < -tie_tolerance) +
                 .5 * np.mean(np.abs(difference) <= tie_tolerance))


def one_cycle(trial, cycle_id):
    cycle = trial / "mppi_cycles" / f"cycle_{cycle_id}.json"
    summary, records = analyze.analyze(
        cycle, trial / "profile.yaml", trial / "gazebo_poses.jsonl")
    _, arrays = analyze.read_cycle(cycle)
    costs, critic_total = analyze.critic_deltas(arrays)
    total = np.asarray(analyze.last(arrays, "weighted.costs"),
                       dtype=np.float64)
    costs["ControlRegularization"] = total - critic_total
    for term in TERMS:
        if term not in costs:
            raise ValueError(f"missing captured term {term}")
    good = np.asarray([record["truth_dynamic_safe"] and not
                       record["costcritic_collision"] for record in records])
    bad = np.asarray([not record["truth_dynamic_safe"] and not
                      record["costcritic_collision"] for record in records])
    costmap_collision = np.asarray(
        [record["costcritic_collision"] for record in records])
    if np.any(good & bad) or int(good.sum() + bad.sum() +
                                costmap_collision.sum()) != len(records):
        raise ValueError("safety comparison sets do not partition batch")
    probabilities = np.asarray(analyze.last(arrays, "weighted.probability"),
                               dtype=np.float64)
    if abs(float(probabilities.sum()) - 1.) > 1e-5:
        raise ValueError("captured MPPI weights do not sum to one")
    rank = np.argsort(total, kind="stable")
    compared = {key: {
        "pairwise_safe_lower_cost_auc": rank_auc(value, good, bad),
        "median_safe_minus_dynamic_unsafe_cost":
            (float(np.median(np.asarray(value)[good]) -
                   np.median(np.asarray(value)[bad]))
             if np.any(good) and np.any(bad) else None),
        "score_span": float(np.ptp(value)),
    } for key, value in [*(costs.items()), ("Total", total)]}
    return {
        "cycle_id": cycle_id,
        "cycle_json_sha256": digest(cycle),
        "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
        "predicted_collision_rollouts": summary[
            "predicted_collision_rollouts"],
        "dynamic_safe_and_costcritic_clear": int(good.sum()),
        "dynamic_unsafe_but_costcritic_clear": int(bad.sum()),
        "costcritic_collision": int(costmap_collision.sum()),
        "dynamic_safe_in_total_top_30": int(good[rank[:30]].sum()),
        "dynamic_safe_mppi_weight": float(probabilities[good].sum()),
        "terms": compared,
    }


def run(trial):
    rows = [one_cycle(trial, cycle_id) for cycle_id in WINDOW]
    eligible = [row for row in rows if row["dynamic_safe_and_costcritic_clear"]
                and row["dynamic_unsafe_but_costcritic_clear"]]
    saturated = [row for row in eligible if row[
        "predicted_collision_rollouts"] == 300]
    def summary(rows):
        return {
            "cycles": len(rows),
            "total_auc_below_half_cycles": sum(
                row["terms"]["Total"]["pairwise_safe_lower_cost_auc"] < .5
                for row in rows),
            "median_auc": {term: float(np.median([
                row["terms"][term]["pairwise_safe_lower_cost_auc"]
                for row in rows])) for term in (*TERMS, "Total")},
        }
    return {
        "schema": "rm_dynamic_prediction_critic_rank_decomposition/v1",
        "scope": "One already captured 20-cycle pre-violation window, not independent runs. Pairwise AUC uses lower score as better and treats absolute differences <=0.001 as ties to suppress float rounding of common V1 penalty. Future Gazebo truth supplies offline labels only. No score or controller changes.",
        "profile_sha256": digest(trial / "profile.yaml"),
        "truth_sha256": digest(trial / "gazebo_poses.jsonl"),
        "window": WINDOW,
        "eligible_summary": summary(eligible),
        "v1_all_collision_summary": summary(saturated),
        "cycles": rows,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.trial)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"eligible": result["eligible_summary"],
                      "v1_all_collision": result["v1_all_collision_summary"]}))
