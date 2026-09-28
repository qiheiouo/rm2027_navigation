#!/usr/bin/env python3
"""Audit V1, native-standard and total ranking of filtered candidates.

All labels use the frozen raw costmap and future Gazebo box only after scores
are fixed. No truth label participates in the MPPI ranking calculation.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_sensitivity import AXES, aggregate
import replay_ranking


CASES = (("early_147", 147, "collision_trial", "score147"),
         ("intrusion_162", 162, "collision_trial", "score162"),
         ("goal_263", 263, "goal_trial", "score263"))


def trial_inputs(args, name, cycle_id, trial_key, score_key):
    trial = getattr(args, trial_key)
    cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile = trial / "profile.yaml"
    truth = trial / "gazebo_poses.jsonl"
    standard = getattr(args, score_key)
    return {"name": name, "cycle_id": cycle_id,
            "trial": str(trial),
            "cycle": str(cycle), "cycle_sha256": digest(cycle),
            "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
            "profile": str(profile), "profile_sha256": digest(profile),
            "truth": str(truth), "truth_sha256": digest(truth),
            "standard": str(standard),
            "standard_sha256": digest(standard)}


def load_case(row):
    for name in ("cycle", "profile", "truth", "standard"):
        if digest(Path(row[name])) != row[name + "_sha256"]:
            raise ValueError(f"frozen input changed: {row['name']}, {name}")
    cycle = Path(row["cycle"])
    if digest(cycle.with_suffix(".bin")) != row["cycle_bin_sha256"]:
        raise ValueError(f"frozen cycle arrays changed: {row['name']}")
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"]) != (
            row["cycle_id"], 300, 30):
        raise ValueError("frozen cycle settings changed")
    trial = Path(row["trial"])
    profile = yaml.safe_load(Path(row["profile"]).read_text())
    controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                         for axis in AXES], axis=-1)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    previous = replay_ranking.history_from_trial(
        trial / "mppi_cycles", row["cycle_id"], arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1)
    poses = filtered_poses(controls, meta, settings, history)
    return meta, arrays, settings, profile, controls, initial, history, poses


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    cases = []
    for name, cycle_id, trial_key, score_key in CASES:
        row = trial_inputs(args, name, cycle_id, trial_key, score_key)
        meta, arrays, _, _, _, _, _, poses = load_case(row)
        fixture = args.output_dir / f"{name}_candidates.bin"
        export(fixture, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(poses[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, Path(row["profile"])))
        row["static_fixture"] = str(fixture.resolve())
        row["static_fixture_sha256"] = digest(fixture)
        cases.append(row)
    (args.output_dir / "inputs.json").write_text(json.dumps(
        {"schema": "rm_dynamic_prediction/filtered_candidate_rank_inputs/v1",
         "cases": cases}, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(cases)}))


def safe_auc(scores, safe):
    positive = np.asarray(scores[safe], dtype=np.float64)
    negative = np.asarray(scores[~safe], dtype=np.float64)
    if not len(positive) or not len(negative):
        return None
    pairs = positive[:, None] - negative[None, :]
    return float((np.count_nonzero(pairs < 0) +
                  .5 * np.count_nonzero(pairs == 0)) / pairs.size)


def ranked_metrics(score, safe):
    order = np.argsort(score, kind="stable")
    safe_order = safe[order]
    return {"safe_vs_unsafe_auc": safe_auc(score, safe),
            "top_10_joint_safe_count": int(np.count_nonzero(safe_order[:10])),
            "best_joint_safe_rank": (int(np.flatnonzero(safe_order)[0]) + 1
                                     if safe.any() else None)}


def evaluate_case(row, mask_dir, detail_dir):
    meta, arrays, settings, profile, controls, initial, history, poses = load_case(row)
    fixture = Path(row["static_fixture"])
    if digest(fixture) != row["static_fixture_sha256"]:
        raise ValueError(f"static fixture changed: {row['name']}")
    mask_path = mask_dir / f"{row['name']}_mask.txt"
    native_values = [int(value) for value in mask_path.read_text().split()]
    if len(native_values) != 300 or any(value not in (0, 1)
                                         for value in native_values):
        raise ValueError(f"native static mask differs: {row['name']}")
    mask = np.asarray(native_values, dtype=bool)
    truth = analyze.rows_from_transport(Path(row["truth"]))
    times = [sample["t"] for sample in truth]
    stamp = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(
                  truth, times, stamp + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    body_gap, padded_gap = geometry_labels(
        tuple(poses[:, :, axis] for axis in range(3)), body,
        meta["padded_footprint"], actual)
    dynamic_safe = (body_gap >= .05) & (padded_gap > 0)
    joint_safe = dynamic_safe & ~mask
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    prediction, hits, _ = filtered_prediction_score(meta, poses, params)
    window = len(hits)
    window_body_gap, window_padded_gap = geometry_labels(
        tuple(poses[:, :window, axis] for axis in range(3)), body,
        meta["padded_footprint"], actual[:window])
    prediction_window_safe = ((window_body_gap >= .05) &
                              (window_padded_gap > 0))
    standard = np.fromfile(row["standard"], dtype="<f4")
    if standard.shape != (300,) or not np.isfinite(standard).all():
        raise ValueError(f"native standard score invalid: {row['name']}")
    result = aggregate(standard + prediction, controls, initial,
                       settings, history)
    probability = result["probability"]
    report = {"name": row["name"], "cycle_id": row["cycle_id"],
              "input_sha256": {key: row[key] for key in (
                  "cycle_sha256", "cycle_bin_sha256", "profile_sha256",
                  "truth_sha256", "standard_sha256",
                  "static_fixture_sha256")},
              "static_mask_sha256": digest(mask_path),
              "dynamic_safe_count": int(np.count_nonzero(dynamic_safe)),
              "static_collision_count": int(np.count_nonzero(mask)),
              "joint_safe_count": int(np.count_nonzero(joint_safe)),
              "joint_safe_probability_mass": float(probability[joint_safe].sum()),
              "prediction_hits_by_step": hits,
              "prediction_window_steps": window,
              "prediction_window_dynamic_safe_count": int(np.count_nonzero(
                  prediction_window_safe)),
              "prediction_window_v1_safe_auc": safe_auc(
                  prediction, prediction_window_safe),
              "prediction_window_min_truth_body_gap_m": float(
                  window_body_gap.min()),
              "prediction_score_span": float(np.ptp(prediction)),
              "prediction_ranking": ranked_metrics(prediction, joint_safe),
              "native_standard_ranking": ranked_metrics(standard, joint_safe),
              "total_weighted_ranking": ranked_metrics(result["weighted"],
                                                       joint_safe),
              "total_top_candidate_joint_safe": bool(joint_safe[np.argmax(
                  probability)]),
              "score_effective_sample_size": float(
                  1. / np.sum(probability ** 2))}
    detail = detail_dir / f"{row['name']}_detail.csv"
    with detail.open("x", newline="") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(("rollout", "prediction_window_body_gap_m",
                         "prediction_window_padded_gap_m", "body_gap_3s_m",
                         "padded_gap_3s_m", "static_collision",
                         "joint_safe_3s", "prediction_v1_score",
                         "native_standard_score", "total_weighted_score",
                         "probability"))
        for index in range(300):
            writer.writerow((index, window_body_gap[index],
                             window_padded_gap[index], body_gap[index],
                             padded_gap[index], int(mask[index]),
                             int(joint_safe[index]), prediction[index],
                             standard[index], result["weighted"][index],
                             probability[index]))
    report["detail_file"] = detail.name
    report["detail_sha256"] = digest(detail)
    if row["cycle_id"] == 263:
        goal = json.loads((Path(row["trial"]) / "runtime_audit.json")
                          .read_text())["navigation_result"]["goal"]
        goal_mask = np.hypot(poses[:, -1, 0] - goal[0],
                             poses[:, -1, 1] - goal[1]) <= .15
        report["filtered_goal_candidate_count"] = int(goal_mask.sum())
        report["filtered_goal_joint_safe_count"] = int(np.count_nonzero(
            goal_mask & joint_safe))
        report["filtered_goal_candidate_probability_mass"] = float(
            probability[goal_mask].sum())
        report["filtered_goal_joint_safe_probability_mass"] = float(
            probability[goal_mask & joint_safe].sum())
    return report


def evaluate(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    inputs = json.loads(args.inputs.read_text())
    report = {"schema": "rm_dynamic_prediction/filtered_candidate_rank_audit/v1",
              "scope": "Preselected cycles 147, 162, 263. Individually clipped/filtered candidates, native current-speed first-step integration, seven guarded native standard critic scores and frozen filtered V1. Joint safe means true body clearance >= 0.05m and true padded clearance > 0 across 30 steps plus native raw-costmap CostCritic no collision. The V1 prediction-window label uses only its nine scored steps and the dynamic geometry. Future truth labels only after scores are fixed. AUC is P(lower score for safe than unsafe); samples and cycles are correlated.",
              "inputs_sha256": digest(args.inputs),
              "native_mask_binary_sha256": digest(args.native_binary),
              "cases": [evaluate_case(row, args.mask_dir, args.output.parent)
                        for row in inputs["cases"]]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["cases"])}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("collision-trial", "goal-trial", "score147", "score162",
                 "score263", "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("inputs", "mask-dir", "native-binary", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "evaluate": evaluate}[args.command](args)
