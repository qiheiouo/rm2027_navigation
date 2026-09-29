#!/usr/bin/env python3
"""Explain top-ranked versus best true-safe filtered candidates at cycle 162.

Future Gazebo poses choose the comparison candidate only after fixed scores
and weights are reconstructed. They do not enter any candidate score.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, sample_omni
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_sensitivity import AXES, aggregate
from output_first_step_audit import actual
from peak_temporal_rank_probe import step_overlap
import replay_ranking


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    cycle = args.trial / "mppi_cycles/cycle_162.json"
    profile_path = args.trial / "profile.yaml"
    truth_path = args.trial / "gazebo_poses.jsonl"
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"]) != (162, 300, 30):
        raise ValueError("not the frozen cycle 162")
    profile = yaml.safe_load(profile_path.read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    history_axes = replay_ranking.history_from_trial(
        args.trial / "mppi_cycles", 162, arrays)
    history = np.stack([history_axes[axis] for axis in AXES], axis=-1)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    truth_boxes = actual(meta, settings, truth_path)
    prediction = analyze.event(meta, "prediction.input")
    track = next(track for track in prediction["tracks"] if track["state"] == 2)
    if sum(track["state"] == 2 for track in prediction["tracks"]) != 1:
        raise ValueError("single confirmed box fixture required")
    output_summary = (args.evidence / "output_first_step_audit_20260928" /
                      "summary.json")
    published = json.loads(output_summary.read_text())
    output_rows = {row["name"]: row for row in published["rows"]}
    critic_summary = (args.evidence / "filtered_all_critic_20260928" /
                      "summary.json")
    all_critic = json.loads(critic_summary.read_text())
    expected_controls = {row["name"]: row["all_critic_returned_control"]
                         for row in all_critic["rows"]}
    rows = []
    for seed in (-1, 2, 3):
        name = "captured_batch300" if seed == -1 else f"seed_{seed}_batch300"
        if seed == -1:
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
        else:
            sampled, _ = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([sampled[axis][:300] for axis in AXES], axis=-1)
        poses = filtered_poses(controls, meta, settings, history)
        v1, hits, mean = filtered_prediction_score(meta, poses, params)
        by_step = step_overlap(meta, poses, params)
        if float(np.max(np.abs(by_step.mean(axis=1) - mean))) > 1e-8:
            raise ValueError("stepwise overlap differs from original V1")
        score_path = (args.evidence / "filtered_all_critic_20260928" /
                      f"{name}_scores.bin")
        standard = np.fromfile(score_path, dtype="<f4")
        if standard.shape != (300,):
            raise ValueError("native standard score count differs")
        result = aggregate(standard + v1, controls, initial, settings, history)
        control_error = float(np.max(np.abs(
            np.asarray(result["returned_control"]) -
            np.asarray(expected_controls[name]))))
        if control_error > 1e-5:
            raise ValueError(f"published control differs: {name}")
        label_path = (args.evidence / "filtered_batch_candidate_rank_20260928" /
                      ("captured_batch300_labels.csv" if seed == -1 else
                       f"seed_{seed}_batch2000_labels.csv"))
        with label_path.open(newline="") as source:
            labels = list(csv.DictReader(source))[:300]
        safe = np.asarray([row["joint_safe_3s"] == "1" for row in labels])
        if not safe.any():
            raise ValueError("no safe comparison candidate")
        top = int(np.argmin(result["weighted"]))
        best_safe = int(np.flatnonzero(safe)[np.argmin(result["weighted"][safe])])
        rank = int(np.flatnonzero(np.argsort(result["weighted"]) == best_safe)[0] + 1)

        def candidate(index):
            dynamic = []
            for step in range(9):
                pose = tuple(float(x) for x in poses[index, step])
                robot_body = analyze.placed(body, pose)
                robot_padded = analyze.placed(meta["padded_footprint"], pose)
                predicted = analyze.predicted_box(
                    track["xy"], track["vxy"], track["size_xy"],
                    (params["object_width"], params["object_height"]),
                    prediction["source_age_s"], (step + 1) * settings["dt"],
                    params["reference_acceleration"]).polygon()
                dynamic.append({
                    "step": step + 1,
                    "truth_body_gap_m": analyze.polygon_distance(
                        robot_body, truth_boxes[step]),
                    "truth_padded_gap_m": analyze.polygon_distance(
                        robot_padded, truth_boxes[step]),
                    "predicted_hard_padded_gap_m": analyze.polygon_distance(
                        robot_padded, predicted),
                    "uniform_center_overlap_fraction": float(
                        by_step[index, step])})
            return {"index": index, "joint_safe_3s": bool(safe[index]),
                    "body_min_gap_3s_m": float(labels[index]["body_gap_3s_m"]),
                    "static_collision": labels[index]["static_collision"] == "1",
                    "standard_score": float(standard[index]),
                    "v1_score": float(v1[index]),
                    "mppi_regularizer": float(result["weighted"][index] -
                                              standard[index] - v1[index]),
                    "weighted_score": float(result["weighted"][index]),
                    "probability": float(result["probability"][index]),
                    "steps": dynamic}

        row = {"name": name, "seed": seed,
               "native_standard_score_sha256": digest(score_path),
               "truth_label_sha256": digest(label_path),
               "published_control_max_abs_error": control_error,
               "hard_predicted_hits_by_step": hits,
               "safe_candidate_count": int(safe.sum()),
               "best_safe_rank": rank,
               "safe_probability_mass": float(result["probability"][safe].sum()),
               "effective_sample_size": float(1. / np.sum(
                   result["probability"] ** 2)),
               "published_output_body_gap_m": output_rows[name][
                   "native_first_body_gap_m"],
               "top": candidate(top), "best_true_safe": candidate(best_safe)}
        rows.append(row)
    source_age = prediction["source_age_s"]
    truth = analyze.rows_from_transport(truth_path)
    truth_times = [row["t"] for row in truth]
    truth_pose = analyze.interpolated_pose(
        truth, truth_times, prediction["consumer_sim_s"] + .4)
    report = {"schema": "rm_dynamic_prediction/near_term_pair_audit/v1",
              "scope": "Frozen cycle 162, captured 300 and fixed seed 2/3 300. Scores and weights are reconstructed before Gazebo truth chooses the best truly safe comparison candidate. Hard V1, continuous V1, seven native critics, original MPPI aggregation and thresholds unchanged. First nine steps only; descriptive mechanism audit, not a new policy.",
              "input_sha256": {"cycle": digest(cycle),
                               "cycle_bin": digest(cycle.with_suffix(".bin")),
                               "profile": digest(profile_path),
                               "truth": digest(truth_path),
                               "output_summary": digest(output_summary),
                               "critic_summary": digest(critic_summary)},
              "prediction_source_age_s": source_age,
              "track_visible_xy": track["xy"],
              "track_vxy": track["vxy"],
              "predicted_visible_xy_step4": [
                  track["xy"][axis] + track["vxy"][axis] * (source_age + .4)
                  for axis in (0, 1)],
              "truth_physical_center_xy_step4": truth_pose[:2],
              "cases": rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args())
