#!/usr/bin/env python3
"""Audit the preselected phase-2 cycle with individually filtered candidates."""
import argparse
import csv
import json
from pathlib import Path
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels
from costmap_mask_fixture import export, shortcut_threshold
from filtered_all_critic_probe import filtered_controls
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_fixture import costmap_parameters, parameters
from native_critic_sensitivity import aggregate
import replay_ranking


AXES = ("vx", "vy", "wz")


def selected(trial):
    choice = json.loads((trial / "selection.json").read_text())
    cycle = Path(choice["selected_cycle_json"])
    if digest(cycle) != choice["selected_cycle_sha256"] or \
            digest(cycle.with_suffix(".bin")) != choice["selected_binary_sha256"]:
        raise ValueError("selected cycle changed")
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"]) != (
            choice["selected_cycle_id"], 300, 30):
        raise ValueError("selected cycle settings changed")
    profile_path, truth_path = trial / "profile.yaml", trial / "gazebo_poses.jsonl"
    if digest(truth_path) != choice["gazebo_truth_sha256"]:
        raise ValueError("physical truth changed")
    return cycle, meta, arrays, settings, profile_path, truth_path


def candidates(trial, meta, arrays, settings):
    previous = replay_ranking.history_from_trial(
        trial / "mppi_cycles", meta["cycle_id"], arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1).astype(np.float32)
    controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                         for axis in AXES], axis=-1)
    filtered = filtered_controls(controls, settings, history)
    poses = filtered_poses(controls, meta, settings, history)
    return controls, filtered, poses, history


def prepare(trial, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, truth_path = selected(trial)
    profile = yaml.safe_load(profile_path.read_text())
    controls, filtered, poses, history = candidates(trial, meta, arrays, settings)
    output.mkdir(parents=True)
    map_file = output / "filtered_map_and_poses.bin"
    control_file = output / "filtered_controls.bin"
    meta_file = output / "filtered_meta.json"
    export(map_file, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(poses[:, :, axis] for axis in range(3)),
           shortcut_threshold(meta, profile_path))
    control_file.write_bytes(struct.pack("<3I", 0x43545231, 300, 30) +
                             filtered.astype("<f4").tobytes())
    meta_file.write_text(json.dumps({
        "pose": meta["pose"], "speed": meta["speed"], "path": meta["path"],
        "map_frame": meta["map"]["frame"], "parameters": parameters(profile, 300),
        "costmap_parameters": costmap_parameters(profile), "batch": 300,
        "map_data_file": map_file.name}, indent=2, sort_keys=True) + "\n")
    inputs = {"schema": "rm_dynamic_prediction_phase2_filtered_inputs/v1",
              "cycle_id": meta["cycle_id"],
              "cycle_sha256": digest(cycle),
              "cycle_binary_sha256": digest(cycle.with_suffix(".bin")),
              "profile_sha256": digest(profile_path), "truth_sha256": digest(truth_path),
              "previous_cycle_ids": list(range(meta["cycle_id"] - 4, meta["cycle_id"])),
              "previous_cycle_json_sha256": {
                  str(i): digest(trial / f"mppi_cycles/cycle_{i}.json")
                  for i in range(meta["cycle_id"] - 4, meta["cycle_id"])},
              "fixture_sha256": {p.name: digest(p)
                                 for p in (map_file, control_file, meta_file)}}
    (output / "inputs.json").write_text(json.dumps(inputs, indent=2,
                                                    sort_keys=True) + "\n")
    print(json.dumps({"cycle_id": meta["cycle_id"], "fixture": str(output),
                      "source_sha256": inputs["cycle_sha256"]}))


def rank(scores, safe):
    order = np.argsort(scores, kind="stable")
    found = np.flatnonzero(safe[order])
    return {"best_joint_safe_rank": int(found[0] + 1) if len(found) else None,
            "joint_safe_top_10": int(np.count_nonzero(safe[order[:10]]))}


def evaluate(trial, fixture, native_binary, raw_scores_path, filtered_scores_path,
             filtered_mask_path, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, truth_path = selected(trial)
    inputs = json.loads((fixture / "inputs.json").read_text())
    for key, path in (("cycle_sha256", cycle), ("cycle_binary_sha256", cycle.with_suffix(".bin")),
                      ("profile_sha256", profile_path), ("truth_sha256", truth_path)):
        if digest(path) != inputs[key]:
            raise ValueError(f"frozen input changed: {key}")
    for name, sha in inputs["fixture_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError(f"filtered native fixture changed: {name}")
    for index, sha in inputs["previous_cycle_json_sha256"].items():
        if digest(trial / f"mppi_cycles/cycle_{index}.json") != sha:
            raise ValueError(f"filter history changed: {index}")
    profile = yaml.safe_load(profile_path.read_text())
    controls, filtered, poses, history = candidates(trial, meta, arrays, settings)
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    consumer = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, times,
                  consumer + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    gap, padded = geometry_labels(tuple(poses[:, :, axis] for axis in range(3)),
                                  body, meta["padded_footprint"], actual)
    mask = np.asarray([int(value) for value in filtered_mask_path.read_text().split()],
                      dtype=bool)
    if mask.shape != (300,):
        raise ValueError("native static mask count differs")
    dynamic_safe = (gap >= .05) & (padded > 0)
    joint_safe = dynamic_safe & ~mask
    raw_native = np.fromfile(raw_scores_path, dtype="<f4")
    filtered_native = np.fromfile(filtered_scores_path, dtype="<f4")
    if raw_native.shape != filtered_native.shape or raw_native.shape != (300,) or \
            not np.isfinite(filtered_native).all():
        raise ValueError("native score count or finiteness differs")
    captured_pre_v1 = analyze.last(arrays, "critic.FollowPath.PathAngleCritic")
    native_baseline_error = float(np.max(np.abs(raw_native - captured_pre_v1)))
    if native_baseline_error > 1e-3:
        raise ValueError(f"native raw scorer does not reproduce capture: {native_baseline_error}")
    captured_v1 = analyze.critic_deltas(arrays)[0]["FollowPath.PredictionV1Critic"]
    params = profile["controller_server"]["ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    _, predicted_hits, _ = filtered_prediction_score(meta, poses, params)
    if params.get("collision_rank_mode", "legacy") != "legacy" or predicted_hits[0] != 300:
        raise ValueError("expected first-step uniform legacy V1 collision")
    constant_v1 = np.float32((3.81 / 254.) * 1_000_000. / len(predicted_hits))
    if float(np.max(np.abs(captured_v1 - constant_v1))) > 1e-3:
        raise ValueError("captured V1 term is not the uniform hard score")
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    captured_weight = analyze.last(arrays, "weighted.probability")
    baseline = aggregate(analyze.last(arrays, "scored.costs"), controls,
                         initial, settings, history)
    baseline_error = float(np.max(np.abs(baseline["probability"] - captured_weight)))
    if baseline_error > 3e-5:
        raise ValueError(f"captured softmax does not replay: {baseline_error}")
    rescored = aggregate(filtered_native + constant_v1, controls,
                         initial, settings, history)
    output.mkdir(parents=True)
    detail = output / "candidates.csv"
    with detail.open("x", newline="") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(("rollout", "filtered_truth_body_gap_m", "filtered_truth_padded_gap_m",
                         "native_static_collision", "filtered_joint_safe", "filtered_standard_score",
                         "captured_standard_score", "captured_v1_score", "captured_probability",
                         "rescored_probability"))
        for index in range(300):
            writer.writerow((index, gap[index], padded[index], int(mask[index]),
                             int(joint_safe[index]), filtered_native[index], raw_native[index],
                             captured_v1[index], captured_weight[index],
                             rescored["probability"][index]))
    report = {
        "schema": "rm_dynamic_prediction_phase2_filtered_audit/v1",
        "scope": "Preselected held-out cycle; per-candidate clipped and Savitzky-Golay filtered controls with Nav2 current-speed first step. Native standard critic is rescored on each filtered candidate. V1 hard collision and native static CostCritic are separate; aggregation reweights original sampled controls, so its output is a diagnostic, not a runtime policy.",
        "cycle_id": meta["cycle_id"], "consumer_sim_s": consumer,
        "input_sha256": {"inputs": digest(fixture / "inputs.json"),
                         "native_binary": digest(native_binary),
                         "raw_native_scores": digest(raw_scores_path),
                         "filtered_native_scores": digest(filtered_scores_path),
                         "filtered_static_mask": digest(filtered_mask_path)},
        "native_raw_score_max_abs_capture_error": native_baseline_error,
        "captured_softmax_max_abs_replay_error": baseline_error,
        "filtered_dynamic_safe_count": int(np.count_nonzero(dynamic_safe)),
        "filtered_static_collision_count": int(np.count_nonzero(mask)),
        "filtered_joint_safe_count": int(np.count_nonzero(joint_safe)),
        "filtered_best_true_body_gap_m": float(np.max(gap)),
        "prediction_hits_by_step": predicted_hits,
        "captured_v1_score_span": float(np.ptp(captured_v1)),
        "filtered_native_standard_rank": rank(filtered_native, joint_safe),
        "rescored_total_rank": rank(rescored["weighted"], joint_safe),
        "captured_probability_on_filtered_joint_safe": float(captured_weight[joint_safe].sum()),
        "rescored_probability_on_filtered_joint_safe": float(rescored["probability"][joint_safe].sum()),
        "captured_top_candidate_filtered_joint_safe": bool(joint_safe[np.argmax(captured_weight)]),
        "rescored_top_candidate_filtered_joint_safe": bool(joint_safe[np.argmax(rescored["probability"])]),
        "detail_sha256": digest(detail),
    }
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                                  sort_keys=True) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("trial", type=Path)
    prep.add_argument("fixture", type=Path)
    check = sub.add_parser("evaluate")
    for name in ("trial", "fixture", "native_binary", "raw_scores", "filtered_scores",
                 "filtered_mask", "output"):
        check.add_argument(name, type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.fixture)
    else:
        evaluate(args.trial, args.fixture, args.native_binary, args.raw_scores,
                 args.filtered_scores, args.filtered_mask, args.output)
