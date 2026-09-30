#!/usr/bin/env python3
"""Isolated guarded-PathAlign score and MPPI aggregate on south cycle 65."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_fixture import controls_file, costmap_parameters, parameters
from native_critic_sensitivity import aggregate
from near_x_scale4_score import controls_for, raw_poses
from near_x_south_batch import SEEDS, frozen, physical
from phase2_filtered_audit import candidates


SCORER_SHA = "6d243002735e96d9a99104934b5b7284483a2542037c8761c34718831eef5aeb"
PLUGIN_SHA = "fe8f7895bd7cc2ec2ab2e4e5f46e49034682ec56dbae5049acec05ac1fa06e83"
SOURCE_MANIFEST_SHA = "c510263c32814564e35d9e4f700bb316a0a21922ddf131ec89e1c411e3f55210"
AXES = ("vx", "vy", "wz")


def prepare(trial, noise_fixture, scorer, plugin, source_manifest, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, _ = frozen(trial)
    if (digest(scorer), digest(plugin), digest(source_manifest)) != (
            SCORER_SHA, PLUGIN_SHA, SOURCE_MANIFEST_SHA):
        raise ValueError("validated guarded scorer source or binary changed")
    noise = json.loads((noise_fixture / "inputs.json").read_text())
    if noise["cycle_sha256"] != digest(cycle) or noise["scales"] != [2, 4]:
        raise ValueError("fixed noise fixture changed")
    profile = yaml.safe_load(profile_path.read_text())
    params = parameters(profile, 2000)
    for axis in AXES:
        params[f"FollowPath.{axis}_std"] *= 4
    output.mkdir(parents=True)
    files = {}
    for seed in SEEDS:
        controls = controls_for(meta, arrays, seed)
        case = output / f"seed_{seed}"
        case.mkdir()
        map_path = case / "raw_map.bin"
        export(map_path, meta, analyze.last(arrays, "locked.raw_map"),
               raw_poses(controls, meta, settings),
               shortcut_threshold(meta, profile_path))
        control_path = case / "controls.bin"
        controls_file(control_path,
                      tuple(controls[:, :, axis] for axis in range(3)), 2000, 30)
        meta_path = case / "meta.json"
        meta_path.write_text(json.dumps({
            "pose": meta["pose"], "speed": meta["speed"],
            "path": meta["path"], "map_frame": meta["map"]["frame"],
            "parameters": params, "costmap_parameters": costmap_parameters(profile),
            "seed": seed, "batch": 2000, "map_data_file": map_path.name},
            indent=2, sort_keys=True) + "\n")
        files[str(seed)] = {p.name: digest(p) for p in
                            (map_path, control_path, meta_path)}
    record = {"schema": "rm_dynamic_prediction_south_guarded_score_inputs/v1",
              "cycle_sha256": digest(cycle),
              "noise_inputs_sha256": digest(noise_fixture / "inputs.json"),
              "captured_fixture_sha256": {p.name: digest(p) for p in (
                  trial / "native_raw_cycle_65/meta.json",
                  trial / "native_raw_cycle_65/captured.bin",
                  trial / "native_raw_cycle_65/controls.bin")},
              "scorer_sha256": digest(scorer), "plugin_sha256": digest(plugin),
              "source_manifest_sha256": digest(source_manifest),
              "files_sha256": files}
    (output / "inputs.json").write_text(json.dumps(record, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"cases": len(SEEDS) + 1, "output": str(output)}))


def evaluate(trial, noise_fixture, noise_masks, score_inputs, scores, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, truth_path = frozen(trial)
    record = json.loads((score_inputs / "inputs.json").read_text())
    if record["cycle_sha256"] != digest(cycle) or \
            record["noise_inputs_sha256"] != digest(noise_fixture / "inputs.json"):
        raise ValueError("guarded score inputs changed")
    for seed, names in record["files_sha256"].items():
        for name, sha in names.items():
            if digest(score_inputs / f"seed_{seed}" / name) != sha:
                raise ValueError("guarded scorer fixture changed")
    captured_native = np.fromfile(scores / "captured_scores.bin", dtype="<f4")
    captured_legacy = np.fromfile(trial / "native_raw_cycle_65/raw_scores.bin",
                                  dtype="<f4")
    captured_trace = analyze.last(arrays, "critic.FollowPath.PathAngleCritic")
    if captured_native.shape != (300,) or captured_legacy.shape != (300,):
        raise ValueError("captured guarded score size differs")
    captured_comparison = {
        "guarded_score_sha256": digest(scores / "captured_scores.bin"),
        "guarded_vs_captured_trace_changed_rollouts": np.flatnonzero(
            np.abs(captured_native - captured_trace) > 1e-3).astype(int).tolist(),
        "guarded_vs_unbounded_native_changed_rollouts": np.flatnonzero(
            np.abs(captured_native - captured_legacy) > 1e-3).astype(int).tolist(),
        "guarded_vs_trace_max_abs": float(np.max(np.abs(
            captured_native - captured_trace)))}
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    actual = physical(meta, settings, truth_path)
    history = candidates(trial, meta, arrays, settings)[3]
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1).astype(np.float32)
    scaled = dict(settings)
    for axis in AXES:
        scaled[axis + "_std"] *= 4
    hard = analyze.critic_deltas(arrays)[0]["FollowPath.PredictionV1Critic"]
    if np.ptp(hard) > 1e-3:
        raise ValueError("captured V1 hard score is not tied")
    hard_constant = np.float32(np.mean(hard))
    goal = json.loads((trial / "observation/summary.json").read_text())["goal"]
    start_goal = float(np.hypot(meta["pose"][0] - goal[0],
                                meta["pose"][1] - goal[1]))
    output.mkdir(parents=True)
    rows, final_poses = [], []
    for seed in SEEDS:
        controls = controls_for(meta, arrays, seed)
        guarded_path = scores / f"seed_{seed}_scores.bin"
        native = np.fromfile(guarded_path, dtype="<f4")
        with np.load(noise_fixture / f"seed_{seed}_scale_4_gaps.npz") as data:
            body_gap, padded_gap = data["body"], data["padded"]
        mask_path = noise_masks / f"seed_{seed}_scale_4_mask.txt"
        mask = np.loadtxt(mask_path, dtype=bool)
        if native.shape != (2000,) or body_gap.shape != native.shape or \
                padded_gap.shape != native.shape or mask.shape != native.shape:
            raise ValueError("guarded score or label count differs")
        safe = (body_gap >= .05) & (padded_gap > 0) & ~mask
        result = aggregate(native + hard_constant, controls, initial, scaled, history)
        order = np.argsort(result["weighted"], kind="stable")
        positions = np.flatnonzero(safe[order])
        sequence = result["filtered_sequence"]
        velocity = np.concatenate((np.asarray(meta["speed"], dtype=np.float32)[None, :],
                                   sequence[:-1]), axis=0)
        pose = analyze.integrate_omni(*(velocity[:, axis] for axis in range(3)),
                                      meta["pose"], settings["dt"])
        final_poses.append(pose)
        final_body, final_padded = geometry_labels(
            tuple(axis[None, :] for axis in pose), body,
            meta["padded_footprint"], actual)
        rows.append({"seed": seed, "batch": 2000,
                     "guarded_scores_sha256": digest(guarded_path),
                     "static_mask_sha256": digest(mask_path),
                     "joint_safe_count": int(safe.sum()),
                     "best_safe_weighted_rank": int(positions[0] + 1)
                     if len(positions) else None,
                     "joint_safe_top_10": int(np.sum(safe[order[:10]])),
                     "joint_safe_probability_mass": float(result[
                         "probability"][safe].sum()),
                     "effective_sample_size": float(1. / np.sum(
                         result["probability"] ** 2)),
                     "returned_control": result["returned_control"],
                     "aggregate_body_gap_m": float(final_body[0]),
                     "aggregate_padded_gap_m": float(final_padded[0]),
                     "aggregate_dynamic_gate_pass": bool(
                         final_body[0] >= .05 and final_padded[0] > 0),
                     "aggregate_goal_progress_m": start_goal - float(np.hypot(
                         pose[0][-1] - goal[0], pose[1][-1] - goal[1]))})
    map_path = output / "aggregate_map.bin"
    export(map_path, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack([pose[axis] for pose in final_poses])
                 for axis in range(3)),
           shortcut_threshold(meta, profile_path))
    report = {"schema": "rm_dynamic_prediction_south_guarded_scale4_score/v1",
              "scope": "Isolated previously validated guarded PathAlign C++ scorer on frozen raw samples; V1 legacy hard term common. Offline aggregate and physical future labels only, no runtime controller change.",
              "score_inputs_sha256": digest(score_inputs / "inputs.json"),
              "captured_comparison": captured_comparison,
              "aggregate_map_sha256": digest(map_path), "rows": rows}
    (output / "preliminary.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"safe_rank": [row["best_safe_weighted_rank"] for row in rows],
                      "dynamic_gate": [row["aggregate_dynamic_gate_pass"] for row in rows]}))


def finalize(output):
    report = json.loads((output / "preliminary.json").read_text())
    if digest(output / "aggregate_map.bin") != report["aggregate_map_sha256"]:
        raise ValueError("aggregate map changed")
    mask_path = output / "aggregate_static_mask.txt"
    mask = np.loadtxt(mask_path, dtype=bool)
    if mask.shape != (4,) or (output / "summary.json").exists():
        raise ValueError("aggregate static mask size differs")
    for row, collision in zip(report["rows"], mask):
        row["aggregate_native_static_collision"] = bool(collision)
        row["aggregate_joint_gate_pass"] = bool(
            row["aggregate_dynamic_gate_pass"] and not collision)
    report["aggregate_static_mask_sha256"] = digest(mask_path)
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                              sort_keys=True) + "\n")
    print(json.dumps({"joint_gate": [row["aggregate_joint_gate_pass"]
                      for row in report["rows"]]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("trial", "noise-fixture", "scorer", "plugin", "source-manifest", "output"):
        prep.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("trial", "noise-fixture", "noise-masks", "score-inputs", "scores", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    done = sub.add_parser("finalize")
    done.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.noise_fixture, args.scorer, args.plugin,
                args.source_manifest, args.output)
    elif args.command == "evaluate":
        evaluate(args.trial, args.noise_fixture, args.noise_masks,
                 args.score_inputs, args.scores, args.output)
    else:
        finalize(args.output)
