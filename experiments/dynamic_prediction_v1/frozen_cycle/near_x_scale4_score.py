#!/usr/bin/env python3
"""Score and aggregate the four fixed scale-four near-X frozen samples."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_fixture import controls_file, costmap_parameters, parameters
from native_critic_sensitivity import aggregate
from near_x_batch_sensitivity import AXES, EXPECTED, SEEDS, frozen
import replay_ranking


SCALE = 4


def controls_for(meta, arrays, seed):
    sampled, _ = sample_omni(meta, arrays, 2000, seed)
    base = np.stack([sampled[axis] for axis in AXES], axis=-1)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1).astype(np.float32)
    return initial[None, :, :] + np.float32(SCALE) * (base - initial[None, :, :])


def raw_poses(controls, meta, settings):
    velocity = np.empty_like(controls)
    velocity[:, 0, :] = np.asarray(meta["speed"], dtype=np.float32)
    velocity[:, 1:, :] = controls[:, :-1, :]
    return analyze.integrate_omni(
        *(velocity[:, :, axis] for axis in range(3)),
        meta["pose"], settings["dt"])


def prepare(trial, noise_fixture, output):
    if output.exists():
        raise FileExistsError(output)
    paths, meta, arrays, settings = frozen(trial)
    noise = json.loads((noise_fixture / "inputs.json").read_text())
    if noise["inputs_sha256"] != EXPECTED or noise["scales"] != [2, 4]:
        raise ValueError("fixed noise input changed")
    profile = yaml.safe_load(paths["profile"].read_text())
    params = parameters(profile, 2000)
    for axis in AXES:
        params[f"FollowPath.{axis}_std"] *= SCALE
    output.mkdir(parents=True)
    files = {}
    for seed in SEEDS:
        controls = controls_for(meta, arrays, seed)
        path = output / f"seed_{seed}"
        path.mkdir()
        sampled_map = path / "raw_map.bin"
        export(sampled_map, meta, analyze.last(arrays, "locked.raw_map"),
               raw_poses(controls, meta, settings),
               shortcut_threshold(meta, paths["profile"]))
        control_path = path / "controls.bin"
        controls_file(control_path,
                      tuple(controls[:, :, axis] for axis in range(3)), 2000, 30)
        payload = {"pose": meta["pose"], "speed": meta["speed"],
                   "path": meta["path"], "map_frame": meta["map"]["frame"],
                   "parameters": params,
                   "costmap_parameters": costmap_parameters(profile),
                   "seed": seed, "batch": 2000,
                   "map_data_file": sampled_map.name}
        (path / "meta.json").write_text(json.dumps(payload, indent=2,
                                                  sort_keys=True) + "\n")
        files[str(seed)] = {name: digest(path / name)
                            for name in ("raw_map.bin", "controls.bin", "meta.json")}
    report = {"schema": "rm_dynamic_prediction_near_x_scale4_score_inputs/v1",
              "cycle_sha256": EXPECTED["cycle"], "noise_inputs_sha256": digest(
                  noise_fixture / "inputs.json"),
              "native_source_sha256": digest(Path(__file__).parent /
                  "costmap_mask_probe_cpp/src/frozen_critic_score.cpp"),
              "files_sha256": files}
    (output / "inputs.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"cases": len(SEEDS), "output": str(output)}))


def prediction_hits(meta, poses, params):
    message = analyze.event(meta, "prediction.input")
    confirmed = [track for track in message["tracks"] if track["state"] == 2]
    if len(confirmed) != 1:
        raise ValueError("single confirmed box required")
    track = confirmed[0]
    settings = analyze.event(meta, "settings")
    steps = int(math.floor(params["horizon"] / settings["dt"] + 1e-9))
    if steps != 9:
        raise ValueError("frozen V1 horizon differs")
    result = []
    for step in range(steps):
        box = analyze.predicted_box(
            track["xy"], track["vxy"], track["size_xy"],
            (params["object_width"], params["object_height"]),
            message["source_age_s"], (step + 1) * settings["dt"],
            params["reference_acceleration"]).polygon()
        result.append(sum(analyze.polygon_distance(
            analyze.placed(meta["padded_footprint"],
                           tuple(float(axis[i, step]) for axis in poses)),
            box) <= 1e-9 for i in range(2000)))
    return result


def evaluate(trial, noise_fixture, noise_masks, score_inputs, native_scores, output):
    if output.exists():
        raise FileExistsError(output)
    paths, meta, arrays, settings = frozen(trial)
    record = json.loads((score_inputs / "inputs.json").read_text())
    if record["cycle_sha256"] != EXPECTED["cycle"] or \
            record["noise_inputs_sha256"] != digest(noise_fixture / "inputs.json"):
        raise ValueError("score inputs changed")
    for seed, files in record["files_sha256"].items():
        for name, sha in files.items():
            if digest(score_inputs / f"seed_{seed}" / name) != sha:
                raise ValueError("native scorer fixture changed")
    previous = replay_ranking.history_from_trial(trial / "mppi_cycles", 73, arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1).astype(np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1).astype(np.float32)
    scaled_settings = dict(settings)
    for axis in AXES:
        scaled_settings[axis + "_std"] *= SCALE
    profile = yaml.safe_load(paths["profile"].read_text())
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    captured_v1 = analyze.critic_deltas(arrays)[0]["FollowPath.PredictionV1Critic"]
    if np.ptp(captured_v1) > 1e-3:
        raise ValueError("captured V1 hard score is not tied")
    hard = np.float32(np.mean(captured_v1))
    truth = analyze.rows_from_transport(paths["truth"])
    times = [row["t"] for row in truth]
    consumer = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    physical = [analyze.placed(analyze.obstacle_polygon(),
                analyze.interpolated_pose(truth, times,
                    consumer + (j + 1) * settings["dt"]))
                for j in range(settings["steps"])]
    goal = json.loads((trial / "observation/summary.json").read_text())["goal"]
    start_goal = float(np.hypot(meta["pose"][0] - goal[0],
                                meta["pose"][1] - goal[1]))
    output.mkdir(parents=True)
    rows, aggregate_poses = [], []
    for seed in SEEDS:
        controls = controls_for(meta, arrays, seed)
        poses = raw_poses(controls, meta, settings)
        hits = prediction_hits(meta, poses, params)
        if hits[0] != 2000:
            raise ValueError("first-step V1 hard tie no longer holds")
        scores_path = native_scores / f"seed_{seed}_scores.bin"
        native = np.fromfile(scores_path, dtype="<f4")
        with np.load(noise_fixture / f"seed_{seed}_scale_4_gaps.npz") as gaps:
            body_gap, padded_gap = gaps["body"], gaps["padded"]
        mask_path = noise_masks / f"seed_{seed}_scale_4_mask.txt"
        mask = np.loadtxt(mask_path, dtype=bool)
        if native.shape != (2000,) or body_gap.shape != native.shape or \
                padded_gap.shape != native.shape or mask.shape != native.shape:
            raise ValueError("score or label dimensions differ")
        safe = (body_gap >= .05) & (padded_gap > 0) & ~mask
        result = aggregate(native + hard, controls, initial, scaled_settings, history)
        order = np.argsort(result["weighted"], kind="stable")
        eligible = np.flatnonzero(safe[order])
        sequence = result["filtered_sequence"]
        velocity = np.concatenate((np.asarray(meta["speed"], dtype=np.float32)[None, :],
                                   sequence[:-1]), axis=0)
        one = analyze.integrate_omni(*(velocity[:, i] for i in range(3)),
                                     meta["pose"], settings["dt"])
        aggregate_poses.append(one)
        final_body, final_padded = geometry_labels(
            tuple(axis[None, :] for axis in one), body,
            meta["padded_footprint"], physical)
        progress = start_goal - float(np.hypot(one[0][-1] - goal[0],
                                               one[1][-1] - goal[1]))
        rows.append({"seed": seed, "batch": 2000,
                     "native_scores_sha256": digest(scores_path),
                     "filtered_static_mask_sha256": digest(mask_path),
                     "v1_predicted_hits_by_step": hits,
                     "v1_score_span": 0.0, "v1_hard_score": float(hard),
                     "joint_safe_count": int(safe.sum()),
                     "best_safe_weighted_rank": int(eligible[0] + 1)
                     if len(eligible) else None,
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
                     "aggregate_goal_progress_m": progress})
    map_path = output / "aggregate_map.bin"
    export(map_path, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack([pose[axis] for pose in aggregate_poses])
                 for axis in range(3)),
           shortcut_threshold(meta, paths["profile"]))
    report = {"schema": "rm_dynamic_prediction_near_x_scale4_score/v1",
              "scope": "Four fixed scale-four offline raw sampled trajectories scored with native standard critics and original tied V1 hard term; MPPI regularizer uses scaled std. Original output filter and current-speed-first-step final geometry. Not a runtime controller trial.",
              "score_inputs_sha256": digest(score_inputs / "inputs.json"),
              "aggregate_map_sha256": digest(map_path),
              "rows": rows}
    (output / "preliminary.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"safe_rank": [row["best_safe_weighted_rank"] for row in rows],
                      "output_gate": [row["aggregate_dynamic_gate_pass"] for row in rows]}))


def finalize(output):
    path = output / "preliminary.json"
    report = json.loads(path.read_text())
    if digest(output / "aggregate_map.bin") != report["aggregate_map_sha256"]:
        raise ValueError("aggregate map fixture changed")
    mask_path = output / "aggregate_static_mask.txt"
    mask = np.loadtxt(mask_path, dtype=bool)
    if mask.shape != (4,) or (output / "summary.json").exists():
        raise ValueError("aggregate static mask differs or output exists")
    for row, flag in zip(report["rows"], mask):
        row["aggregate_native_static_collision"] = bool(flag)
        row["aggregate_joint_gate_pass"] = bool(
            row["aggregate_dynamic_gate_pass"] and not flag)
    report["aggregate_static_mask_sha256"] = digest(mask_path)
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                              sort_keys=True) + "\n")
    print(json.dumps({"output_joint_gate": [row["aggregate_joint_gate_pass"]
                      for row in report["rows"]]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("trial", "noise-fixture", "output"):
        prep.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("trial", "noise-fixture", "noise-masks", "score-inputs",
                 "native-scores", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    done = sub.add_parser("finalize")
    done.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.noise_fixture, args.output)
    elif args.command == "evaluate":
        evaluate(args.trial, args.noise_fixture, args.noise_masks,
                 args.score_inputs, args.native_scores, args.output)
    else:
        finalize(args.output)
