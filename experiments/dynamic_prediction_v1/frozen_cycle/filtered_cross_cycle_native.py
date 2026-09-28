#!/usr/bin/env python3
"""Native seven-critic follow-up for filtered V1 candidates at 147 and 263.

This closes the standard-critic representation gap in the preselected
cross-cycle screen. Aggregation still uses original controls plus final filter.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from costmap_mask_fixture import export, shortcut_threshold
from filtered_all_critic_probe import filtered_controls
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_fixture import costmap_parameters, parameters
from native_critic_sensitivity import aggregate
import replay_ranking
import short_horizon_filter_probe as short


CASES = (("early_147", 147), ("goal_263", 263))
AXES = ("vx", "vy", "wz")


def case_trial(args, case):
    return args.collision_trial if case == "early_147" else args.goal_trial


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    rows = []
    for name, cycle_id in CASES:
        trial = case_trial(args, name)
        cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
        profile_path = trial / "profile.yaml"
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        if settings["batch"] != 300 or settings["steps"] != 30:
            raise ValueError(f"frozen cycle settings differ: {name}")
        profile = yaml.safe_load(profile_path.read_text())
        params = profile["controller_server"]["ros__parameters"]["FollowPath"][
            "PredictionV1Critic"]
        previous = replay_ranking.history_from_trial(
            trial / "mppi_cycles", cycle_id, arrays)
        history = np.stack([previous[axis] for axis in AXES], axis=-1)
        controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                             for axis in AXES], axis=-1)
        filtered = filtered_controls(controls, settings, history)
        poses = filtered_poses(controls, meta, settings, history)
        prediction, hits, _ = filtered_prediction_score(meta, poses, params)
        target = args.output_dir / name
        target.mkdir()
        map_path = target / "map_and_poses.bin"
        controls_path = target / "controls.bin"
        prediction_path = target / "prediction.bin"
        export(map_path, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(poses[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        controls_path.write_bytes(struct.pack("<3I", 0x43545231, 300, 30) +
                                  filtered.astype("<f4").tobytes())
        prediction.astype("<f4").tofile(prediction_path)
        fixture_meta = {
            "pose": meta["pose"], "speed": meta["speed"],
            "path": meta["path"], "map_frame": meta["map"]["frame"],
            "parameters": parameters(profile, 300),
            "costmap_parameters": costmap_parameters(profile),
            "batch": 300, "map_data_file": map_path.name}
        meta_path = target / "meta.json"
        meta_path.write_text(json.dumps(fixture_meta, indent=2,
                                        sort_keys=True) + "\n")
        rows.append({"name": name, "cycle_id": cycle_id,
                     "cycle": str(cycle), "cycle_sha256": digest(cycle),
                     "profile": str(profile_path),
                     "profile_sha256": digest(profile_path),
                     "map": str(map_path.resolve()),
                     "map_sha256": digest(map_path),
                     "controls": str(controls_path.resolve()),
                     "controls_sha256": digest(controls_path),
                     "prediction": str(prediction_path.resolve()),
                     "prediction_sha256": digest(prediction_path),
                     "meta": str(meta_path.resolve()),
                     "meta_sha256": digest(meta_path),
                     "prediction_hits_by_step": hits})
    record = {"schema": "rm_dynamic_prediction/filtered_cross_cycle_inputs/v1",
              "cases": rows}
    (args.output_dir / "inputs.json").write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


def native(args):
    inputs = json.loads(args.inputs.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=True)

    def mounted(host):
        return args.mount_root / Path(host).relative_to(args.host_root)

    for case in inputs["cases"]:
        for key in ("meta", "map", "controls"):
            if digest(mounted(case[key])) != case[key + "_sha256"]:
                raise ValueError(f"fixture changed: {case['name']}, {key}")
        score = args.output_dir / f"{case['name']}_scores.bin"
        result = subprocess.run(
            [str(args.binary), str(mounted(case["meta"])),
             str(mounted(case["map"])), str(mounted(case["controls"])),
             str(score)], capture_output=True, text=True, check=False)
        (args.output_dir / f"{case['name']}_log.txt").write_text(
            result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f"{case['name']}: {result.stderr[-1000:]}")
    print(json.dumps({"cases": len(inputs["cases"])}))


def evaluate(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    inputs = json.loads(args.inputs.read_text())
    screen = json.loads(args.screen.read_text())
    screen_rows = {row["cycle_id"]: row for row in screen["cycles"]}
    goal = json.loads((args.goal_trial / "runtime_audit.json").read_text())[
        "navigation_result"]["goal"]
    rows = []
    for case in inputs["cases"]:
        name, cycle_id = case["name"], case["cycle_id"]
        trial = case_trial(args, name)
        cycle = Path(case["cycle"])
        profile_path = Path(case["profile"])
        if digest(cycle) != case["cycle_sha256"] or digest(profile_path) != case[
                "profile_sha256"]:
            raise ValueError("frozen cycle or profile changed")
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1)
        controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                             for axis in AXES], axis=-1)
        axes_history = replay_ranking.history_from_trial(
            trial / "mppi_cycles", cycle_id, arrays)
        history = np.stack([axes_history[axis] for axis in AXES], axis=-1)
        score_path = args.scores / f"{name}_scores.bin"
        standard = np.fromfile(score_path, dtype="<f4")
        prediction_path = Path(case["prediction"])
        if digest(prediction_path) != case["prediction_sha256"]:
            raise ValueError("prediction fixture changed")
        prediction = np.fromfile(prediction_path, dtype="<f4")
        if standard.shape != prediction.shape or standard.shape != (300,):
            raise ValueError("score count differs")
        result = aggregate(standard + prediction, controls, initial,
                           settings, history)
        truth, times, body, box, inputs_hash = short.trial_input(trial)
        gap = short.control_gap(
            {axis: result["filtered_sequence"][:, i]
             for i, axis in enumerate(AXES)},
            meta, settings, truth, times, body, box,
            analyze.event(meta, "prediction.input")["consumer_sim_s"])
        row = {"name": name, "cycle_id": cycle_id,
               "standard_score_sha256": digest(score_path),
               "prediction_score_sha256": digest(prediction_path),
               "raw_critic_mixed_screen": screen_rows[cycle_id]["candidate"],
               "native_all_critic_body_min_gap_m": gap[
                   "body_min_clearance_m"],
               "native_all_critic_body_min_gap_step": gap["step"],
               "returned_vx_vy_wz": result["returned_control"],
               "effective_sample_size": float(
                   1. / np.sum(result["probability"] ** 2)),
               "truth_sha256": inputs_hash["truth_sha256"]}
        if cycle_id == 263:
            x, y = (analyze.last(arrays, "rollout." + axis)
                    for axis in ("x", "y"))
            raw_goal = np.hypot(x[:, -1] - goal[0],
                                y[:, -1] - goal[1]) <= .15
            filtered = filtered_poses(controls, meta, settings, history)
            filtered_goal = np.hypot(filtered[:, -1, 0] - goal[0],
                                     filtered[:, -1, 1] - goal[1]) <= .15
            baseline_probability = analyze.last(arrays,
                                                 "weighted.probability")
            trajectory = analyze.integrate_omni(
                *(result["filtered_sequence"][:, i] for i in range(3)),
                meta["pose"], settings["dt"])
            row["goal_position_rollout_weight"] = float(
                result["probability"][raw_goal].sum())
            row["raw_goal_candidate_count"] = int(raw_goal.sum())
            row["filtered_goal_candidate_count"] = int(filtered_goal.sum())
            row["filtered_goal_candidate_weight"] = float(
                result["probability"][filtered_goal].sum())
            row["baseline_filtered_goal_candidate_weight"] = float(
                baseline_probability[filtered_goal].sum())
            row["filtered_goal_candidates_without_v1_penalty"] = int(
                np.count_nonzero(filtered_goal & (prediction == 0)))
            row["endpoint_goal_position_distance_m"] = float(np.hypot(
                trajectory[0][-1] - goal[0],
                trajectory[1][-1] - goal[1]))
        rows.append(row)
    report = {"schema": "rm_dynamic_prediction/filtered_cross_cycle_native/v2",
              "scope": "Preselected cycles 147 and 263. Each filtered candidate pose/control receives seven guarded native standard critic scores and a frozen V1 hard/graded prediction term. MPPI aggregates original sampled controls and applies the original final filter. Offline truth only evaluates output; static aggregate map check not included.",
              "inputs_sha256": digest(args.inputs),
              "screen_sha256": digest(args.screen),
              "native_binary_sha256": digest(args.native_binary),
              "rows": rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cycles": [row["cycle_id"] for row in rows]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("collision-trial", "goal-trial", "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    n = sub.add_parser("native")
    for name in ("inputs", "binary", "host-root", "mount-root", "output-dir"):
        n.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("inputs", "scores", "screen", "native-binary",
                 "collision-trial", "goal-trial", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "native": native, "evaluate": evaluate}[args.command](args)
