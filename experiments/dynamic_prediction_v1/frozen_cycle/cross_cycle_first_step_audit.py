#!/usr/bin/env python3
"""Native first-step audit of baseline, mixed and aligned outputs at 147/263."""
import argparse
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
from native_critic_sensitivity import AXES, aggregate
from output_first_step_audit import pose_array, actual
import replay_ranking


CASES = (("early_147", 147, "collision_trial", "score147"),
         ("goal_263", 263, "goal_trial", "score263"))
LABELS = ("baseline", "mixed", "aligned")


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    rows = []
    for name, cycle_id, trial_key, score_key in CASES:
        trial = getattr(args, trial_key)
        score_path = getattr(args, score_key)
        cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
        profile_path = trial / "profile.yaml"
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        if (meta["cycle_id"], settings["batch"], settings["steps"]) != (
                cycle_id, 300, 30):
            raise ValueError(f"frozen settings differ: {name}")
        profile = yaml.safe_load(profile_path.read_text())
        controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                             for axis in AXES], axis=-1)
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1)
        previous = replay_ranking.history_from_trial(
            trial / "mppi_cycles", cycle_id, arrays)
        history = np.stack([previous[axis] for axis in AXES], axis=-1)
        filtered = filtered_controls(controls, settings, history)
        poses = filtered_poses(controls, meta, settings, history)
        params = profile["controller_server"]["ros__parameters"][
            "FollowPath"]["PredictionV1Critic"]
        prediction, _, _ = filtered_prediction_score(meta, poses, params)
        standard = np.fromfile(score_path, dtype="<f4")
        if standard.shape != (300,) or not np.isfinite(standard).all():
            raise ValueError(f"native standard scores differ: {name}")
        ranked = aggregate(standard + prediction, controls, initial,
                           settings, history)
        baseline = np.stack([analyze.last(arrays, "after_filter." + axis)
                             for axis in AXES], axis=-1)
        aligned = np.einsum("i,ijk->jk", ranked["probability"], filtered,
                            dtype=np.float32)
        sequences = np.asarray((baseline, ranked["filtered_sequence"], aligned),
                               dtype="<f4")
        target = args.output_dir / name
        target.mkdir()
        control_path = target / "controls.bin"
        control_path.write_bytes(struct.pack("<3I", 0x43545231, 3, 30) +
                                 sequences.tobytes())
        python_pose = target / "python_native_first_poses.bin"
        pose_array(sequences, meta, settings, True).tofile(python_pose)
        map_path = target / "raw_map.bin"
        empty = np.zeros((3, 30), dtype=np.float32)
        export(map_path, meta, analyze.last(arrays, "locked.raw_map"),
               (empty, empty, empty), shortcut_threshold(meta, profile_path))
        fixture_meta = json.loads((args.cross_inputs / name / "meta.json")
                                  .read_text())
        fixture_meta["batch"] = 3
        fixture_meta["parameters"]["FollowPath.batch_size"] = 3
        fixture_meta["initial_controls"] = initial.astype(float).tolist()
        fixture_meta["history_controls"] = history.astype(float).tolist()
        fixture_meta["prediction_input"] = analyze.event(meta, "prediction.input")
        # This replay is used only for native pose integration. The frozen
        # scorer supports legacy V1 alone; original candidate scores above
        # retain the trial's actual rank mode (graded at cycle 263).
        fixture_meta["pose_replay_original_collision_rank_mode"] = params.get(
            "collision_rank_mode", "legacy")
        fixture_meta["prediction_parameters"] = dict(
            params, collision_rank_mode="legacy")
        native_meta = target / "native_meta.json"
        native_meta.write_text(json.dumps(fixture_meta, indent=2,
                                          sort_keys=True) + "\n")
        rows.append({"name": name, "cycle_id": cycle_id,
                     "trial": str(trial),
                     "cycle_sha256": digest(cycle),
                     "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
                     "profile_sha256": digest(profile_path),
                     "truth_sha256": digest(trial / "gazebo_poses.jsonl"),
                     "standard_sha256": digest(score_path),
                     "files": {path.name: digest(path) for path in (
                         control_path, python_pose, map_path, native_meta)}})
    record = {"schema": "rm_dynamic_prediction/cross_cycle_first_step_inputs/v1",
              "cases": rows}
    (args.output_dir / "inputs.json").write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


def evaluate(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    inputs = json.loads(args.inputs.read_text())
    rows = []
    for case in inputs["cases"]:
        name, cycle_id = case["name"], case["cycle_id"]
        source = args.inputs.parent / name
        for filename, expected in case["files"].items():
            if digest(source / filename) != expected:
                raise ValueError(f"fixture changed: {name}, {filename}")
        trial = Path(case["trial"])
        cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
        if digest(cycle) != case["cycle_sha256"] or digest(
                cycle.with_suffix(".bin")) != case["cycle_bin_sha256"]:
            raise ValueError(f"frozen cycle changed: {name}")
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        native_path = args.native_root / name / "replay_poses.bin"
        native = np.fromfile(native_path, dtype="<f4").reshape(3, 30, 3)
        python = np.fromfile(source / "python_native_first_poses.bin",
                             dtype="<f4").reshape(3, 30, 3)
        parity = float(np.max(np.abs(native - python)))
        if parity > 1e-5:
            raise ValueError(f"native integration mismatch: {name}, {parity}")
        profile_path = trial / "profile.yaml"
        profile = yaml.safe_load(profile_path.read_text())
        replay_meta = json.loads((source / "native_meta.json").read_text())
        original_mode = profile["controller_server"]["ros__parameters"][
            "FollowPath"]["PredictionV1Critic"].get(
                "collision_rank_mode", "legacy")
        if (replay_meta["pose_replay_original_collision_rank_mode"] != original_mode
                or replay_meta["prediction_parameters"]["collision_rank_mode"]
                != "legacy"):
            raise ValueError(f"pose-only scorer mode mismatch: {name}")
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        truth = actual(meta, settings, trial / "gazebo_poses.jsonl")
        body_gap, padded_gap = geometry_labels(
            tuple(native[:, :, axis] for axis in range(3)), body,
            meta["padded_footprint"], truth)
        fixture = args.output.parent / f"{name}_static.bin"
        export(fixture, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(native[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        entry = {"name": name, "cycle_id": cycle_id,
                 "native_pose_sha256": digest(native_path),
                 "native_python_pose_max_abs_error": parity,
                 "static_fixture_sha256": digest(fixture),
                 "outputs": []}
        goal = (json.loads((trial / "runtime_audit.json").read_text())[
            "navigation_result"]["goal"] if cycle_id == 263 else None)
        for index, label in enumerate(LABELS):
            metric = {"name": label,
                      "body_min_gap_m": float(body_gap[index]),
                      "padded_min_gap_m": float(padded_gap[index]),
                      "dynamic_gate_met": bool(body_gap[index] >= .05 and
                                               padded_gap[index] > 0)}
            if goal is not None:
                metric["endpoint_goal_position_distance_m"] = float(np.hypot(
                    native[index, -1, 0] - goal[0],
                    native[index, -1, 1] - goal[1]))
            entry["outputs"].append(metric)
        rows.append(entry)
    report = {"schema": "rm_dynamic_prediction/cross_cycle_first_step_audit/v1",
              "scope": "Preselected cycles 147 and 263; baseline, filtered-candidate mixed aggregation and aligned aggregation. Same frozen candidate scores/controls; native C++ MPPI current-speed first-step pose integration checked against Python. The native replay scorer is forced to legacy mode solely to obtain poses; its scores are unused, and cycle 263 candidate scoring retains its recorded graded mode. Future truth labels only. Native static masks pending finalize.",
              "inputs_sha256": digest(args.inputs),
              "native_binary_sha256": digest(args.native_binary),
              "rows": rows}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


def finalize(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    report = json.loads(args.evaluated.read_text())
    for row in report["rows"]:
        name = row["name"]
        fixture = args.evaluated.parent / f"{name}_static.bin"
        if digest(fixture) != row["static_fixture_sha256"]:
            raise ValueError(f"static fixture changed: {name}")
        mask_path = args.mask_root / f"{name}_mask.txt"
        values = [int(value) for value in mask_path.read_text().split()]
        if len(values) != 3 or any(value not in (0, 1) for value in values):
            raise ValueError(f"native mask malformed: {name}")
        row["static_mask_sha256"] = digest(mask_path)
        for metric, value in zip(row["outputs"], values):
            metric["costcritic_collision"] = bool(value)
            metric["joint_gate_met"] = bool(
                metric["dynamic_gate_met"] and not value)
    report["scope"] = report["scope"].replace(
        "Native static masks pending finalize.",
        "Original raw-costmap native CostCritic checked on all six outputs.")
    report["evaluated_sha256"] = digest(args.evaluated)
    report["mask_binary_sha256"] = digest(args.mask_binary)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["rows"])}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("collision-trial", "goal-trial", "score147", "score263",
                 "cross-inputs", "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("inputs", "native-root", "native-binary", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    done = sub.add_parser("finalize")
    for name in ("evaluated", "mask-root", "mask-binary", "output"):
        done.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "evaluate": evaluate,
     "finalize": finalize}[args.command](args)
