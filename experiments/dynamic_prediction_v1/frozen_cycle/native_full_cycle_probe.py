#!/usr/bin/env python3
"""Prepare, run, and validate native frozen MPPI integration/aggregation.

Captured or fixed sampled controls replace the noise generator in this one
cycle replay. Frozen accepted prediction input drives a mirror of the V1
geometry score; the ROS subscription plugin itself is not invoked.
"""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, sample_omni
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry


BATCHES = (300, 600, 1000, 2000)
OUTPUT_SUFFIXES = ("scores", "prediction_scores", "full_scores", "poses",
                   "before_filter", "after_filter")


def source_cases(captured, sensitivity):
    yield "captured_batch300", 300, captured, -1
    for seed in range(4):
        for batch in BATCHES:
            yield (f"seed_{seed}_batch{batch}", batch,
                   sensitivity / f"seed{seed}_batch{batch}", seed)


def prepare(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"],
            settings["iterations"]) != (162, 300, 30, 1):
        raise ValueError("not the fixed cycle 162 profile")
    history = json.loads(args.history.read_text())
    if history["cycle_id"] != 162 or len(history["previous_outputs"]) != 4:
        raise ValueError("history does not match cycle 162")
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    prediction = analyze.event(meta, "prediction.input")
    if prediction["status"] != "accepted" or not prediction["complete"]:
        raise ValueError("frozen V1 prediction input is unavailable")
    profile = yaml.safe_load(args.profile.read_text())
    prediction_parameters = profile["controller_server"]["ros__parameters"][
        "FollowPath"]["PredictionV1Critic"]
    args.output.mkdir(parents=True)
    records = []
    for name, batch, source, seed in source_cases(args.captured, args.sensitivity):
        original_meta = source / "meta.json"
        payload = json.loads(original_meta.read_text())
        if payload["batch"] != batch:
            raise ValueError(f"fixture batch differs: {original_meta}")
        payload["initial_controls"] = initial.astype(float).tolist()
        payload["history_controls"] = history["previous_outputs"]
        payload["prediction_input"] = prediction
        payload["prediction_parameters"] = prediction_parameters
        target = args.output / f"{name}_meta.json"
        target.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        map_file = source / payload["map_data_file"]
        controls = source / "controls.bin"
        records.append({
            "name": name, "seed": seed, "batch": batch,
            "meta": str(target.resolve()), "map": str(map_file.resolve()),
            "controls": str(controls.resolve()),
            "sha256": {"original_meta": digest(original_meta),
                       "replay_meta": digest(target), "map": digest(map_file),
                       "controls": digest(controls)},
        })
    result = {"schema": "rm_dynamic_prediction/native_full_cycle_inputs/v1",
              "cycle_sha256": digest(args.cycle),
              "history_sha256": digest(args.history),
              "profile_sha256": digest(args.profile), "cases": records}
    (args.output / "inputs.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(records), "output": str(args.output)}))


def native(args):
    inputs = json.loads(args.inputs.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    for row in inputs["cases"]:
        def mapped(key):
            host = Path(row[key])
            path = args.mount_root / host.relative_to(args.host_root)
            if digest(path) != row["sha256"]["replay_meta" if key == "meta" else key]:
                raise ValueError(f"fixture changed: {path}")
            return path
        prefix = args.output / row["name"]
        completed = subprocess.run(
            [str(args.binary), str(mapped("meta")), str(mapped("map")),
             str(mapped("controls")), str(prefix)],
            capture_output=True, text=True, check=False)
        (args.output / f"{row['name']}_log.txt").write_text(
            completed.stdout + completed.stderr)
        if completed.returncode:
            raise RuntimeError(f"{row['name']}: {completed.stderr[-1000:]}")
    print(json.dumps({"cases": len(inputs["cases"])}))


def validate(args):
    inputs = json.loads(args.inputs.read_text())
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    guarded = json.loads(args.guarded_summary.read_text())
    guarded_rows = {row["name"]: row for row in guarded["rows"]}
    history = np.asarray(json.loads(args.history.read_text())["previous_outputs"],
                         dtype=np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    profile = yaml.safe_load(args.profile.read_text())
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    padded = meta["padded_footprint"]
    truth = analyze.rows_from_transport(args.truth)
    times = [row["t"] for row in truth]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, times,
                  consumed + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    cached_seed = None
    cached_controls = cached_poses = None
    rows = []
    for case in inputs["cases"]:
        name, batch, seed = case["name"], case["batch"], case["seed"]
        if seed == -1:
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
            reference_poses = np.stack([analyze.last(arrays, "rollout." + axis)
                                        for axis in ("x", "y", "yaw")], axis=-1)
        else:
            if cached_seed != seed:
                sampled, trajectories = sample_omni(meta, arrays, 2000, seed)
                cached_controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
                cached_poses = np.stack(trajectories, axis=-1)
                cached_seed = seed
            controls = cached_controls[:batch]
            reference_poses = cached_poses[:batch]
        prefix = args.native_output / name
        poses = np.fromfile(str(prefix) + "_poses.bin", dtype="<f4").reshape(batch, 30, 3)
        scores = np.fromfile(str(prefix) + "_scores.bin", dtype="<f4")
        prediction_scores = np.fromfile(
            str(prefix) + "_prediction_scores.bin", dtype="<f4")
        full_scores = np.fromfile(str(prefix) + "_full_scores.bin", dtype="<f4")
        before = np.fromfile(str(prefix) + "_before_filter.bin", dtype="<f4").reshape(30, 3)
        after = np.fromfile(str(prefix) + "_after_filter.bin", dtype="<f4").reshape(30, 3)
        reference_scores_path = args.reference_scores / f"{name}_cpp_scores.bin"
        reference_scores = np.fromfile(reference_scores_path, dtype="<f4")
        if any(value.shape != (batch,) for value in (
                scores, prediction_scores, full_scores, reference_scores)):
            raise ValueError(f"score count differs: {name}")
        if float(np.ptp(prediction_scores)) > 1e-4 or \
                abs(float(prediction_scores[0]) - (3.81 / 254.0 * 1e6 / 9)) > 1e-3:
            raise AssertionError(f"V1 first-step tie changed: {name}")
        predicted = aggregate(full_scores, controls, initial, settings, history)
        errors = {
            "poses_max_abs": float(np.max(np.abs(poses - reference_poses))),
            "scores_max_abs": float(np.max(np.abs(scores - reference_scores))),
            "prediction_score_span": float(np.ptp(prediction_scores)),
            "before_filter_max_abs": float(np.max(np.abs(
                before - predicted["unfiltered_sequence"]))),
            "after_filter_max_abs": float(np.max(np.abs(
                after - predicted["filtered_sequence"]))),
        }
        if (errors["poses_max_abs"] > 1e-5 or
                errors["scores_max_abs"] > 1e-3 or
                errors["before_filter_max_abs"] > 5e-5 or
                errors["after_filter_max_abs"] > 5e-5):
            raise AssertionError(f"native replay differs: {name}: {errors}")
        if seed == -1:
            captured_increment = (analyze.last(arrays,
                "critic.FollowPath.PredictionV1Critic") - analyze.last(arrays,
                "critic.FollowPath.PathAngleCritic"))
            errors["prediction_vs_captured_increment_max_abs"] = float(
                np.max(np.abs(prediction_scores - captured_increment)))
            if errors["prediction_vs_captured_increment_max_abs"] > 1e-3:
                raise AssertionError("V1 geometry replay differs from captured critic")
        geometry = open_loop_geometry(after, meta, settings, actual, body, padded)
        reference_geometry = guarded_rows[name]["guarded_filtered_geometry"]
        if abs(geometry["body_min_gap_m"] - reference_geometry["body_min_gap_m"]) > 1e-4:
            raise AssertionError(f"native aggregate geometry differs: {name}")
        files = {suffix: digest(Path(str(prefix) + f"_{suffix}.bin"))
                 for suffix in OUTPUT_SUFFIXES}
        timing_path = Path(str(prefix) + "_timing.json")
        rows.append({"name": name, "batch": batch, "errors": errors,
                     "native_file_sha256": files,
                     "timing": json.loads(timing_path.read_text()),
                     "timing_sha256": digest(timing_path),
                     "filtered_geometry": geometry,
                     "returned_control": after[settings["offset"]].astype(float).tolist()})
    output = {
        "schema": "rm_dynamic_prediction/native_full_cycle_replay/v1",
        "scope": "Frozen controls injected after noise generation; native Omni dynamics, seven guarded standard critics, V1 legacy geometry arithmetic mirrored from the current source with frozen accepted input and age, regularized softmax, constraints and Savitzky-Golay filter. V1 ROS subscription and noise generation are bypassed. One warm replay per case; not a full controller/ROS performance benchmark.",
        "environment": {
            "image_id": args.image_id,
            "guarded_source_manifest_sha256": digest(args.source_manifest),
            "guarded_source_manifest": json.loads(args.source_manifest.read_text()),
            "guarded_critics_plugin_sha256": digest(args.guarded_plugin),
            "prediction_geometry_header_sha256": digest(args.geometry_header),
            "prediction_critic_source_sha256": digest(args.prediction_critic_source),
            "single_worker_build_log_sha256": digest(args.build_log),
            "build_parallel_workers": 1,
            "cmake_build_parallel_level": 1,
        },
        "input_sha256": {"inputs": digest(args.inputs), "cycle": digest(args.cycle),
                         "history": digest(args.history), "profile": digest(args.profile),
                         "truth": digest(args.truth),
                         "guarded_summary": digest(args.guarded_summary),
                         "binary": digest(args.binary)},
        "rows": rows,
        "dynamic_gate_pass_count": sum(row["filtered_geometry"]["dynamic_clearance_gate_met"]
                                       for row in rows),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "dynamic_gate_pass_count":
                      output["dynamic_gate_pass_count"],
                      "max_errors": {key: max(row["errors"][key] for row in rows
                                              if key in row["errors"])
                                     for key in rows[0]["errors"]}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for flag in ("cycle", "history", "profile", "captured", "sensitivity", "output"):
        prep.add_argument("--" + flag, type=Path, required=True)
    replay = sub.add_parser("native")
    for flag in ("inputs", "binary", "host-root", "mount-root", "output"):
        replay.add_argument("--" + flag, type=Path, required=True)
    check = sub.add_parser("validate")
    for flag in ("inputs", "cycle", "history", "profile", "truth", "native-output",
                 "reference-scores", "guarded-summary", "binary", "output",
                 "source-manifest", "guarded-plugin", "build-log",
                 "geometry-header", "prediction-critic-source"):
        check.add_argument("--" + flag, type=Path, required=True)
    check.add_argument("--image-id", required=True)
    args = parser.parse_args()
    {"prepare": prepare, "native": native, "validate": validate}[args.command](args)
