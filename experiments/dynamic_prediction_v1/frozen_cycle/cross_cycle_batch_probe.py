#!/usr/bin/env python3
"""Nested-batch diagnostic on the preselected 147 and 263 frozen inputs.

The installed native seven-critic scorer consumes individually filtered
candidate poses/controls. Gazebo truth is only read by the evaluation phase.
No runtime controller or prediction parameter is changed.
"""
import argparse
import json
import math
from pathlib import Path
import re
import struct
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_all_critic_probe import filtered_controls
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_fixture import costmap_parameters, parameters
from native_critic_sensitivity import aggregate
from output_first_step_audit import pose_array
import replay_ranking


AXES = ("vx", "vy", "wz")
CASES = (("early_147", 147, "collision_trial"),
         ("goal_263", 263, "goal_trial"))
SEEDS = range(4)


def inputs_for(trial, cycle_id):
    cycle = trial / "mppi_cycles" / f"cycle_{cycle_id}.json"
    profile_path = trial / "profile.yaml"
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"],
            settings["iterations"], settings["offset"]) != (cycle_id, 300, 30, 1, 1):
        raise ValueError("frozen MPPI settings changed")
    profile = yaml.safe_load(profile_path.read_text())
    history_axes = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    history = np.stack([history_axes[axis] for axis in AXES], axis=-1)
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    return cycle, profile_path, meta, arrays, settings, profile, history, params


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    rows = []
    for name, cycle_id, key in CASES:
        trial = getattr(args, key)
        cycle, profile_path, meta, arrays, settings, profile, history, params = \
            inputs_for(trial, cycle_id)
        raw_map = analyze.last(arrays, "locked.raw_map")
        threshold = shortcut_threshold(meta, profile_path)
        for seed in SEEDS:
            target = args.output_dir / f"{name}_seed{seed}"
            target.mkdir()
            sampled, _ = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
            filtered = filtered_controls(controls, settings, history)
            poses = filtered_poses(controls, meta, settings, history)
            prediction, hits, _ = filtered_prediction_score(meta, poses, params)
            map_path = target / "map_and_poses.bin"
            control_path = target / "controls.bin"
            meta_path = target / "meta.json"
            prediction_path = target / "prediction.bin"
            export(map_path, meta, raw_map,
                   tuple(poses[:, :, axis] for axis in range(3)), threshold)
            control_path.write_bytes(struct.pack("<3I", 0x43545231, 2000, 30) +
                                     filtered.astype("<f4").tobytes())
            prediction.astype("<f4").tofile(prediction_path)
            fixture_meta = {"pose": meta["pose"], "speed": meta["speed"],
                            "path": meta["path"],
                            "map_frame": meta["map"]["frame"],
                            "parameters": parameters(profile, 2000),
                            "costmap_parameters": costmap_parameters(profile),
                            "batch": 2000, "map_data_file": map_path.name}
            meta_path.write_text(json.dumps(fixture_meta, indent=2,
                                            sort_keys=True) + "\n")
            row = {"name": name, "cycle_id": cycle_id, "seed": seed,
                   "cycle_sha256": digest(cycle),
                   "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
                   "profile_sha256": digest(profile_path),
                   "truth_sha256": digest(trial / "gazebo_poses.jsonl"),
                   "prediction_hits_by_step": hits}
            for key, path in (("map", map_path), ("controls", control_path),
                              ("meta", meta_path), ("prediction", prediction_path)):
                row[key] = str(path.resolve())
                row[key + "_sha256"] = digest(path)
            rows.append(row)
    (args.output_dir / "inputs.json").write_text(json.dumps({
        "schema": "rm_dynamic_prediction/cross_cycle_batch_inputs/v1",
        "cases": rows}, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "output": str(args.output_dir)}))


def prefixes(args):
    """Materialize exact nested prefixes for batch-dependent native critics."""
    input_path = args.input_dir / "inputs.json"
    inputs = json.loads(input_path.read_text())
    if inputs.get("prefixes"):
        raise ValueError("prefix fixtures already created")
    for row in inputs["cases"]:
        root = Path(row["meta"]).parent
        meta = json.loads(Path(row["meta"]).read_text())
        control_data = Path(row["controls"]).read_bytes()
        if control_data[:12] != struct.pack("<3I", 0x43545231, 2000, 30):
            raise ValueError("2000 control header differs")
        row["prefixes"] = {}
        for batch in BATCHES:
            path_meta = root / f"meta_{batch}.json"
            path_controls = root / f"controls_{batch}.bin"
            native_meta = json.loads(json.dumps(meta))
            native_meta["batch"] = batch
            native_meta["parameters"]["FollowPath.batch_size"] = batch
            path_meta.write_text(json.dumps(native_meta, indent=2,
                                            sort_keys=True) + "\n")
            path_controls.write_bytes(
                struct.pack("<3I", 0x43545231, batch, 30) +
                control_data[12:12 + batch * 30 * 3 * 4])
            row["prefixes"][str(batch)] = {
                "meta": str(path_meta.resolve()),
                "meta_sha256": digest(path_meta),
                "controls": str(path_controls.resolve()),
                "controls_sha256": digest(path_controls)}
    inputs["prefixes"] = list(BATCHES)
    input_path.write_text(json.dumps(inputs, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(inputs["cases"]),
                      "prefixes": list(BATCHES)}))


def native(args):
    """Run installed scorers serially inside the pinned isolated container."""
    inputs = json.loads(args.inputs.read_text())
    if inputs.get("prefixes") != list(BATCHES):
        raise ValueError("nested batch fixtures absent")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    def mounted(host):
        return args.mount_root / Path(host).relative_to(args.host_root)

    for row in inputs["cases"]:
        for key in ("map", "controls", "meta", "prediction"):
            if digest(mounted(row[key])) != row[key + "_sha256"]:
                raise ValueError(f"fixture changed: {row['name']} seed {row['seed']}")
        stem = f'{row["name"]}_seed{row["seed"]}'
        mask = args.output_dir / f"{stem}_mask.txt"
        subprocess.run([str(args.mask_binary), str(mounted(row["map"])),
                        str(mask)], check=True, capture_output=True, text=True)
        for batch in BATCHES:
            prefix = row["prefixes"][str(batch)]
            for key in ("meta", "controls"):
                if digest(mounted(prefix[key])) != prefix[key + "_sha256"]:
                    raise ValueError(f"prefix changed: {stem} batch {batch}")
            score = args.output_dir / f"{stem}_batch{batch}_scores.bin"
            done = subprocess.run([
                str(args.binary), str(mounted(prefix["meta"])),
                str(mounted(row["map"])), str(mounted(prefix["controls"])),
                str(score)], capture_output=True, text=True, check=False)
            (args.output_dir / f"{stem}_batch{batch}_log.txt").write_text(
                "\n".join(line.rstrip() for line in
                          (done.stdout + done.stderr).splitlines()) + "\n")
            if done.returncode:
                raise RuntimeError(f"{stem} batch {batch}: {done.stderr[-1000:]}")
        print(json.dumps({"completed": stem}), flush=True)


def native_output_masks(args):
    for name, _, _ in CASES:
        fixture = args.input_dir / f"{name}_output_static.bin"
        mask = args.input_dir / f"{name}_output_mask.txt"
        subprocess.run([str(args.mask_binary), str(fixture), str(mask)],
                       check=True, capture_output=True, text=True)
    print(json.dumps({"output_masks": len(CASES)}))


def first_predicted_conflict_steps(meta, poses, params):
    prediction = analyze.event(meta, "prediction.input")
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("single confirmed box fixture required")
    track = tracks[0]
    settings = analyze.event(meta, "settings")
    horizon_steps = int(math.floor(params["horizon"] / settings["dt"] + 1e-9))
    first = np.zeros(len(poses), dtype=np.int16)
    for step in range(horizon_steps):
        box = analyze.predicted_box(
            track["xy"], track["vxy"], track["size_xy"],
            (params["object_width"], params["object_height"]),
            prediction["source_age_s"], (step + 1) * settings["dt"],
            params["reference_acceleration"]).polygon()
        for index in range(len(poses)):
            if first[index]:
                continue
            robot = analyze.placed(meta["padded_footprint"],
                                   tuple(float(x) for x in poses[index, step]))
            if analyze.polygon_distance(robot, box) <= 1e-9:
                first[index] = step + 1
    return first


def evaluate(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    inputs = json.loads(args.inputs.read_text())
    records = []
    output_poses = {name: [] for name, _, _ in CASES}
    for name, cycle_id, key in CASES:
        trial = getattr(args, key)
        cycle, profile_path, meta, arrays, settings, profile, history, params = \
            inputs_for(trial, cycle_id)
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1)
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        truth_path = trial / "gazebo_poses.jsonl"
        truth = analyze.rows_from_transport(truth_path)
        times = [row["t"] for row in truth]
        consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
        actual = [analyze.placed(analyze.obstacle_polygon(),
            analyze.interpolated_pose(truth, times,
                consumed + (step + 1) * settings["dt"]))
            for step in range(settings["steps"])]
        goal = json.loads((trial / "runtime_audit.json").read_text())[
            "navigation_result"]["goal"] if cycle_id == 263 else None
        for seed in SEEDS:
            source = next(row for row in inputs["cases"] if
                          (row["name"], row["seed"]) == (name, seed))
            for path_key in ("map", "controls", "meta", "prediction"):
                if digest(Path(source[path_key])) != source[path_key + "_sha256"]:
                    raise ValueError(f"fixture changed: {name} seed {seed}")
            if (digest(cycle), digest(cycle.with_suffix(".bin")),
                digest(profile_path), digest(truth_path)) != tuple(source[k] for k
                    in ("cycle_sha256", "cycle_bin_sha256", "profile_sha256",
                        "truth_sha256")):
                raise ValueError("frozen source changed")
            sampled, _ = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
            poses = filtered_poses(controls, meta, settings, history)
            prediction, _, _ = filtered_prediction_score(meta, poses, params)
            saved_prediction = np.fromfile(source["prediction"], dtype="<f4")
            if not np.array_equal(prediction, saved_prediction):
                raise ValueError("V1 risk changed")
            static_path = args.masks / f"{name}_seed{seed}_mask.txt"
            static = np.loadtxt(static_path, dtype=bool)
            if static.shape != (2000,):
                raise ValueError("native static mask count differs")
            body_gap, padded_gap = geometry_labels(
                tuple(poses[:, :, axis] for axis in range(3)),
                body, meta["padded_footprint"], actual)
            safe = (body_gap >= .05) & (padded_gap > 0) & ~static
            first_conflict = first_predicted_conflict_steps(
                meta, poses, params)
            metrics_path = args.output.parent / f"{name}_seed{seed}_candidate_metrics.npz"
            np.savez_compressed(metrics_path,
                                body_gap_m=body_gap, padded_gap_m=padded_gap,
                                static_collision=static,
                                prediction_score=prediction,
                                first_predicted_conflict_step=first_conflict)
            goal_mask = (np.hypot(poses[:, -1, 0] - goal[0],
                                  poses[:, -1, 1] - goal[1]) <= .15
                         if goal is not None else None)
            for batch in BATCHES:
                prefix = source["prefixes"][str(batch)]
                for key in ("meta", "controls"):
                    if digest(Path(prefix[key])) != prefix[key + "_sha256"]:
                        raise ValueError("prefix fixture changed")
                score_path = args.scores / f"{name}_seed{seed}_batch{batch}_scores.bin"
                standard = np.fromfile(score_path, dtype="<f4")
                if standard.shape != (batch,):
                    raise ValueError("native standard score count differs")
                timing = (args.scores /
                          f"{name}_seed{seed}_batch{batch}_log.txt").read_text()
                match = re.search(r"^score_eval_ms=([0-9.]+)$", timing, re.M)
                if match is None:
                    raise ValueError("native score timing missing")
                result = aggregate(standard + prediction[:batch],
                                   controls[:batch], initial, settings, history)
                weighted = result["weighted"]
                order = np.argsort(weighted)
                safe_indexes = np.flatnonzero(safe[:batch])
                output_pose = pose_array(
                    result["filtered_sequence"][None, :, :],
                    meta, settings, True)[0]
                output_poses[name].append(output_pose)
                out_body, out_pad = geometry_labels(
                    tuple(output_pose[None, :, axis] for axis in range(3)),
                    body, meta["padded_footprint"], actual)
                record = {"name": name, "cycle_id": cycle_id,
                          "seed": seed, "batch": batch,
                          "candidate_metrics_sha256": digest(metrics_path),
                          "native_score_sha256": digest(score_path),
                          "native_score_eval_ms": float(match.group(1)),
                          "native_mask_sha256": digest(static_path),
                          "joint_safe_candidate_count": int(np.sum(safe[:batch])),
                          "predicted_conflict_candidate_count": int(np.count_nonzero(
                              first_conflict[:batch])),
                          "joint_safe_with_predicted_conflict_count": int(
                              np.count_nonzero(safe[:batch] &
                                               (first_conflict[:batch] > 0))),
                          "best_candidate_body_gap_m": float(np.max(body_gap[:batch])),
                          "minimum_score_candidate_joint_safe": bool(safe[order[0]]),
                          "minimum_score_candidate_body_gap_m": float(
                              body_gap[order[0]]),
                          "minimum_score_candidate_predicted_conflict_step": int(
                              first_conflict[order[0]]),
                          "top_10_joint_safe_count": int(np.sum(safe[order[:10]])),
                          "best_joint_safe_rank": int(np.flatnonzero(np.isin(
                              order, safe_indexes))[0] + 1) if len(safe_indexes)
                              else None,
                          "joint_safe_probability_mass": float(np.sum(
                              result["probability"][safe_indexes])),
                          "effective_sample_size": float(1. / np.sum(
                              result["probability"] ** 2)),
                          "returned_control": result["returned_control"],
                          "output_body_gap_m": float(out_body[0]),
                          "output_padded_gap_m": float(out_pad[0]),
                          "output_body_gap_step": int(np.argmin([
                              analyze.polygon_distance(
                                  analyze.placed(body, tuple(output_pose[s])),
                                  actual[s]) for s in range(30)]) + 1)}
                if goal is not None:
                    record["filtered_goal_candidate_count"] = int(
                        np.sum(goal_mask[:batch]))
                    record["filtered_goal_candidate_probability_mass"] = float(
                        np.sum(result["probability"][goal_mask[:batch]]))
                    record["output_goal_position_distance_m"] = float(np.hypot(
                        output_pose[-1, 0] - goal[0],
                        output_pose[-1, 1] - goal[1]))
                records.append(record)
    fixture_hashes = {}
    for name, cycle_id, key in CASES:
        trial = getattr(args, key)
        meta, arrays = analyze.read_cycle(
            trial / "mppi_cycles" / f"cycle_{cycle_id}.json")
        poses = np.stack(output_poses[name])
        target = args.output.parent / f"{name}_output_static.bin"
        export_path = (target.with_name(target.stem + "_verify.bin")
                       if target.exists() else target)
        export(export_path, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(poses[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, trial / "profile.yaml"))
        if export_path != target:
            if digest(export_path) != digest(target):
                raise ValueError("existing output static fixture differs")
            export_path.unlink()
        fixture_hashes[name] = digest(target)
    report = {"schema": "rm_dynamic_prediction/cross_cycle_batch_probe/v1",
              "scope": "Fixed 147/263 inputs, four deterministic independent seeds and nested 300/600/1000/2000 Gaussian samples. Each sampled control individually filtered before original seven native standard critics and existing experimental V1 hard+uniform-overlap score. Raw controls are aggregated and finally filtered. Truth is used only to label candidate and output 3 s geometry. Original raw map is checked by installed Nav2 CostCritic mask tool. Native timing covers seven-critic evaluation only, not a complete ROS cycle. Exploratory sample generator differs from native RNG; no runtime change.",
              "inputs_sha256": digest(args.inputs),
              "native_binary_sha256": digest(args.native_binary),
              "native_mask_binary_sha256": digest(args.mask_binary),
              "output_static_fixture_sha256": fixture_hashes,
              "rows": records}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"rows": len(records), "output": str(args.output)}))


def finalize(args):
    report = json.loads(args.report.read_text())
    if "output_static_collision" in report["rows"][0]:
        raise ValueError("report already finalized")
    for name, _, _ in CASES:
        fixture = args.report.parent / f"{name}_output_static.bin"
        if digest(fixture) != report["output_static_fixture_sha256"][name]:
            raise ValueError("output static fixture changed")
        mask_path = args.masks / f"{name}_output_mask.txt"
        mask = np.loadtxt(mask_path, dtype=bool)
        rows = [row for row in report["rows"] if row["name"] == name]
        if mask.shape != (len(rows),):
            raise ValueError("output static mask size differs")
        for row, collision in zip(rows, mask):
            row["output_static_collision"] = bool(collision)
            row["output_joint_gate_met"] = bool(
                row["output_body_gap_m"] >= .05 and
                row["output_padded_gap_m"] > 0 and not collision)
    report["output_static_masks_sha256"] = {
        name: digest(args.masks / f"{name}_output_mask.txt")
        for name, _, _ in CASES}
    report["joint_gate_pass_count"] = sum(
        row["output_joint_gate_met"] for row in report["rows"])
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"joint_gate_pass_count": report[
        "joint_gate_pass_count"], "rows": len(report["rows"])}))


def window(args):
    """Separate the actual V1 nine-step window from the 30-step truth gate."""
    report = json.loads(args.report.read_text())
    if any("output_first_9_body_gap_m" in row for row in report["rows"]):
        raise ValueError("window audit already present")
    for name, cycle_id, key in CASES:
        trial = getattr(args, key)
        _, _, meta, arrays, settings, profile, _, params = inputs_for(
            trial, cycle_id)
        source = args.report.parent / f"{name}_output_static.bin"
        if digest(source) != report["output_static_fixture_sha256"][name]:
            raise ValueError("output pose fixture changed")
        payload = source.read_bytes()
        magic, width, height, count, steps, vertices, _ = struct.unpack_from(
            "<7I", payload)
        if (magic, count, steps) != (0x43504D31, 16, 30):
            raise ValueError("output pose fixture header differs")
        offset = struct.calcsize("<7I3d") + vertices * 16 + width * height
        poses = np.frombuffer(payload, dtype="<f4", offset=offset).reshape(
            count, steps, 3)
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        truth = analyze.rows_from_transport(trial / "gazebo_poses.jsonl")
        times = [row["t"] for row in truth]
        consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
        actual = [analyze.placed(analyze.obstacle_polygon(),
            analyze.interpolated_pose(truth, times,
                consumed + (step + 1) * settings["dt"]))
            for step in range(9)]
        body_gap, padded_gap = geometry_labels(
            tuple(poses[:, :9, axis] for axis in range(3)),
            body, meta["padded_footprint"], actual)
        first_conflict = first_predicted_conflict_steps(meta, poses, params)
        rows = [row for row in report["rows"] if row["name"] == name]
        for index, row in enumerate(rows):
            row["output_first_9_body_gap_m"] = float(body_gap[index])
            row["output_first_9_padded_gap_m"] = float(padded_gap[index])
            row["output_first_9_truth_gate_met"] = bool(
                body_gap[index] >= .05 and padded_gap[index] > 0)
            row["output_predicted_first_conflict_step"] = int(
                first_conflict[index])
    report["first_9_truth_gate_pass_count"] = sum(
        row["output_first_9_truth_gate_met"] for row in report["rows"])
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"first_9_truth_gate_pass_count": report[
        "first_9_truth_gate_pass_count"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("prepare", "evaluate"):
        p = sub.add_parser(command)
        p.add_argument("--collision-trial", type=Path, required=True)
        p.add_argument("--goal-trial", type=Path, required=True)
        if command == "prepare":
            p.add_argument("--output-dir", type=Path, required=True)
        else:
            for field in ("inputs", "scores", "masks", "native-binary",
                          "mask-binary", "output"):
                p.add_argument("--" + field, type=Path, required=True)
    p = sub.add_parser("prefixes")
    p.add_argument("--input-dir", type=Path, required=True)
    p = sub.add_parser("native")
    for field in ("inputs", "binary", "mask-binary", "host-root",
                  "mount-root", "output-dir"):
        p.add_argument("--" + field, type=Path, required=True)
    p = sub.add_parser("native-output-masks")
    p.add_argument("--input-dir", type=Path, required=True)
    p.add_argument("--mask-binary", type=Path, required=True)
    p = sub.add_parser("finalize")
    p.add_argument("--report", type=Path, required=True)
    p.add_argument("--masks", type=Path, required=True)
    p = sub.add_parser("window")
    p.add_argument("--report", type=Path, required=True)
    p.add_argument("--collision-trial", type=Path, required=True)
    p.add_argument("--goal-trial", type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "prefixes": prefixes, "native": native,
     "native-output-masks": native_output_masks, "evaluate": evaluate,
     "finalize": finalize, "window": window}[args.command](args)
