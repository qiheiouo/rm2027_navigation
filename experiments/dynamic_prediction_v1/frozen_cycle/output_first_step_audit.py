#!/usr/bin/env python3
"""Check filtered MPPI outputs with Nav2's current-speed first-step model.

Prior output geometry integrated the filtered command at step one, while the
native MPPI rollout integrator uses the measured speed at step one. This
probe freezes the 17 previously scored outputs, checks native integration
parity, then checks truth geometry and the original raw local costmap.
"""
import argparse
import json
from pathlib import Path
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels
from costmap_mask_fixture import export, shortcut_threshold
from filtered_batch_candidate_rank import raw_controls
from native_critic_sensitivity import AXES, aggregate
import replay_ranking


def pose_array(controls, meta, settings, first_speed):
    result = []
    for sequence in controls:
        if first_speed:
            speed = np.asarray(meta["speed"], dtype=np.float32)
            velocities = np.concatenate((speed[None, :], sequence[:-1]),
                                        axis=0)
        else:
            velocities = sequence
        result.append(np.stack(analyze.integrate_omni(
            *(velocities[:, axis] for axis in range(3)),
            meta["pose"], settings["dt"]), axis=-1))
    return np.asarray(result, dtype="<f4")


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"]) != (162, 300, 30):
        raise ValueError("not frozen cycle 162")
    raw_inputs = json.loads(args.raw_inputs.read_text())
    raw_rows = {row["name"]: row for row in raw_inputs["cases"]}
    aligned = json.loads(args.aligned_summary.read_text())
    risk_rows = {row["name"]: row for row in aligned["rows"]}
    all_critic = json.loads(args.all_critic_summary.read_text())
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    previous = replay_ranking.history_from_trial(
        args.cycle.parent, 162, arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1)
    unit = np.float32((3.81 / 254.) * 1_000_000. / 9)
    sequences = []
    rows = []
    for row in all_critic["rows"]:
        name, batch = row["name"], row["batch"]
        raw = raw_rows[name]
        risk = risk_rows[name]
        controls_path = Path(raw["controls"])
        risk_path = args.risk_dir / risk["filtered_risk_file"]
        score_path = args.score_dir / f"{name}_scores.bin"
        for path, expected in ((controls_path, raw["sha256"]["controls"]),
                               (risk_path, risk["filtered_risk_sha256"]),
                               (score_path, row[
                                   "filtered_native_score_sha256"])):
            if digest(path) != expected:
                raise ValueError(f"source changed: {name}, {path}")
        controls = raw_controls(controls_path, batch)
        standard = np.fromfile(score_path, dtype="<f4")
        overlap = np.fromfile(risk_path, dtype="<f8")[:batch]
        if standard.shape != overlap.shape or standard.shape != (batch,):
            raise ValueError(f"score dimensions differ: {name}")
        result = aggregate(standard + np.float32(unit * overlap),
                           controls, initial, settings, history)
        error = float(np.max(np.abs(np.asarray(result["returned_control"]) -
                                    row["all_critic_returned_control"])))
        if error > 1e-5:
            raise ValueError(f"published control differs: {name}")
        sequences.append(result["filtered_sequence"])
        rows.append({"name": name, "batch": batch,
                     "published_control_max_abs_error": error,
                     "published_direct_body_gap_m": row[
                         "all_critic_geometry"]["body_min_gap_m"],
                     "published_direct_padded_gap_m": row[
                         "all_critic_geometry"]["padded_min_gap_m"]})
    sequences = np.asarray(sequences, dtype="<f4")
    count = len(sequences)
    if count != 17:
        raise ValueError("expected 17 frozen batch outputs")
    controls_fixture = args.output_dir / "output_controls.bin"
    controls_fixture.write_bytes(struct.pack("<3I", 0x43545231, count, 30) +
                                 sequences.tobytes())
    native_first = args.output_dir / "python_native_first_poses.bin"
    direct = args.output_dir / "python_direct_poses.bin"
    pose_array(sequences, meta, settings, True).tofile(native_first)
    pose_array(sequences, meta, settings, False).tofile(direct)
    profile = yaml.safe_load(args.profile.read_text())
    map_fixture = args.output_dir / "raw_map.bin"
    empty = np.zeros((count, 30), dtype=np.float32)
    export(map_fixture, meta, analyze.last(arrays, "locked.raw_map"),
           (empty, empty, empty), shortcut_threshold(meta, args.profile))
    replay_meta = json.loads(args.replay_meta.read_text())
    replay_meta["batch"] = count
    replay_meta["parameters"]["FollowPath.batch_size"] = count
    meta_path = args.output_dir / "native_meta.json"
    meta_path.write_text(json.dumps(replay_meta, indent=2, sort_keys=True) + "\n")
    record = {"schema": "rm_dynamic_prediction/output_first_step_inputs/v1",
              "source_sha256": {"cycle": digest(args.cycle),
                                "cycle_bin": digest(args.cycle.with_suffix(".bin")),
                                "profile": digest(args.profile),
                                "truth": digest(args.truth),
                                "raw_inputs": digest(args.raw_inputs),
                                "aligned_summary": digest(args.aligned_summary),
                                "all_critic_summary": digest(args.all_critic_summary),
                                "replay_meta": digest(args.replay_meta)},
              "file_sha256": {path.name: digest(path) for path in (
                  controls_fixture, native_first, direct, map_fixture,
                  meta_path)},
              "rows": rows}
    (args.output_dir / "inputs.json").write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": count}))


def actual(meta, settings, truth_path):
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    stamp = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    return [analyze.placed(analyze.obstacle_polygon(),
            analyze.interpolated_pose(truth, times,
                                      stamp + (step + 1) * settings["dt"]))
            for step in range(settings["steps"])]


def evaluate(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    inputs = json.loads(args.inputs.read_text())
    for name, expected in inputs["file_sha256"].items():
        if digest(args.inputs.parent / name) != expected:
            raise ValueError(f"prepared input changed: {name}")
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    native = np.fromfile(args.native_poses, dtype="<f4").reshape(17, 30, 3)
    python = np.fromfile(args.inputs.parent / "python_native_first_poses.bin",
                         dtype="<f4").reshape(17, 30, 3)
    direct = np.fromfile(args.inputs.parent / "python_direct_poses.bin",
                         dtype="<f4").reshape(17, 30, 3)
    parity = float(np.max(np.abs(native - python)))
    if parity > 1e-5:
        raise ValueError(f"native integration mismatch: {parity}")
    profile = yaml.safe_load(args.profile.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    truth = actual(meta, settings, args.truth)
    geometries = []
    for poses in (direct, native):
        body_gap, padded_gap = geometry_labels(
            tuple(poses[:, :, axis] for axis in range(3)), body,
            meta["padded_footprint"], truth)
        geometries.append((body_gap, padded_gap))
    rows = []
    for index, source in enumerate(inputs["rows"]):
        direct_body, direct_padded = geometries[0]
        native_body, native_padded = geometries[1]
        if abs(direct_body[index] - source["published_direct_body_gap_m"]) > 1e-5 or \
                abs(direct_padded[index] - source[
                    "published_direct_padded_gap_m"]) > 1e-5:
            raise ValueError(f"published direct geometry differs: {source['name']}")
        rows.append({**source,
                     "direct_body_gap_m": float(direct_body[index]),
                     "direct_padded_gap_m": float(direct_padded[index]),
                     "native_first_body_gap_m": float(native_body[index]),
                     "native_first_padded_gap_m": float(native_padded[index]),
                     "native_first_dynamic_gate_met": bool(
                         native_body[index] >= .05 and native_padded[index] > 0)})
    export(args.static_fixture, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(native[:, :, axis] for axis in range(3)),
           shortcut_threshold(meta, args.profile))
    report = {"schema": "rm_dynamic_prediction/output_first_step_audit/v1",
              "scope": "Frozen cycle 162, 17 prior all-critic graded-V1 outputs. Direct-command output geometry compared with native MPPI current-measured-speed first-step geometry on identical filtered control sequences. Native C++ integration parity checked. Future Gazebo truth labels outputs only. Native static check pending finalize.",
              "inputs_sha256": digest(args.inputs),
              "native_replay_binary_sha256": digest(args.native_binary),
              "native_poses_sha256": digest(args.native_poses),
              "python_native_pose_max_abs_error": parity,
              "static_fixture_sha256": digest(args.static_fixture),
              "rows": rows}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"native_first_dynamic_pass": sum(
        row["native_first_dynamic_gate_met"] for row in rows)}))


def finalize(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    report = json.loads(args.evaluated.read_text())
    if digest(args.static_fixture) != report["static_fixture_sha256"]:
        raise ValueError("static fixture changed")
    mask_values = [int(value) for value in args.mask.read_text().split()]
    if len(mask_values) != 17 or any(value not in (0, 1)
                                     for value in mask_values):
        raise ValueError("native static mask malformed")
    for row, collision in zip(report["rows"], mask_values):
        row["native_first_costcritic_collision"] = bool(collision)
        row["native_first_joint_gate_met"] = bool(
            row["native_first_dynamic_gate_met"] and not collision)
    report["scope"] = report["scope"].replace(
        "Native static check pending finalize.",
        "Native raw-costmap CostCritic checked on each C++ integrated output.")
    report["evaluated_sha256"] = digest(args.evaluated)
    report["native_mask_binary_sha256"] = digest(args.mask_binary)
    report["native_mask_sha256"] = digest(args.mask)
    report["native_first_joint_pass_count"] = sum(
        row["native_first_joint_gate_met"] for row in report["rows"])
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"joint_pass": report["native_first_joint_pass_count"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("cycle", "profile", "truth", "raw-inputs", "aligned-summary",
                 "all-critic-summary", "risk-dir", "score-dir", "replay-meta",
                 "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("cycle", "profile", "truth", "inputs", "native-poses",
                 "native-binary", "static-fixture", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    done = sub.add_parser("finalize")
    for name in ("evaluated", "static-fixture", "mask", "mask-binary",
                 "output"):
        done.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "evaluate": evaluate,
     "finalize": finalize}[args.command](args)
