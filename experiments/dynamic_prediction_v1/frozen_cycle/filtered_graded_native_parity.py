#!/usr/bin/env python3
"""Prepare and validate native integration of individually filtered controls."""
import argparse
import json
from pathlib import Path
import struct

import numpy as np

import analyze
from batch_sampling_probe import digest, sample_omni
from filtered_graded_batch_probe import filtered_poses
from native_critic_sensitivity import AXES
import replay_ranking


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["steps"]) != (162, 30):
        raise ValueError("not the frozen cycle 162")
    history_record = json.loads(args.history.read_text())
    if history_record["cycle_id"] != 162:
        raise ValueError("history belongs to another cycle")
    history = np.asarray(history_record["previous_outputs"], dtype=np.float32)
    sampled, _ = sample_omni(meta, arrays, args.batch, args.seed)
    controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
    limits = ((settings["vx_min"], settings["vx_max"]),
              (-settings["vy_max"], settings["vy_max"]),
              (-settings["wz_max"], settings["wz_max"]))
    filtered = np.stack([np.stack([replay_ranking.smooth_axis(
        np.clip(control[:, axis], *limits[axis]), history[:, axis])
        for axis in range(3)], axis=-1) for control in controls])
    poses = filtered_poses(controls, meta, settings, history)
    args.output_dir.mkdir(parents=True)
    control_file = args.output_dir / "controls.bin"
    pose_file = args.output_dir / "python_poses.bin"
    control_file.write_bytes(struct.pack("<3I", 0x43545231, args.batch, 30) +
                             filtered.astype("<f4").tobytes())
    poses.astype("<f4").tofile(pose_file)
    record = {"schema": "rm_dynamic_prediction/filtered_native_parity_inputs/v1",
              "cycle_sha256": digest(args.cycle),
              "history_sha256": digest(args.history),
              "seed": args.seed, "batch": args.batch,
              "controls_sha256": digest(control_file),
              "python_poses_sha256": digest(pose_file)}
    (args.output_dir / "inputs.json").write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps(record))


def validate(args):
    inputs = json.loads((args.output_dir / "inputs.json").read_text())
    for name, key in (("controls.bin", "controls_sha256"),
                      ("python_poses.bin", "python_poses_sha256")):
        if digest(args.output_dir / name) != inputs[key]:
            raise ValueError(f"prepared file changed: {name}")
    shape = (inputs["batch"], 30, 3)
    predicted = np.fromfile(args.output_dir / "python_poses.bin",
                            dtype="<f4").reshape(shape)
    native = np.fromfile(args.native_poses, dtype="<f4").reshape(shape)
    error = np.abs(predicted - native)
    if float(error.max()) > 1e-5:
        raise AssertionError("filtered candidate integration differs from native")
    report = {"schema": "rm_dynamic_prediction/filtered_native_parity/v1",
              "inputs": inputs,
              "native_binary_sha256": digest(args.native_binary),
              "native_poses_sha256": digest(args.native_poses),
              "max_abs_pose_error": float(error.max()),
              "pose_components": int(error.size),
              "count_above_1e-5": int(np.count_nonzero(error > 1e-5))}
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    a = sub.add_parser("prepare")
    a.add_argument("--cycle", type=Path, required=True)
    a.add_argument("--history", type=Path, required=True)
    a.add_argument("--seed", type=int, default=2)
    a.add_argument("--batch", type=int, default=2000)
    a.add_argument("--output-dir", type=Path, required=True)
    b = sub.add_parser("validate")
    b.add_argument("--output-dir", type=Path, required=True)
    b.add_argument("--native-poses", type=Path, required=True)
    b.add_argument("--native-binary", type=Path, required=True)
    b.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "validate": validate}[args.command](args)
