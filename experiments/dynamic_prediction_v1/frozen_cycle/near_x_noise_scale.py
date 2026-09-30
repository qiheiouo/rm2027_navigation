#!/usr/bin/env python3
"""Fixed-seed, single-variable Gaussian noise-scale audit for near-X cycle."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from near_x_batch_sensitivity import AXES, EXPECTED, SEEDS, frozen
import replay_ranking


SCALES = (2, 4)


def prepare(trial, baseline, output):
    if output.exists():
        raise FileExistsError(output)
    paths, meta, arrays, settings = frozen(trial)
    history_source = replay_ranking.history_from_trial(
        trial / "mppi_cycles", 73, arrays)
    history = np.stack([history_source[axis] for axis in AXES], axis=-1).astype(np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1).astype(np.float32)
    truth = analyze.rows_from_transport(paths["truth"])
    times = [row["t"] for row in truth]
    consumer = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, times,
                  consumer + (j + 1) * settings["dt"]))
              for j in range(settings["steps"])]
    profile = yaml.safe_load(paths["profile"].read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    raw = analyze.last(arrays, "locked.raw_map")
    threshold = shortcut_threshold(meta, paths["profile"])
    locked = json.loads((baseline / "inputs.json").read_text())
    if locked["inputs_sha256"] != EXPECTED:
        raise ValueError("baseline fixture differs")
    output.mkdir(parents=True)
    for seed in SEEDS:
        sampled, _ = sample_omni(meta, arrays, 2000, seed)
        original = np.stack([sampled[axis] for axis in AXES], axis=-1)
        base_poses = filtered_poses(original, meta, settings, history)
        baseline_check = output / f"seed_{seed}_baseline_check.bin"
        export(baseline_check, meta, raw,
               tuple(base_poses[:, :, axis] for axis in range(3)), threshold)
        if digest(baseline_check) != locked["files_sha256"][f"seed_{seed}_map.bin"]:
            raise ValueError("scale-one sampling does not reproduce baseline")
        baseline_check.unlink()
        for scale in SCALES:
            controls = initial[None, :, :] + \
                np.float32(scale) * (original - initial[None, :, :])
            poses = filtered_poses(controls, meta, settings, history)
            body_gap, padded_gap = geometry_labels(
                tuple(poses[:, :, axis] for axis in range(3)),
                body, meta["padded_footprint"], actual)
            stem = f"seed_{seed}_scale_{scale}"
            np.savez(output / f"{stem}_gaps.npz", body=body_gap,
                     padded=padded_gap)
            export(output / f"{stem}_map.bin", meta, raw,
                   tuple(poses[:, :, axis] for axis in range(3)), threshold)
    report = {"schema": "rm_dynamic_prediction_near_x_noise_inputs/v1",
              "inputs_sha256": EXPECTED,
              "baseline_inputs_sha256": digest(baseline / "inputs.json"),
              "seeds": SEEDS, "scales": SCALES, "batch": 2000,
              "history_sha256": locked["history_sha256"],
              "files_sha256": {p.name: digest(p) for p in sorted(output.iterdir())}}
    (output / "inputs.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"cases": len(SEEDS) * len(SCALES), "output": str(output)}))


def evaluate(trial, baseline, fixture, masks, output):
    if output.exists():
        raise FileExistsError(output)
    frozen(trial)
    record = json.loads((fixture / "inputs.json").read_text())
    if record["inputs_sha256"] != EXPECTED or \
            record["baseline_inputs_sha256"] != digest(baseline / "inputs.json") or \
            record["seeds"] != list(SEEDS) or record["scales"] != list(SCALES):
        raise ValueError("preregistered noise inputs differ")
    for name, sha in record["files_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError(f"noise fixture changed: {name}")
    for index, sha in record["history_sha256"].items():
        if digest(trial / f"mppi_cycles/cycle_{index}.json") != sha:
            raise ValueError("control history changed")
    rows = []
    for seed in SEEDS:
        for scale in SCALES:
            stem = f"seed_{seed}_scale_{scale}"
            with np.load(fixture / f"{stem}_gaps.npz") as gaps:
                body, padded = gaps["body"], gaps["padded"]
            mask_file = masks / f"{stem}_mask.txt"
            mask = np.loadtxt(mask_file, dtype=bool)
            if body.shape != (2000,) or padded.shape != body.shape or mask.shape != body.shape:
                raise ValueError(f"{stem}: output size differs")
            dynamic = (body >= .05) & (padded > 0)
            rows.append({"seed": seed, "noise_std_multiplier": scale,
                         "dynamic_safe_count": int(dynamic.sum()),
                         "native_static_collision_count": int(mask.sum()),
                         "joint_safe_count": int(np.sum(dynamic & ~mask)),
                         "best_true_body_gap_m": float(body.max()),
                         "mask_sha256": digest(mask_file)})
    report = {"schema": "rm_dynamic_prediction_near_x_noise_sensitivity/v1",
              "scope": "Offline fixed seed/batch/horizon Gaussian noise-scale sensitivity only; not runtime tuning, independent closed loops, ranking or full-cycle timing.",
              "fixture_inputs_sha256": digest(fixture / "inputs.json"),
              "baseline_inputs_sha256": digest(baseline / "inputs.json"),
              "rows": rows}
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"joint_safe_by_scale": {str(scale): [
        row["joint_safe_count"] for row in rows if row["noise_std_multiplier"] == scale]
        for scale in SCALES}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("trial", "baseline", "output"):
        prep.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("trial", "baseline", "fixture", "masks", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.baseline, args.output)
    else:
        evaluate(args.trial, args.baseline, args.fixture, args.masks, args.output)
