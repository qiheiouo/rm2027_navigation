#!/usr/bin/env python3
"""Preregistered noise-scale diagnostic on opposite-side frozen cycle 65."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from near_x_south_batch import SEEDS, frozen, physical
from phase2_filtered_audit import candidates


SCALES = (2, 4)
AXES = ("vx", "vy", "wz")


def prepare(trial, baseline, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, truth = frozen(trial)
    base_record = json.loads((baseline / "inputs.json").read_text())
    if base_record["cycle_sha256"] != digest(cycle):
        raise ValueError("opposite-side baseline changed")
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    actual = physical(meta, settings, truth)
    raw = analyze.last(arrays, "locked.raw_map")
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1).astype(np.float32)
    history = candidates(trial, meta, arrays, settings)[3]
    threshold = shortcut_threshold(meta, profile_path)
    output.mkdir(parents=True)
    for seed in SEEDS:
        sampled, _ = sample_omni(meta, arrays, 2000, seed)
        base_controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
        base_poses = filtered_poses(base_controls, meta, settings, history)
        check = output / f"seed_{seed}_scale1_check.bin"
        export(check, meta, raw,
               tuple(base_poses[:, :, axis] for axis in range(3)), threshold)
        if digest(check) != base_record["files_sha256"][f"seed_{seed}_map.bin"]:
            raise ValueError("scale-one sample does not match fixed baseline")
        check.unlink()
        for scale in SCALES:
            controls = initial[None, :, :] + np.float32(scale) * \
                (base_controls - initial[None, :, :])
            poses = filtered_poses(controls, meta, settings, history)
            body_gap, padded_gap = geometry_labels(
                tuple(poses[:, :, axis] for axis in range(3)),
                body, meta["padded_footprint"], actual)
            stem = f"seed_{seed}_scale_{scale}"
            np.savez(output / f"{stem}_gaps.npz", body=body_gap,
                     padded=padded_gap)
            export(output / f"{stem}_map.bin", meta, raw,
                   tuple(poses[:, :, axis] for axis in range(3)), threshold)
    report = {"schema": "rm_dynamic_prediction_south_noise_inputs/v1",
              "cycle_sha256": digest(cycle),
              "baseline_inputs_sha256": digest(baseline / "inputs.json"),
              "seeds": SEEDS, "scales": SCALES, "batch": 2000,
              "files_sha256": {p.name: digest(p) for p in output.iterdir()}}
    (output / "inputs.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"cases": len(SEEDS) * len(SCALES), "output": str(output)}))


def evaluate(trial, baseline, fixture, masks, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, _, _, _, _, _ = frozen(trial)
    report = json.loads((fixture / "inputs.json").read_text())
    if report["cycle_sha256"] != digest(cycle) or \
            report["baseline_inputs_sha256"] != digest(baseline / "inputs.json") or \
            report["seeds"] != list(SEEDS) or report["scales"] != list(SCALES):
        raise ValueError("noise fixture metadata changed")
    for name, sha in report["files_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError("noise fixture changed: " + name)
    rows = []
    for seed in SEEDS:
        for scale in SCALES:
            stem = f"seed_{seed}_scale_{scale}"
            with np.load(fixture / f"{stem}_gaps.npz") as gaps:
                body, padded = gaps["body"], gaps["padded"]
            mask_path = masks / f"{stem}_mask.txt"
            mask = np.loadtxt(mask_path, dtype=bool)
            if body.shape != (2000,) or padded.shape != body.shape or \
                    mask.shape != body.shape:
                raise ValueError("noise result dimensions differ")
            dynamic = (body >= .05) & (padded > 0)
            rows.append({"seed": seed, "noise_std_multiplier": scale,
                         "dynamic_safe_count": int(dynamic.sum()),
                         "native_static_collision_count": int(mask.sum()),
                         "joint_safe_count": int(np.sum(dynamic & ~mask)),
                         "best_true_body_gap_m": float(body.max()),
                         "static_mask_sha256": digest(mask_path)})
    result = {"schema": "rm_dynamic_prediction_south_noise_sensitivity/v1",
              "scope": "Opposite-side selected cycle, fixed batch/seeds/horizon and current-speed first step; only noise std scale differs. Offline truth labels and native raw-map masks, no ranking or runtime parameter changes.",
              "fixture_inputs_sha256": digest(fixture / "inputs.json"),
              "rows": rows}
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
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
