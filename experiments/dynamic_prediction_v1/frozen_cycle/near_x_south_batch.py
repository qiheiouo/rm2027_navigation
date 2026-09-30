#!/usr/bin/env python3
"""Nested filtered batch and captured baseline gate on south-start cycle 65."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from phase2_filtered_audit import candidates, selected


EXPECTED_CYCLE = "cf62847bc075612b793594cbe6c18c9b33db11a69f051aa68e02a551d83f68c1"
EXPECTED_TRUTH = "1762eaf96807d1de58c2e2a4567bf38f0eeabcb8fb7145eecd111acac4ddf173"
SEEDS = (0, 1, 2, 3)


def frozen(trial):
    cycle, meta, arrays, settings, profile, truth = selected(trial)
    if meta["cycle_id"] != 65 or digest(cycle) != EXPECTED_CYCLE or \
            digest(truth) != EXPECTED_TRUTH or settings["steps"] != 30 or \
            settings["batch"] != 300 or settings["iterations"] != 1:
        raise ValueError("preregistered south-start selected input changed")
    return cycle, meta, arrays, settings, profile, truth


def physical(meta, settings, truth):
    rows = analyze.rows_from_transport(truth)
    times = [r["t"] for r in rows]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    if times[-1] - consumed < 3.0:
        raise ValueError("physical future shorter than 3 s")
    return [analyze.placed(analyze.obstacle_polygon(),
            analyze.interpolated_pose(rows, times,
                consumed + (j + 1) * settings["dt"]))
            for j in range(settings["steps"])]


def prepare(trial, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, truth = frozen(trial)
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    actual = physical(meta, settings, truth)
    raw = analyze.last(arrays, "locked.raw_map")
    history_source = candidates(trial, meta, arrays, settings)[3]
    output.mkdir(parents=True)
    for seed in SEEDS:
        sampled, _ = sample_omni(meta, arrays, 2000, seed)
        controls = np.stack([sampled[axis] for axis in ("vx", "vy", "wz")],
                            axis=-1)
        poses = filtered_poses(controls, meta, settings, history_source)
        body_gap, padded_gap = geometry_labels(
            tuple(poses[:, :, axis] for axis in range(3)),
            body, meta["padded_footprint"], actual)
        np.savez(output / f"seed_{seed}_gaps.npz", body=body_gap,
                 padded=padded_gap)
        export(output / f"seed_{seed}_map.bin", meta, raw,
               tuple(poses[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
    report = {"schema": "rm_dynamic_prediction_south_batch_inputs/v1",
              "cycle_sha256": digest(cycle), "cycle_bin_sha256": digest(
                  cycle.with_suffix(".bin")),
              "profile_sha256": digest(profile_path), "truth_sha256": digest(truth),
              "control_history_sha256": {str(i): digest(
                  trial / f"mppi_cycles/cycle_{i}.json") for i in range(61, 65)},
              "seeds": SEEDS, "batches": BATCHES,
              "files_sha256": {p.name: digest(p) for p in output.iterdir()}}
    (output / "inputs.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"cases": len(SEEDS), "output": str(output)}))


def evaluate(trial, fixture, masks, native_checker, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, truth = frozen(trial)
    inputs = json.loads((fixture / "inputs.json").read_text())
    for key, path in (("cycle_sha256", cycle),
                      ("cycle_bin_sha256", cycle.with_suffix(".bin")),
                      ("profile_sha256", profile_path), ("truth_sha256", truth)):
        if digest(path) != inputs[key]:
            raise ValueError("frozen input changed: " + key)
    for index, sha in inputs["control_history_sha256"].items():
        if digest(trial / f"mppi_cycles/cycle_{index}.json") != sha:
            raise ValueError("control history changed")
    for name, sha in inputs["files_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError("sample fixture changed: " + name)
    if digest(native_checker) != "37671b08cd8248f336703d79da4b5dd73fd7677999d24706f4955b7b6f6c059a":
        raise ValueError("native checker changed")
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    actual = physical(meta, settings, truth)
    raw_mask = np.loadtxt(trial / "native_raw_cycle_65/raw_static_mask.txt", dtype=bool)
    captured_mask = np.asarray(analyze.last(arrays, "cost_critic.collisions"),
                               dtype=bool)
    if raw_mask.shape != (300,) or not np.array_equal(raw_mask, captured_mask):
        raise ValueError("captured static mask does not replay")
    _, _, captured_poses, _ = candidates(trial, meta, arrays, settings)
    captured_body, captured_padded = geometry_labels(
        tuple(captured_poses[:, :, axis] for axis in range(3)),
        body, meta["padded_footprint"], actual)
    filtered_mask_path = trial / "filtered_fixture_65/filtered_static_mask.txt"
    filtered_mask = np.loadtxt(filtered_mask_path, dtype=bool)
    if filtered_mask.shape != (300,):
        raise ValueError("filtered static mask size changed")
    captured_safe = (captured_body >= .05) & (captured_padded > 0)
    captured = {"filtered_dynamic_safe_count": int(captured_safe.sum()),
                "filtered_static_collision_count": int(filtered_mask.sum()),
                "filtered_joint_safe_count": int(np.sum(captured_safe & ~filtered_mask)),
                "filtered_best_true_body_gap_m": float(captured_body.max()),
                "captured_raw_static_mask_mismatch_count": 0,
                "filtered_static_mask_sha256": digest(filtered_mask_path)}
    raw_scores = np.fromfile(trial / "native_raw_cycle_65/raw_scores.bin", dtype="<f4")
    captured_scores = analyze.last(arrays, "critic.FollowPath.PathAngleCritic")
    diff = np.abs(raw_scores - captured_scores)
    captured["native_score_replay_mismatch_gt_1e_3"] = int(np.sum(diff > 1e-3))
    captured["native_score_replay_max_abs_error"] = float(diff.max())
    rows = []
    for seed in SEEDS:
        with np.load(fixture / f"seed_{seed}_gaps.npz") as gaps:
            body_gap, padded_gap = gaps["body"], gaps["padded"]
        mask_path = masks / f"seed_{seed}_mask.txt"
        mask = np.loadtxt(mask_path, dtype=bool)
        if body_gap.shape != (2000,) or padded_gap.shape != body_gap.shape or \
                mask.shape != body_gap.shape:
            raise ValueError("seed result shape changed")
        safe = (body_gap >= .05) & (padded_gap > 0)
        prefixes = {str(n): {"dynamic_safe_count": int(safe[:n].sum()),
                             "native_static_collision_count": int(mask[:n].sum()),
                             "joint_safe_count": int(np.sum(safe[:n] & ~mask[:n])),
                             "best_true_body_gap_m": float(body_gap[:n].max())}
                    for n in BATCHES}
        rows.append({"seed": seed, "mask_sha256": digest(mask_path),
                     "prefixes": prefixes})
    report = {"schema": "rm_dynamic_prediction_south_batch/v1",
              "scope": "Independent opposite-side closed-loop source; selected frozen cycle only. Fixed-seed nested offline candidate geometry and exact native raw-map static masks. Native standard score replay difference disclosed, so no ranking inference.",
              "fixture_inputs_sha256": digest(fixture / "inputs.json"),
              "native_checker_sha256": digest(native_checker),
              "captured": captured, "rows": rows}
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"captured_joint": captured["filtered_joint_safe_count"],
                      "batch2000_joint": [row["prefixes"]["2000"][
                          "joint_safe_count"] for row in rows]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--trial", type=Path, required=True)
    prep.add_argument("--output", type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("trial", "fixture", "masks", "native-checker", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.output)
    else:
        evaluate(args.trial, args.fixture, args.masks, args.native_checker,
                 args.output)
