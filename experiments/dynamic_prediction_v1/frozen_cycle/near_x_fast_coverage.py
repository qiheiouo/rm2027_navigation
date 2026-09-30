#!/usr/bin/env python3
"""Captured and fixed-seed coverage gates for the selected fast-X cycle 47."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from near_x_south_batch import physical
from phase2_filtered_audit import candidates, selected


EXPECTED = {
    "cycle": "b31352dcf763fbec0f05b8a092a32237c8b7c5a3d076e9b8e1b46139a501aec6",
    "binary": "9a21c928a1a89a7c73320de890bebfd2e259dd4d2f74452ee06c5523ee1eff51",
    "truth": "b5de3aaa439d999b4eec1ab2b142a1bb26516dd00703c218d2044342547c902f",
    "profile": "d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640"}
CHECKER_SHA = "37671b08cd8248f336703d79da4b5dd73fd7677999d24706f4955b7b6f6c059a"
SEEDS = (0, 1, 2, 3)
AXES = ("vx", "vy", "wz")


def frozen(trial):
    cycle, meta, arrays, settings, profile, truth = selected(trial)
    paths = {"cycle": cycle, "binary": cycle.with_suffix(".bin"),
             "truth": truth, "profile": profile}
    if any(digest(path) != EXPECTED[key] for key, path in paths.items()) or \
            (meta["cycle_id"], settings["batch"], settings["steps"],
             settings["iterations"], settings["offset"]) != (47, 300, 30, 1, 1):
        raise ValueError("preregistered fast-motion frozen input changed")
    return cycle, meta, arrays, settings, profile, truth


def prepare(trial, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, truth = frozen(trial)
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    actual = physical(meta, settings, truth)
    raw = analyze.last(arrays, "locked.raw_map")
    _, _, captured, history = candidates(trial, meta, arrays, settings)
    threshold = shortcut_threshold(meta, profile_path)
    output.mkdir(parents=True)

    def save(stem, poses):
        gaps = geometry_labels(tuple(poses[:, :, axis] for axis in range(3)),
                               body, meta["padded_footprint"], actual)
        np.savez(output / f"{stem}_gaps.npz", body=gaps[0], padded=gaps[1])
        export(output / f"{stem}_map.bin", meta, raw,
               tuple(poses[:, :, axis] for axis in range(3)), threshold)
        return gaps

    save("captured_filtered", captured)
    sequence = np.stack([analyze.last(arrays, "after_filter." + axis)
                         for axis in AXES], axis=-1)
    velocity = np.concatenate((np.asarray(meta["speed"], dtype=np.float32)[None, :],
                               sequence[:-1]), axis=0)
    pose = analyze.integrate_omni(*(velocity[:, axis] for axis in range(3)),
                                  meta["pose"], settings["dt"])
    save("captured_output", np.stack(pose, axis=-1)[None, :, :])
    goal = json.loads((trial / "observation/summary.json").read_text())["goal"]
    progress = float(np.hypot(meta["pose"][0] - goal[0], meta["pose"][1] - goal[1]) -
                     np.hypot(pose[0][-1] - goal[0], pose[1][-1] - goal[1]))
    for seed in SEEDS:
        sampled, _ = sample_omni(meta, arrays, 2000, seed)
        controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
        save(f"seed_{seed}", filtered_poses(controls, meta, settings, history))
    report = {"schema": "rm_dynamic_prediction_fast_x_coverage_inputs/v1",
              "input_sha256": EXPECTED, "seeds": SEEDS, "batches": BATCHES,
              "captured_output_goal_progress_m": progress,
              "history_sha256": {str(i): digest(trial / f"mppi_cycles/cycle_{i}.json")
                                 for i in range(43, 47)},
              "files_sha256": {p.name: digest(p) for p in sorted(output.iterdir())}}
    (output / "inputs.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"cycle_id": 47, "seeds": len(SEEDS), "output": str(output)}))


def evaluate(trial, fixture, masks, raw_fixture, checker, output):
    if output.exists():
        raise FileExistsError(output)
    _, meta, arrays, _, _, _ = frozen(trial)
    record = json.loads((fixture / "inputs.json").read_text())
    if record["input_sha256"] != EXPECTED or record["seeds"] != list(SEEDS) or \
            record["batches"] != list(BATCHES) or digest(checker) != CHECKER_SHA:
        raise ValueError("coverage metadata or native checker changed")
    for name, sha in record["files_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError("coverage fixture changed: " + name)
    for index, sha in record["history_sha256"].items():
        if digest(trial / f"mppi_cycles/cycle_{index}.json") != sha:
            raise ValueError("filter history changed")
    native_mask = np.loadtxt(raw_fixture / "raw_static_mask.txt", dtype=bool)
    captured_mask = np.asarray(analyze.last(arrays, "cost_critic.collisions"), dtype=bool)
    native_score = np.fromfile(raw_fixture / "raw_scores.bin", dtype="<f4")
    captured_score = analyze.last(arrays, "critic.FollowPath.PathAngleCritic")
    if native_mask.shape != (300,) or native_score.shape != (300,):
        raise ValueError("captured native replay count changed")
    error = np.abs(native_score - captured_score)
    replay = {"raw_static_mask_mismatch_count": int(np.sum(native_mask != captured_mask)),
              "raw_standard_score_mismatch_gt_1e_3": int(np.sum(error > 1e-3)),
              "raw_standard_score_max_abs_error": float(error.max()),
              "raw_scores_sha256": digest(raw_fixture / "raw_scores.bin"),
              "raw_static_mask_sha256": digest(raw_fixture / "raw_static_mask.txt")}
    if replay["raw_static_mask_mismatch_count"]:
        raise ValueError("native raw-map static mask does not match capture")

    def counts(stem, count):
        with np.load(fixture / f"{stem}_gaps.npz") as data:
            body, padded = data["body"], data["padded"]
        mask_path = masks / f"{stem}_mask.txt"
        mask = np.atleast_1d(np.loadtxt(mask_path, dtype=bool))
        if body.shape != (count,) or padded.shape != body.shape or mask.shape != body.shape:
            raise ValueError("coverage label dimensions differ")
        safe = (body >= .05) & (padded > 0) & ~mask
        sizes = BATCHES if count == 2000 else (count,)
        return {"mask_sha256": digest(mask_path), "prefixes": {
            str(size): {"joint_safe_count": int(safe[:size].sum()),
                        "dynamic_safe_count": int(((body[:size] >= .05) &
                                                  (padded[:size] > 0)).sum()),
                        "static_collision_count": int(mask[:size].sum()),
                        "best_body_gap_m": float(body[:size].max()),
                        "minimum_body_gap_m": float(body[:size].min()),
                        "minimum_padded_gap_m": float(padded[:size].min())}
            for size in sizes}}

    captured = counts("captured_filtered", 300)
    current = counts("captured_output", 1)
    rows = [{"seed": seed, **counts(f"seed_{seed}", 2000)} for seed in SEEDS]
    result = {"schema": "rm_dynamic_prediction_fast_x_coverage/v1",
              "scope": "One fixed period-six selected cycle. Captured and nested original-noise samples after individual filtering, current-speed first step, physical future labels and native raw-map masks. No runtime tuning.",
              "input_sha256": EXPECTED,
              "fixture_sha256": digest(fixture / "inputs.json"),
              "native_checker_sha256": digest(checker),
              "native_replay": replay, "captured_filtered": captured,
              "captured_output": current,
              "captured_output_goal_progress_m": record["captured_output_goal_progress_m"],
              "rows": rows,
              "scale4_triggered": any(r["prefixes"]["2000"]["joint_safe_count"] == 0
                                      for r in rows)}
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"captured_filtered_joint_safe": captured["prefixes"]["300"]["joint_safe_count"],
                      "batch2000_safe": [r["prefixes"]["2000"]["joint_safe_count"] for r in rows],
                      "scale4_triggered": result["scale4_triggered"], "native_replay": replay}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--trial", type=Path, required=True)
    prep.add_argument("--output", type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("trial", "fixture", "masks", "raw-fixture", "checker", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.output)
    else:
        evaluate(args.trial, args.fixture, args.masks, args.raw_fixture,
                 args.checker, args.output)
