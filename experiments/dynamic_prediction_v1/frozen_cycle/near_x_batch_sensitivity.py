#!/usr/bin/env python3
"""Preregistered nested filtered sample coverage on near-X cycle 73."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
import replay_ranking


EXPECTED = {
    "cycle": "5b9608928eb729a91256404770ca20a4143f4661ec4ec1938293a772e4283491",
    "binary": "37fc918ce37503f7192669f43162e21d7cb9507b318534dc80098734570d7087",
    "truth": "51ca1bd1851ebe7aeecab490a4b018908d26d58ecb512395cde4190e9149a491",
    "profile": "d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640",
}
SEEDS = (0, 1, 2, 3)
AXES = ("vx", "vy", "wz")


def frozen(trial):
    choice = json.loads((trial / "selection.json").read_text())
    cycle = trial / "mppi_cycles/cycle_73.json"
    paths = {"cycle": cycle, "binary": cycle.with_suffix(".bin"),
             "truth": trial / "gazebo_poses.jsonl", "profile": trial / "profile.yaml"}
    if choice["selected_cycle_id"] != 73 or any(
            digest(path) != EXPECTED[key] for key, path in paths.items()):
        raise ValueError("selected frozen input differs from preregistration")
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"],
            settings["iterations"], settings["offset"]) != (73, 300, 30, 1, 1):
        raise ValueError("frozen MPPI settings changed")
    if analyze.event(meta, "prediction.input")["status"] != "accepted":
        raise ValueError("prediction was not accepted")
    return paths, meta, arrays, settings


def prepare(trial, output):
    if output.exists():
        raise FileExistsError(output)
    paths, meta, arrays, settings = frozen(trial)
    truth = analyze.rows_from_transport(paths["truth"])
    times = [row["t"] for row in truth]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    if max(times) - consumed < 3.0:
        raise ValueError("physical future is shorter than 3 s")
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, times,
                  consumed + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    profile = yaml.safe_load(paths["profile"].read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    padded = meta["padded_footprint"]
    previous = replay_ranking.history_from_trial(
        trial / "mppi_cycles", 73, arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1).astype(np.float32)
    history_sha = {str(i): digest(trial / f"mppi_cycles/cycle_{i}.json")
                   for i in range(69, 73)}
    output.mkdir(parents=True)
    for seed in SEEDS:
        sampled, _ = sample_omni(meta, arrays, BATCHES[-1], seed)
        controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
        poses = filtered_poses(controls, meta, settings, history)
        body_gap, padded_gap = geometry_labels(
            tuple(poses[:, :, axis] for axis in range(3)), body, padded, actual)
        np.savez(output / f"seed_{seed}_gaps.npz",
                 body=body_gap, padded=padded_gap)
        export(output / f"seed_{seed}_map.bin", meta,
               analyze.last(arrays, "locked.raw_map"),
               tuple(poses[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, paths["profile"]))
    record = {"schema": "rm_dynamic_prediction_near_x_batch_inputs/v1",
              "inputs_sha256": EXPECTED, "history_sha256": history_sha,
              "seeds": SEEDS, "batches": BATCHES,
              "consumer_sim_s": consumed, "physical_future_s": max(times) - consumed,
              "files_sha256": {p.name: digest(p) for p in sorted(output.iterdir())}}
    (output / "inputs.json").write_text(json.dumps(record, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"prepared": len(SEEDS), "output": str(output)}))


def evaluate(trial, fixture, masks, native_binary, image_id, output):
    if output.exists():
        raise FileExistsError(output)
    paths, _, _, _ = frozen(trial)
    record = json.loads((fixture / "inputs.json").read_text())
    if record["inputs_sha256"] != EXPECTED or record["seeds"] != list(SEEDS) or \
            record["batches"] != list(BATCHES):
        raise ValueError("preregistered fixture metadata changed")
    for index, sha in record["history_sha256"].items():
        if digest(trial / f"mppi_cycles/cycle_{index}.json") != sha:
            raise ValueError("control history changed")
    for name, sha in record["files_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError(f"fixture changed: {name}")
    if not image_id.startswith("sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6"):
        raise ValueError("wrong runtime image")
    if digest(native_binary) != "37671b08cd8248f336703d79da4b5dd73fd7677999d24706f4955b7b6f6c059a":
        raise ValueError("native static checker changed")
    rows = []
    for seed in SEEDS:
        with np.load(fixture / f"seed_{seed}_gaps.npz") as gaps:
            body, padded = gaps["body"], gaps["padded"]
        mask_path = masks / f"seed_{seed}_mask.txt"
        mask = np.loadtxt(mask_path, dtype=bool)
        if body.shape != (2000,) or padded.shape != body.shape or mask.shape != body.shape:
            raise ValueError(f"seed {seed} output size differs")
        dynamic = (body >= .05) & (padded > 0)
        prefixes = {}
        for batch in BATCHES:
            prefixes[str(batch)] = {
                "dynamic_safe_count": int(dynamic[:batch].sum()),
                "native_static_collision_count": int(mask[:batch].sum()),
                "joint_safe_count": int(np.sum(dynamic[:batch] & ~mask[:batch])),
                "best_true_body_gap_m": float(body[:batch].max()),
            }
        rows.append({"seed": seed, "mask_sha256": digest(mask_path),
                     "prefixes": prefixes})
    report = {"schema": "rm_dynamic_prediction_near_x_batch_sensitivity/v1",
              "scope": "Nested offline samples of one frozen input, individually constrained and filtered; future physical truth labels, native raw-map static masks. Not independent closed loops or full MPPI timing.",
              "image_id": image_id, "native_binary_sha256": digest(native_binary),
              "fixture_inputs_sha256": digest(fixture / "inputs.json"),
              "input_sha256": {key: digest(path) for key, path in paths.items()},
              "rows": rows}
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "batch2000_joint_safe":
                      [row["prefixes"]["2000"]["joint_safe_count"] for row in rows]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--trial", type=Path, required=True)
    prep.add_argument("--output", type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("trial", "fixture", "masks", "native-binary", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    check.add_argument("--image-id", required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.output)
    else:
        evaluate(args.trial, args.fixture, args.masks, args.native_binary,
                 args.image_id, args.output)
