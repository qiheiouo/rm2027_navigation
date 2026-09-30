#!/usr/bin/env python3
"""Fixed, nonoptimized constant-command grid for frozen near-X reachability."""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from near_x_batch_sensitivity import AXES, EXPECTED, frozen
import replay_ranking


VX = (-.5, -.25, 0., .25, .5, .8)
VY = (-.5, -.25, 0., .25, .5)
WZ = (-1.2, -.6, 0., .6, 1.2)
GRID = tuple(itertools.product(VX, VY, WZ))


def prepare(trial, output):
    if output.exists():
        raise FileExistsError(output)
    paths, meta, arrays, settings = frozen(trial)
    previous = replay_ranking.history_from_trial(trial / "mppi_cycles", 73, arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1).astype(np.float32)
    controls = np.broadcast_to(np.asarray(GRID, dtype=np.float32)[:, None, :],
                               (len(GRID), settings["steps"], 3)).copy()
    poses = filtered_poses(controls, meta, settings, history)
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
    body_gap, padded_gap = geometry_labels(
        tuple(poses[:, :, axis] for axis in range(3)), body,
        meta["padded_footprint"], actual)
    output.mkdir(parents=True)
    np.savez(output / "labels.npz", body=body_gap, padded=padded_gap,
             endpoint=poses[:, -1, :])
    export(output / "map.bin", meta, analyze.last(arrays, "locked.raw_map"),
           tuple(poses[:, :, axis] for axis in range(3)),
           shortcut_threshold(meta, paths["profile"]))
    report = {"schema": "rm_dynamic_prediction_near_x_boundary_grid_inputs/v1",
              "inputs_sha256": EXPECTED, "grid": GRID,
              "history_sha256": {str(i): digest(trial / f"mppi_cycles/cycle_{i}.json")
                                 for i in range(69, 73)},
              "files_sha256": {p.name: digest(p) for p in output.iterdir()}}
    (output / "inputs.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"grid_count": len(GRID), "output": str(output)}))


def evaluate(trial, fixture, mask_path, output):
    if output.exists():
        raise FileExistsError(output)
    _, meta, _, _ = frozen(trial)
    inputs = json.loads((fixture / "inputs.json").read_text())
    if inputs["inputs_sha256"] != EXPECTED or tuple(map(tuple, inputs["grid"])) != GRID:
        raise ValueError("fixed grid changed")
    for name, sha in inputs["files_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError("grid fixture changed")
    for index, sha in inputs["history_sha256"].items():
        if digest(trial / f"mppi_cycles/cycle_{index}.json") != sha:
            raise ValueError("control history changed")
    with np.load(fixture / "labels.npz") as data:
        body, padded, endpoint = data["body"], data["padded"], data["endpoint"]
    mask = np.loadtxt(mask_path, dtype=bool)
    if body.shape != (150,) or padded.shape != body.shape or \
            endpoint.shape != (150, 3) or mask.shape != body.shape:
        raise ValueError("grid output shape differs")
    safe = (body >= .05) & (padded > 0) & ~mask
    goal = meta["path"][-1]
    start_goal = float(np.hypot(meta["pose"][0] - goal[0],
                                meta["pose"][1] - goal[1]))
    witnesses = [{"grid_index": int(i), "constant_control": GRID[i],
                  "body_gap_m": float(body[i]), "padded_gap_m": float(padded[i]),
                  "goal_progress_m": start_goal - float(np.hypot(
                      endpoint[i, 0] - goal[0], endpoint[i, 1] - goal[1]))}
                 for i in np.flatnonzero(safe)]
    report = {"schema": "rm_dynamic_prediction_near_x_boundary_grid/v1",
              "scope": "Offline constant-command witness grid; original constraints/filter/current-speed first step and raw static mask. No online policy or full-domain impossibility proof.",
              "inputs_sha256": digest(fixture / "inputs.json"),
              "native_mask_sha256": digest(mask_path),
              "dynamic_safe_count": int(np.sum((body >= .05) & (padded > 0))),
              "native_static_collision_count": int(mask.sum()),
              "joint_safe_count": len(witnesses),
              "best_body_gap_m": float(body.max()),
              "witnesses": witnesses}
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"joint_safe_count": len(witnesses),
                      "best_body_gap_m": report["best_body_gap_m"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--trial", type=Path, required=True)
    prep.add_argument("--output", type=Path, required=True)
    check = sub.add_parser("evaluate")
    check.add_argument("--trial", type=Path, required=True)
    check.add_argument("--fixture", type=Path, required=True)
    check.add_argument("--mask", type=Path, required=True)
    check.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.output)
    else:
        evaluate(args.trial, args.fixture, args.mask, args.output)
