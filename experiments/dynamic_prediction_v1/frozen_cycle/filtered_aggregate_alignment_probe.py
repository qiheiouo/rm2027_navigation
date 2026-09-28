#!/usr/bin/env python3
"""Compare two aggregation semantics for the same filtered-candidate scores.

The score and probability are unchanged. One output weights sampled controls
and applies the final Nav2 filter; the other weights individually filtered
candidate controls and does not apply a second filter. This is an offline
diagnostic on preselected frozen cycles, not a runtime controller change.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from costmap_mask_fixture import export, shortcut_threshold
from filtered_all_critic_probe import filtered_controls
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry
import replay_ranking


CASES = (("early_147", 147, "collision_trial", "score147"),
         ("intrusion_162", 162, "collision_trial", "score162"),
         ("goal_263", 263, "goal_trial", "score263"))


def actual_obstacle(meta, settings, trial):
    truth = analyze.rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in truth]
    stamp = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    return [analyze.placed(analyze.obstacle_polygon(),
            analyze.interpolated_pose(truth, times,
                                      stamp + (step + 1) * settings["dt"]))
            for step in range(settings["steps"])]


def one_case(name, cycle_id, trial, score_path, output_dir):
    cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
    profile_path = trial / "profile.yaml"
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (settings["batch"], settings["steps"]) != (300, 30):
        raise ValueError(f"frozen settings differ: {name}")
    profile = yaml.safe_load(profile_path.read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                         for axis in AXES], axis=-1)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    previous = replay_ranking.history_from_trial(
        trial / "mppi_cycles", cycle_id, arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1)
    candidates = filtered_controls(controls, settings, history)
    poses = filtered_poses(controls, meta, settings, history)
    prediction, _, _ = filtered_prediction_score(meta, poses, params)
    standard = np.fromfile(score_path, dtype="<f4")
    if standard.shape != (300,) or not np.isfinite(standard).all():
        raise ValueError(f"native standard score missing or invalid: {name}")
    ranked = aggregate(standard + prediction, controls, initial,
                       settings, history)
    mixed = ranked["filtered_sequence"]
    aligned = np.einsum("i,ijk->jk", ranked["probability"], candidates,
                        dtype=np.float32)
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    actual = actual_obstacle(meta, settings, trial)
    geometries = [open_loop_geometry(sequence, meta, settings, actual,
                                     body, meta["padded_footprint"])
                  for sequence in (mixed, aligned)]
    if cycle_id == 263:
        goal = json.loads((trial / "runtime_audit.json").read_text())[
            "navigation_result"]["goal"]
        for geometry, sequence in zip(geometries, (mixed, aligned)):
            trajectory = analyze.integrate_omni(
                *(sequence[:, axis] for axis in range(3)),
                meta["pose"], settings["dt"])
            geometry["endpoint_goal_position_distance_m"] = float(np.hypot(
                trajectory[0][-1] - goal[0],
                trajectory[1][-1] - goal[1]))
    trajectories = [analyze.integrate_omni(
        *(sequence[:, axis] for axis in range(3)),
        meta["pose"], settings["dt"]) for sequence in (mixed, aligned)]
    fixture = output_dir / f"{name}_static.bin"
    export(fixture, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack([trajectory[axis] for trajectory in trajectories])
                 for axis in range(3)), shortcut_threshold(meta, profile_path))
    return {"name": name, "cycle_id": cycle_id,
            "cycle_sha256": digest(cycle),
            "profile_sha256": digest(profile_path),
            "truth_sha256": digest(trial / "gazebo_poses.jsonl"),
            "standard_score_sha256": digest(score_path),
            "prediction_score_sha256": hashlib.sha256(
                prediction.astype("<f4").tobytes()).hexdigest(),
            "static_fixture": str(fixture.resolve()),
            "static_fixture_sha256": digest(fixture),
            "full_sequence_max_abs_delta": float(np.max(np.abs(
                aligned - mixed))),
            "returned_control_max_abs_delta": float(np.max(np.abs(
                aligned[settings["offset"]] - mixed[settings["offset"]]))),
            "mixed": {"returned_control": mixed[settings["offset"]]
                      .astype(float).tolist(), "geometry": geometries[0]},
            "aligned": {"returned_control": aligned[settings["offset"]]
                        .astype(float).tolist(), "geometry": geometries[1]}}


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    rows = [one_case(name, cycle_id, getattr(args, trial_key),
                     getattr(args, score_key), args.output_dir)
            for name, cycle_id, trial_key, score_key in CASES]
    report = {"schema": "rm_dynamic_prediction/filtered_aggregate_alignment/v1",
              "scope": "Preselected cycles 147, 162, 263. Identical native seven-critic and filtered-candidate V1 scores, original MPPI regularizer/softmax, two output aggregation semantics. Truth evaluates output only. Static masks pending finalize.",
              "cases": rows}
    (args.output_dir / "prepared.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


def finalize(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    report = json.loads(args.prepared.read_text())
    for row in report["cases"]:
        fixture = Path(row["static_fixture"])
        if digest(fixture) != row["static_fixture_sha256"]:
            raise ValueError(f"static fixture changed: {row['name']}")
        path = args.mask_dir / f"{row['name']}_mask.txt"
        masks = [int(value) for value in path.read_text().split()]
        if len(masks) != 2 or any(value not in (0, 1) for value in masks):
            raise ValueError(f"native static mask malformed: {row['name']}")
        row["static_mask_sha256"] = digest(path)
        for label, mask in zip(("mixed", "aligned"), masks):
            row[label]["geometry"]["costcritic_collision"] = bool(mask)
            row[label]["geometry"]["joint_clearance_gate_met"] = bool(
                row[label]["geometry"]["dynamic_clearance_gate_met"] and
                not mask)
    report["scope"] = report["scope"].replace(
        "Static masks pending finalize.",
        "Output trajectories checked by guarded native CostCritic mask on each frozen raw local costmap.")
    report["prepared_sha256"] = digest(args.prepared)
    report["native_mask_binary_sha256"] = digest(args.native_binary)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["cases"])}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("collision-trial", "goal-trial", "score147", "score162",
                 "score263", "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    done = sub.add_parser("finalize")
    for name in ("prepared", "mask-dir", "native-binary", "output"):
        done.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "finalize": finalize}[args.command](args)
