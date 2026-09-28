#!/usr/bin/env python3
"""Replay the existing V1 continuous-overlap experiment on frozen batches.

This preserves the original hard V1 score, standard critics, sampled controls,
temperature and filter. Uniform physical-box center is an uncalibrated ranking
assumption; future Gazebo truth is used only after aggregation for evaluation.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry
import occupancy_rank_probe


BATCHES = (300, 600, 1000, 2000)


def native_array(args, reference, name, suffix, shape):
    path = args.native_results / f"{name}_{suffix}.bin"
    if digest(path) != reference["native_file_sha256"][suffix]:
        raise ValueError(f"native replay output changed: {path}")
    return np.fromfile(path, dtype="<f4").reshape(shape)


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"],
            settings["iterations"]) != (162, 300, 30, 1):
        raise ValueError("not the frozen cycle 162")
    profile = yaml.safe_load(args.profile.read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    padded = meta["padded_footprint"]
    history_record = json.loads(args.history.read_text())
    if history_record["cycle_id"] != 162:
        raise ValueError("control history belongs to another cycle")
    history = np.asarray(history_record["previous_outputs"], dtype=np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    truth = analyze.rows_from_transport(args.truth)
    times = [row["t"] for row in truth]
    prediction = analyze.event(meta, "prediction.input")
    actual = [analyze.placed(analyze.obstacle_polygon(),
        analyze.interpolated_pose(truth, times,
            prediction["consumer_sim_s"] + (step + 1) * settings["dt"]))
        for step in range(settings["steps"])]
    accepted = json.loads(args.native_summary.read_text())
    references = {row["name"]: row for row in accepted["rows"]}
    guarded = json.loads(args.guarded_summary.read_text())
    guarded_rows = {row["name"]: row for row in guarded["rows"]}
    args.risk_output.mkdir(parents=True, exist_ok=True)
    rows = []
    aggregate_poses = []
    for seed in (-1, 0, 1, 2, 3):
        if seed == -1:
            cases = (("captured_batch300", 300),)
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
            mask = np.loadtxt(args.mask_dir / "captured_mask.txt", dtype=bool)
            largest = 300
        else:
            cases = tuple((f"seed_{seed}_batch{batch}", batch)
                          for batch in BATCHES)
            sampled, _ = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
            mask = np.loadtxt(args.mask_dir / f"seed_{seed}_mask.txt", dtype=bool)
            largest = 2000
        if mask.shape != (largest,):
            raise ValueError("static mask shape differs")
        largest_name = cases[-1][0]
        poses = native_array(args, references[largest_name], largest_name,
                             "poses", (largest, 30, 3))
        augmented = arrays + [("rollout." + axis, poses[:, :, index])
                              for index, axis in enumerate(("x", "y", "yaw"))]
        risk = occupancy_rank_probe.expected_overlap(meta, augmented, params)
        risk_path = args.risk_output / f"{largest_name}_risk.bin"
        risk.astype("<f8").tofile(risk_path)
        body_gaps, padded_gaps = geometry_labels(
            tuple(poses[:, :, index] for index in range(3)),
            body, padded, actual)
        joint_safe = (body_gaps >= .05) & (padded_gaps > 0) & ~mask
        for name, batch in cases:
            reference = references[name]
            if name != largest_name:
                prefix = native_array(args, reference, name, "poses",
                                      (batch, 30, 3))
                if not np.array_equal(prefix, poses[:batch]):
                    raise ValueError(f"native nested pose prefix differs: {name}")
            if int(joint_safe[:batch].sum()) != guarded_rows[name][
                    "raw_joint_safe_count"]:
                raise ValueError(f"joint safe labels changed: {name}")
            scores = native_array(args, reference, name, "full_scores", (batch,))
            native_after = native_array(args, reference, name,
                                        "after_filter", (30, 3))
            baseline = aggregate(scores, controls[:batch], initial,
                                 settings, history)
            if np.max(np.abs(baseline["filtered_sequence"] - native_after)) > 5e-5:
                raise ValueError(f"native baseline does not reaggregate: {name}")
            unit = np.float32((3.81 / 254.) * 1_000_000. / 9)
            graded_scores = scores + np.float32(unit * risk[:batch])
            graded = aggregate(graded_scores, controls[:batch], initial,
                               settings, history)
            baseline_geometry = open_loop_geometry(
                baseline["filtered_sequence"], meta, settings,
                actual, body, padded)
            reference_geometry = reference["filtered_geometry"]
            if abs(baseline_geometry["body_min_gap_m"] -
                   reference_geometry["body_min_gap_m"]) > 1e-4:
                raise ValueError(f"baseline geometry changed: {name}")
            geometry = open_loop_geometry(graded["filtered_sequence"], meta,
                                          settings, actual, body, padded)
            unfiltered_geometry = open_loop_geometry(
                graded["unfiltered_sequence"], meta, settings,
                actual, body, padded)
            aggregate_poses.append(analyze.integrate_omni(
                *(graded["filtered_sequence"][:, index]
                  for index in range(3)), meta["pose"], settings["dt"]))
            safe = joint_safe[:batch]
            weights = graded["probability"]
            row = {"name": name, "batch": batch,
                   "risk_file": risk_path.name,
                   "risk_file_sha256": digest(risk_path),
                   "risk_min": float(risk[:batch].min()),
                   "risk_max": float(risk[:batch].max()),
                   "joint_safe_count": int(safe.sum()),
                   "baseline_joint_safe_weight": float(
                       baseline["probability"][safe].sum()),
                   "graded_joint_safe_weight": float(weights[safe].sum()),
                   "graded_top_rollout": int(np.argmax(weights)),
                   "graded_top_rollout_joint_safe": bool(safe[np.argmax(weights)]),
                   "graded_top_rollout_raw_body_min_gap_m": float(
                       body_gaps[np.argmax(weights)]),
                   "graded_effective_sample_size": float(
                       1. / np.sum(weights * weights)),
                   "baseline_geometry": baseline_geometry,
                   "graded_unfiltered_geometry": unfiltered_geometry,
                   "graded_geometry": geometry,
                   "baseline_returned_control": baseline["returned_control"],
                   "graded_returned_control": graded["returned_control"]}
            rows.append(row)
    export(args.static_fixture, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack([pose[index] for pose in aggregate_poses])
                 for index in range(3)), shortcut_threshold(meta, args.profile))
    report = {"schema": "rm_dynamic_prediction/graded_overlap_batch/v1",
              "scope": "Frozen cycle 162, existing uncalibrated uniform physical-center overlap added at original V1 collision unit to native guarded score plus unchanged hard V1 term. Fixed controls, all other critics, temperature and filter. Gazebo truth only labels after scoring. Offline open-loop only; aggregate static CostCritic is not rechecked.",
              "input_sha256": {"cycle": digest(args.cycle),
                               "profile": digest(args.profile),
                               "truth": digest(args.truth),
                               "history": digest(args.history),
                               "native_summary": digest(args.native_summary),
                               "guarded_summary": digest(args.guarded_summary)},
              "static_fixture_sha256": digest(args.static_fixture),
              "rows": rows,
              "graded_dynamic_gate_pass_count": sum(
                  row["graded_geometry"]["dynamic_clearance_gate_met"]
                  for row in rows)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "dynamic_gate_pass_count":
                      report["graded_dynamic_gate_pass_count"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cycle", "profile", "truth", "history", "native-summary",
                 "native-results", "guarded-summary", "mask-dir", "risk-output",
                 "static-fixture", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
