#!/usr/bin/env python3
"""Diagnose graded V1 risk on individually filtered frozen controls.

Standard critics remain the original native sampled-trajectory scores. The
candidate V1 term alone is evaluated on per-rollout filtered controls, so this
is a trajectory-alignment diagnostic, not a complete MPPI implementation.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, sample_omni
from conflict_exposure_probe import exposure
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry
import occupancy_rank_probe
import replay_ranking


BATCHES = (300, 600, 1000, 2000)


def filtered_poses(controls, meta, settings, history):
    limits = ((settings["vx_min"], settings["vx_max"]),
              (-settings["vy_max"], settings["vy_max"]),
              (-settings["wz_max"], settings["wz_max"]))
    poses = np.empty((len(controls), settings["steps"], 3), dtype=np.float32)
    for index, sampled in enumerate(controls):
        filtered = np.stack([replay_ranking.smooth_axis(
            np.clip(sampled[:, axis], *limits[axis]), history[:, axis])
            for axis in range(3)], axis=-1)
        # Match Optimizer::updateInitialStateVelocities: the first trajectory
        # step uses current robot speed, then prior filtered controls.
        velocities = np.empty_like(filtered)
        velocities[0] = np.asarray(meta["speed"], dtype=np.float32)
        velocities[1:] = filtered[:-1]
        trajectory = analyze.integrate_omni(
            *(velocities[:, axis] for axis in range(3)),
            meta["pose"], settings["dt"])
        for axis in range(3):
            poses[index, :, axis] = trajectory[axis]
    return poses


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"]) != (162, 300, 30):
        raise ValueError("not the fixed cycle 162")
    profile = yaml.safe_load(args.profile.read_text())
    params = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PredictionV1Critic"]
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    padded = meta["padded_footprint"]
    history_record = json.loads(args.history.read_text())
    if history_record["cycle_id"] != 162:
        raise ValueError("history belongs to another cycle")
    history = np.asarray(history_record["previous_outputs"], dtype=np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    truth = analyze.rows_from_transport(args.truth)
    times = [row["t"] for row in truth]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
        analyze.interpolated_pose(truth, times,
            consumed + (step + 1) * settings["dt"]))
        for step in range(settings["steps"])]
    reference = json.loads(args.native_summary.read_text())
    native_rows = {row["name"]: row for row in reference["rows"]}
    graded = json.loads(args.raw_graded_summary.read_text())
    graded_rows = {row["name"]: row for row in graded["rows"]}
    args.risk_output.mkdir(parents=True, exist_ok=True)
    rows = []
    aggregate_poses = []
    for seed in (-1, 0, 1, 2, 3):
        if seed == -1:
            cases = (("captured_batch300", 300),)
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
        else:
            cases = tuple((f"seed_{seed}_batch{batch}", batch)
                          for batch in BATCHES)
            sampled, _ = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
        poses = filtered_poses(controls, meta, settings, history)
        alternate = arrays + [("rollout." + axis, poses[:, :, i])
                              for i, axis in enumerate(("x", "y", "yaw"))]
        occupancy, hits = exposure(meta, alternate, params)
        if not np.all(occupancy == 1.0):
            raise ValueError("filtered controls no longer have common hard V1 term")
        risk = occupancy_rank_probe.expected_overlap(meta, alternate, params)
        risk_file = args.risk_output / f"{cases[-1][0]}_filtered_risk.bin"
        risk.astype("<f8").tofile(risk_file)
        for name, batch in cases:
            native = native_rows[name]
            score_file = args.native_results / f"{name}_full_scores.bin"
            if digest(score_file) != native["native_file_sha256"]["full_scores"]:
                raise ValueError(f"native scores changed: {name}")
            scores = np.fromfile(score_file, dtype="<f4")
            unit = np.float32((3.81 / 254.) * 1_000_000. / 9)
            result = aggregate(scores + np.float32(unit * risk[:batch]),
                               controls[:batch], initial, settings, history)
            geometry = open_loop_geometry(result["filtered_sequence"],
                                          meta, settings, actual, body, padded)
            aggregate_poses.append(analyze.integrate_omni(
                *(result["filtered_sequence"][:, i] for i in range(3)),
                meta["pose"], settings["dt"]))
            rows.append({"name": name, "batch": batch,
                         "filtered_prediction_hits_by_step": [batch] * len(hits),
                         "filtered_risk_file": risk_file.name,
                         "filtered_risk_sha256": digest(risk_file),
                         "risk_min": float(risk[:batch].min()),
                         "risk_max": float(risk[:batch].max()),
                         "raw_graded_geometry": graded_rows[name][
                             "graded_geometry"],
                         "aligned_geometry": geometry,
                         "aligned_effective_sample_size": float(
                             1. / np.sum(result["probability"] ** 2)),
                         "aligned_returned_control": result[
                             "returned_control"]})
    export(args.static_fixture, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack([pose[i] for pose in aggregate_poses])
                 for i in range(3)), shortcut_threshold(meta, args.profile))
    report = {"schema": "rm_dynamic_prediction/filtered_graded_batch/v1",
              "scope": "Diagnostic only: existing V1 graded overlap evaluated on each individually output-filtered control sequence with native current-speed first-step integration; native standard critics remain scores of original raw sampled trajectories. Original hard V1 term remains tied in all 17 batches. Fixed samples, temperature, final aggregate filter. Future Gazebo truth evaluates only.",
              "input_sha256": {"cycle": digest(args.cycle),
                               "profile": digest(args.profile),
                               "truth": digest(args.truth),
                               "history": digest(args.history),
                               "native_summary": digest(args.native_summary),
                               "raw_graded_summary": digest(
                                   args.raw_graded_summary)},
              "static_fixture_sha256": digest(args.static_fixture),
              "rows": rows,
              "aligned_dynamic_gate_pass_count": sum(
                  row["aligned_geometry"]["dynamic_clearance_gate_met"]
                  for row in rows)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "dynamic_gate_pass_count":
                      report["aligned_dynamic_gate_pass_count"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cycle", "profile", "truth", "history", "native-summary",
                 "native-results", "raw-graded-summary", "risk-output",
                 "static-fixture", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
