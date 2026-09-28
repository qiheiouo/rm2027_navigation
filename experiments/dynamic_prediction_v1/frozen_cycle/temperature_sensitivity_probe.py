#!/usr/bin/env python3
"""Test MPPI temperature alone on fixed guarded C++ scores and controls.

The frozen V1 first-step collision is identical for all samples in cycle 162.
Truth is used only for labels and after-aggregation geometry, never for weights.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels, sample_omni
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry


FACTORS = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0)
BATCHES = (300, 600, 1000, 2000)


def run(args):
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"],
            settings["iterations"]) != (162, 300, 30, 1):
        raise ValueError("not the preregistered frozen cycle")
    prediction = analyze.event(meta, "prediction.input")
    if prediction["status"] != "accepted":
        raise ValueError("frozen prediction was not accepted")
    guarded = json.loads(args.guarded_summary.read_text())
    expected = {row["name"]: row for row in guarded["rows"]}
    profile = yaml.safe_load(args.profile.read_text())
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
    actual = [analyze.placed(analyze.obstacle_polygon(),
        analyze.interpolated_pose(truth, times,
            prediction["consumer_sim_s"] + (step + 1) * settings["dt"]))
        for step in range(settings["steps"])]
    rows = []
    for seed in (-1, 0, 1, 2, 3):
        if seed == -1:
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
            trajectories = tuple(analyze.last(arrays, "rollout." + axis)
                                 for axis in ("x", "y", "yaw"))
            batches = (300,)
        else:
            sampled_controls, trajectories = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([sampled_controls[axis] for axis in AXES], axis=-1)
            batches = BATCHES
        body_gaps, padded_gaps = geometry_labels(
            trajectories, body, padded, actual)
        mask_path = args.mask_dir / (
            "captured_mask.txt" if seed == -1 else f"seed_{seed}_mask.txt")
        mask_sha256 = digest(mask_path)
        static_mask = np.loadtxt(mask_path, dtype=bool)
        if static_mask.shape != (len(body_gaps),):
            raise ValueError(f"static mask shape differs: {mask_path}")
        joint_safe = (body_gaps >= 0.05) & (padded_gaps > 0) & ~static_mask
        for batch in batches:
            name = "captured_batch300" if seed == -1 else f"seed_{seed}_batch{batch}"
            score_path = args.score_dir / f"{name}_cpp_scores.bin"
            scores = np.fromfile(score_path, dtype="<f4")
            if scores.shape != (batch,):
                raise ValueError(f"score shape differs: {score_path}")
            if int(np.sum(joint_safe[:batch])) != expected[name]["raw_joint_safe_count"]:
                raise ValueError(f"raw candidate labels changed: {name}")
            for factor in FACTORS:
                changed = dict(settings)
                changed["temperature"] = settings["temperature"] * factor
                result = aggregate(scores, controls[:batch], initial,
                                   changed, history)
                geometry = open_loop_geometry(result["filtered_sequence"],
                    meta, changed, actual, body, padded)
                if factor == 1.0:
                    reference = expected[name]["guarded_filtered_geometry"]
                    if abs(geometry["body_min_gap_m"] -
                           reference["body_min_gap_m"]) > 1e-4:
                        raise ValueError(f"baseline aggregate changed: {name}")
                probability = result["probability"]
                rows.append({
                    "case": name,
                    "batch": batch,
                    "temperature_factor": factor,
                    "temperature": float(changed["temperature"]),
                    "raw_joint_safe_count": int(np.sum(joint_safe[:batch])),
                    "static_mask_sha256": mask_sha256,
                    "raw_joint_safe_probability_mass": float(np.sum(
                        probability[joint_safe[:batch]], dtype=np.float32)),
                    "effective_sample_size": float(1.0 / np.sum(
                        probability ** 2, dtype=np.float32)),
                    "filtered_open_loop": geometry,
                    "returned_control": result["returned_control"],
                })
    summary = {
        "schema": "rm_dynamic_prediction/guarded_temperature_sensitivity/v1",
        "scope": "Frozen cycle 162, guarded C++ seven-critic scores, original sampled controls, constant V1 first-step penalty omitted from ranking. Truth only labels trajectories and evaluated filtered outputs. No new static CostCritic check on temperature-changed aggregates.",
        "factors": FACTORS,
        "input_sha256": {
            "cycle_json": digest(args.cycle),
            "cycle_bin": digest(args.cycle.with_suffix(".bin")),
            "profile": digest(args.profile),
            "truth": digest(args.truth),
            "history": digest(args.history),
            "guarded_summary": digest(args.guarded_summary),
            "scores": {row["case"]: digest(
                args.score_dir / f"{row['case']}_cpp_scores.bin")
                for row in rows if row["temperature_factor"] == 1.0},
        },
        "rows": rows,
        "dynamic_gate_pass_by_factor": {str(factor): sum(
            row["filtered_open_loop"]["dynamic_clearance_gate_met"]
            for row in rows if row["temperature_factor"] == factor)
            for factor in FACTORS},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(expected), "evaluations": len(rows),
                      "dynamic_gate_pass_by_factor": summary["dynamic_gate_pass_by_factor"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("cycle", "profile", "truth", "history", "score-dir",
                 "mask-dir", "guarded-summary", "output"):
        parser.add_argument("--" + flag, type=Path, required=True)
    run(parser.parse_args())
