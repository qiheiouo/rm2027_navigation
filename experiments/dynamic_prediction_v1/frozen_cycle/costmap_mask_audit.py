#!/usr/bin/env python3
"""Audit native frozen CostCritic masks and intersect with dynamic labels."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni


def run(trial, cycle_id, mask_dir, sampling_evidence, executable, image_id):
    cycle = trial / "mppi_cycles" / f"cycle_{cycle_id}.json"
    meta, arrays = analyze.read_cycle(cycle)
    original = np.asarray(analyze.last(arrays, "cost_critic.collisions"),
                          dtype=bool)
    captured_path = mask_dir / "captured_mask.txt"
    mirrored = np.loadtxt(captured_path, dtype=bool)
    if mirrored.shape != original.shape:
        raise ValueError("native baseline mask length differs")
    mismatches = np.flatnonzero(mirrored != original)
    if len(mismatches):
        raise ValueError(f"native mask differs at {mismatches.tolist()}")
    sampling = json.loads(sampling_evidence.read_text())
    if sampling["input_sha256"]["cycle_json"] != digest(cycle):
        raise ValueError("sampling evidence uses another frozen cycle")
    if sampling["input_sha256"]["cycle_bin"] != digest(
            cycle.with_suffix(".bin")):
        raise ValueError("sampling evidence binary changed")
    settings = analyze.event(meta, "settings")
    truth_path = trial / "gazebo_poses.jsonl"
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    prediction = analyze.event(meta, "prediction.input")
    consumed = prediction["consumer_sim_s"]
    profile = yaml.safe_load((trial / "profile.yaml").read_text())
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    padded = meta["padded_footprint"]
    physical = analyze.obstacle_polygon()
    actual = [analyze.placed(physical, analyze.interpolated_pose(
        truth, times, consumed + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    trials = []
    for item in sampling["trials"]:
        seed = item["seed"]
        _, trajectory = sample_omni(meta, arrays, BATCHES[-1], seed)
        body_gaps, padded_gaps = geometry_labels(
            trajectory, body, padded, actual)
        safe = (body_gaps >= .05) & (padded_gaps > 0)
        mask_path = mask_dir / f"seed_{seed}_mask.txt"
        mask = np.loadtxt(mask_path, dtype=bool)
        if mask.shape != safe.shape:
            raise ValueError(f"seed {seed}: native mask length differs")
        prefixes = {}
        for batch in BATCHES:
            key = str(batch)
            if int(safe[:batch].sum()) != item["prefixes"][key][
                    "true_dynamic_safe_count"]:
                raise ValueError(f"seed {seed}: dynamic labels changed")
            prefixes[key] = {
                "dynamic_safe": int(safe[:batch].sum()),
                "costcritic_collision": int(mask[:batch].sum()),
                "dynamic_safe_and_costcritic_clear": int(np.sum(
                    safe[:batch] & ~mask[:batch])),
            }
        trials.append({"seed": seed, "mask_sha256": digest(mask_path),
                       "prefixes": prefixes})
    return {
        "schema": "rm_dynamic_prediction_native_costmap_mask_audit/v1",
        "scope": "Frozen raw costmap, padded footprint and 30-step trajectories. Installed Nav2 FootprintCollisionChecker with source-matched CostCritic inflation shortcut. Baseline 300 collision marks must match the captured mask exactly. New sampled trajectories are truth-labeled offline; no other critic or final control is run. This is not a full MPPI replay or control-cycle timing.",
        "image_id": image_id,
        "native_executable_sha256": digest(executable),
        "native_source_sha256": digest(Path(__file__).parent /
                                        "costmap_mask_probe_cpp/src/costmap_mask_probe.cpp"),
        "fixture_exporter_sha256": digest(Path(__file__).parent /
                                           "costmap_mask_fixture.py"),
        "sampling_evidence_sha256": digest(sampling_evidence),
        "raw_cycle_sha256": digest(cycle),
        "captured_mask_sha256": digest(captured_path),
        "captured_costcritic_collisions": int(original.sum()),
        "captured_mask_mismatch_count": int(len(mismatches)),
        "trials": trials,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--cycle-id", type=int, default=162)
    parser.add_argument("--mask-dir", type=Path, required=True)
    parser.add_argument("--sampling-evidence", type=Path, required=True)
    parser.add_argument("--native-executable", type=Path, required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.trial, args.cycle_id, args.mask_dir,
                 args.sampling_evidence, args.native_executable, args.image_id)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"captured_mismatches": result[
        "captured_mask_mismatch_count"], "output": str(args.output)}))
