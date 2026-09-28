#!/usr/bin/env python3
"""Audit all nine V1 occupancy steps for the frozen 17-batch native replay."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from conflict_exposure_probe import exposure
from batch_sampling_probe import digest


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    meta, _ = analyze.read_cycle(args.cycle)
    params = yaml.safe_load(args.profile.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    reference = json.loads(args.native_summary.read_text())
    rows = []
    for case in reference["rows"]:
        name, batch = case["name"], case["batch"]
        poses_file = args.native_results / f"{name}_poses.bin"
        scores_file = args.native_results / f"{name}_prediction_scores.bin"
        if (digest(poses_file) != case["native_file_sha256"]["poses"] or
                digest(scores_file) != case["native_file_sha256"][
                    "prediction_scores"]):
            raise ValueError(f"native replay artifact changed: {name}")
        poses = np.fromfile(poses_file, dtype="<f4").reshape(batch, 30, 3)
        scores = np.fromfile(scores_file, dtype="<f4")
        if scores.shape != (batch,):
            raise ValueError(f"V1 score count differs: {name}")
        fractions, hits = exposure(
            meta, [("rollout.x", poses[:, :, 0]),
                   ("rollout.y", poses[:, :, 1]),
                   ("rollout.yaw", poses[:, :, 2])], params)
        rows.append({"name": name, "batch": batch,
                     "poses_sha256": digest(poses_file),
                     "prediction_scores_sha256": digest(scores_file),
                     "hits_by_step": hits,
                     "all_nine_steps_hit_count": int(np.count_nonzero(
                         fractions == 1.0)),
                     "hit_fraction_min": float(fractions.min()),
                     "hit_fraction_max": float(fractions.max()),
                     "V1_score_span": float(np.ptp(scores))})
    report = {"schema": "rm_dynamic_prediction/conflict_exposure_batch/v1",
              "scope": "Fixed cycle 162 native replay, 17 original or fixed-seed 300/600/1000/2000 batches. Same consumed V1 prediction, padded footprint, frozen controls and native rollout poses; no future truth in occupancy check.",
              "cycle_sha256": digest(args.cycle),
              "profile_sha256": digest(args.profile),
              "native_summary_sha256": digest(args.native_summary),
              "cases": rows,
              "total_rollouts": sum(row["batch"] for row in rows)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "total_rollouts": report[
        "total_rollouts"], "all_saturated": all(
            row["all_nine_steps_hit_count"] == row["batch"]
            for row in rows)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cycle", "profile", "native-summary", "native-results",
                 "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
