#!/usr/bin/env python3
"""Check original V1 hard-collision saturation on south-start scale-four samples."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from near_x_scale4_score import controls_for, prediction_hits, raw_poses
from near_x_south_batch import SEEDS, frozen


def run(trial, noise_result):
    cycle, meta, arrays, settings, profile_path, _ = frozen(trial)
    noise = json.loads(noise_result.read_text())
    if noise["schema"] != "rm_dynamic_prediction_south_noise_sensitivity/v1" or \
            len(noise["rows"]) != 8:
        raise ValueError("opposite-side noise result differs")
    params = yaml.safe_load(profile_path.read_text())[
        "controller_server"]["ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    captured = analyze.critic_deltas(arrays)[0]["FollowPath.PredictionV1Critic"]
    if np.ptp(captured) > 1e-3:
        raise ValueError("captured legacy hard term is not tied")
    rows = []
    for seed in SEEDS:
        controls = controls_for(meta, arrays, seed)
        hits = prediction_hits(meta, raw_poses(controls, meta, settings), params)
        if hits[0] != 2000:
            raise ValueError("first-step V1 hard tie no longer holds")
        rows.append({"seed": seed, "batch": 2000,
                     "predicted_hits_by_step": hits,
                     "predicted_any_collision_count": 2000,
                     "legacy_v1_score_identical_for_all": True})
    return {"schema": "rm_dynamic_prediction_south_scale4_v1_hits/v1",
            "scope": "Original accepted tracker input and V1 legacy hard-collision rule on four fixed offline raw sampled trajectory sets. Native seven-critic replay gate on this cycle differs for two captured rows, so no full score ranking or aggregate claim.",
            "cycle_sha256": digest(cycle),
            "noise_result_sha256": digest(noise_result),
            "captured_v1_score_span": float(np.ptp(captured)),
            "rows": rows}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--noise-result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.trial, args.noise_result)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"hits_by_seed": [row["predicted_hits_by_step"]
                      for row in result["rows"]]}))
