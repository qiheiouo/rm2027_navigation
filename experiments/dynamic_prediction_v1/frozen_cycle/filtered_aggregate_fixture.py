#!/usr/bin/env python3
"""Export full filtered MPPI aggregates to the native costmap mask checker."""
import argparse
import json
from pathlib import Path

import numpy as np

import analyze
from batch_sampling_probe import BATCHES, digest, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_sensitivity import AXES, aggregate


def run(cycle, profile, history_file, score_root, output):
    if output.exists() or output.with_suffix(".json").exists():
        raise FileExistsError(output)
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    history_payload = json.loads(history_file.read_text())
    if history_payload["cycle_id"] != meta["cycle_id"]:
        raise ValueError("control history belongs to another cycle")
    history = np.asarray(history_payload["previous_outputs"],
                         dtype=np.float32)
    if history.shape != (4, 3):
        raise ValueError("four 3-axis historical controls required")
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    trajectories = [[], [], []]
    rows = []
    for seed in range(4):
        controls, _ = sample_omni(meta, arrays, 2000, seed)
        stacked = np.stack([controls[axis] for axis in AXES], axis=-1)
        for batch in BATCHES:
            scores_file = score_root / f"seed{seed}_batch{batch}/native_scores.bin"
            scores = np.fromfile(scores_file, dtype="<f4")
            if scores.shape != (batch,):
                raise ValueError(f"score count differs at {scores_file}")
            result = aggregate(scores, stacked[:batch], initial, settings,
                               history)
            trajectory = analyze.integrate_omni(
                *(result["filtered_sequence"][:, axis]
                  for axis in range(3)), meta["pose"], settings["dt"])
            for index, values in enumerate(trajectory):
                trajectories[index].append(values)
            rows.append({"seed": seed, "batch": batch,
                         "native_score_sha256": digest(scores_file)})
    threshold = shortcut_threshold(meta, profile)
    export(output, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack(axis) for axis in trajectories), threshold)
    payload = {
        "schema": "rm_dynamic_prediction_filtered_aggregate_costmap_fixture/v1",
        "scope": "Sixteen filtered aggregate open-loop trajectories from the frozen cycle, in seed-major batch order. Use with native costmap_mask_probe; original raw map and padded footprint are unchanged.",
        "cycle_json_sha256": digest(cycle),
        "cycle_bin_sha256": digest(cycle.with_suffix(".bin")),
        "profile_sha256": digest(profile),
        "history_sha256": digest(history_file),
        "fixture_sha256": digest(output),
        "possibly_inscribed_cost_threshold": threshold,
        "rows": rows,
    }
    output.with_suffix(".json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return payload


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycle", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--history", type=Path, required=True)
    parser.add_argument("--score-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    record = run(args.cycle, args.profile, args.history,
                 args.score_root, args.output)
    print(json.dumps({"output": str(args.output), "rows": len(record["rows"]),
                      "threshold": record["possibly_inscribed_cost_threshold"]}))
