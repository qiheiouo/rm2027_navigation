#!/usr/bin/env python3
"""Locate the first physical dynamic gate violation in fixed sample fixtures."""
import argparse
import json
from pathlib import Path
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from near_x_batch_sensitivity import EXPECTED, SEEDS, frozen


HEADER = "<7I3d"


def trajectory(path, meta):
    with path.open("rb") as stream:
        header = struct.unpack(HEADER, stream.read(struct.calcsize(HEADER)))
        magic, width, height, count, steps, vertices, _, resolution, ox, oy = header
        expected = meta["map"]
        if (magic, width, height, count, steps, vertices) != (
                0x43504D31, expected["width"], expected["height"], 2000, 30,
                len(meta["padded_footprint"])) or abs(resolution - expected["resolution"]) > 1e-9:
            raise ValueError("native map fixture header differs")
        footprint = np.fromfile(stream, dtype="<f8", count=2 * vertices).reshape(-1, 2)
        raw = np.fromfile(stream, dtype=np.uint8, count=width * height)
        pose = np.fromfile(stream, dtype="<f4", count=count * steps * 3)
    if pose.size != count * steps * 3 or raw.size != width * height or \
            not np.allclose(footprint, meta["padded_footprint"]):
        raise ValueError("native map fixture data differs")
    return pose.reshape(count, steps, 3)


def run(trial, fixture):
    paths, meta, arrays, settings = frozen(trial)
    record = json.loads((fixture / "inputs.json").read_text())
    if record["inputs_sha256"] != EXPECTED:
        raise ValueError("wrong fixture inputs")
    profile = yaml.safe_load(paths["profile"].read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    padded = meta["padded_footprint"]
    truth = analyze.rows_from_transport(paths["truth"])
    times = [row["t"] for row in truth]
    consumer = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, times,
                  consumer + (j + 1) * settings["dt"]))
              for j in range(settings["steps"])]
    rows = []
    for seed in SEEDS:
        path = fixture / f"seed_{seed}_map.bin"
        if digest(path) != record["files_sha256"][path.name]:
            raise ValueError("native map fixture changed")
        pose = trajectory(path, meta)
        body_gap = np.empty((2000, 30))
        padded_gap = np.empty_like(body_gap)
        for j, box in enumerate(actual):
            for i in range(2000):
                position = tuple(float(v) for v in pose[i, j])
                body_gap[i, j] = analyze.polygon_distance(
                    analyze.placed(body, position), box)
                padded_gap[i, j] = analyze.polygon_distance(
                    analyze.placed(padded, position), box)
        with np.load(fixture / f"seed_{seed}_gaps.npz") as gaps:
            if not np.allclose(body_gap.min(axis=1), gaps["body"]) or \
                    not np.allclose(padded_gap.min(axis=1), gaps["padded"]):
                raise ValueError("stepwise geometry disagrees with preregistered labels")
        violations = (body_gap < .05) | (padded_gap <= 0)
        first = np.argmax(violations, axis=1) + 1
        first[~violations.any(axis=1)] = 0
        rows.append({"seed": seed,
                     "first_violation_step_histogram": {
                         str(step): int(np.sum(first == step)) for step in range(31)
                         if np.any(first == step)},
                     "body_gate_pass_by_step": np.sum(body_gap >= .05, axis=0).tolist(),
                     "padded_clear_by_step": np.sum(padded_gap > 0, axis=0).tolist(),
                     "best_body_gap_by_step_m": body_gap.max(axis=0).tolist(),
                     "all_prefix_dynamic_safe": bool(np.any(first == 0))})
    return {"schema": "rm_dynamic_prediction_near_x_first_conflict/v1",
            "scope": "Stepwise physical truth on the four fixed filtered 2000-row fixtures; diagnostics only, not online prediction or a reachability proof.",
            "cycle_sha256": EXPECTED["cycle"],
            "fixture_inputs_sha256": digest(fixture / "inputs.json"),
            "rows": rows}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = run(args.trial, args.fixture)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"first_violation_by_seed": [row[
        "first_violation_step_histogram"] for row in result["rows"]]}))
