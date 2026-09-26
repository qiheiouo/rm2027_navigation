#!/usr/bin/env python3
"""Export frozen raw costmap and captured/synthetic rollouts for Nav2 checker."""
import argparse
import json
import math
from pathlib import Path
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import sample_omni


MAGIC = 0x43504D31


def export(path, meta, raw, trajectory, footprint_threshold):
    count, steps = trajectory[0].shape
    header = struct.pack(
        "<7I3d", MAGIC, meta["map"]["width"], meta["map"]["height"],
        count, steps, len(meta["padded_footprint"]), footprint_threshold,
        meta["map"]["resolution"], *meta["map"]["origin"])
    footprint = np.asarray(meta["padded_footprint"], dtype="<f8")
    poses = np.stack(trajectory, axis=-1).astype("<f4")
    with path.open("xb") as output:
        output.write(header)
        output.write(footprint.tobytes())
        output.write(np.asarray(raw, dtype=np.uint8).tobytes())
        output.write(poses.tobytes())


def run(cycle, output_dir, seeds):
    if output_dir.exists():
        raise FileExistsError(output_dir)
    meta, arrays = analyze.read_cycle(cycle)
    raw = analyze.last(arrays, "locked.raw_map")
    local = yaml.safe_load((cycle.parent.parent / "profile.yaml").read_text())[
        "local_costmap"]["local_costmap"]["ros__parameters"]
    footprint = meta["padded_footprint"]
    circumscribed = max(math.hypot(*point) for point in footprint)
    inscribed = min(abs(x1 * y2 - y1 * x2) /
                    math.hypot(x2 - x1, y2 - y1)
                    for (x1, y1), (x2, y2) in zip(
                        footprint, footprint[1:] + footprint[:1]))
    scaling = float(local["inflation_layer"]["cost_scaling_factor"])
    # Nav2's InflationLayer::computeCost(circumscribed_radius/resolution).
    threshold = int(252 * math.exp(-scaling *
                                   (circumscribed - inscribed)))
    output_dir.mkdir(parents=True)
    captured = tuple(analyze.last(arrays, "rollout." + axis)
                     for axis in ("x", "y", "yaw"))
    export(output_dir / "captured.bin", meta, raw, captured, threshold)
    for seed in seeds:
        _, sampled = sample_omni(meta, arrays, 2000, seed)
        export(output_dir / f"seed_{seed}.bin", meta, raw, sampled, threshold)
    return {"cycle": str(cycle), "seeds": seeds,
            "captured_count": captured[0].shape[0],
            "sampled_count": 2000,
            "footprint_vertices": len(footprint),
            "possibly_inscribed_cost_threshold": threshold}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycle", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3])
    args = parser.parse_args()
    print(json.dumps(run(args.cycle, args.output_dir, args.seeds)))
