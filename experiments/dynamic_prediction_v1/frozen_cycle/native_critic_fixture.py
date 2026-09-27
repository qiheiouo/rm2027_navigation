#!/usr/bin/env python3
"""Prepare frozen Nav2 critic inputs without changing the runtime profile."""
import argparse
import json
from pathlib import Path
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import sample_omni
from costmap_mask_fixture import run as export_map


AXES = ("cvx", "cvy", "cwz")
EXCLUDED = {"PredictionV1Critic"}


def parameters(profile, batch):
    source = profile["controller_server"]["ros__parameters"]
    follow = source["FollowPath"]
    result = {"controller_frequency": source["controller_frequency"]}
    for name, value in follow.items():
        if name == "critics":
            value = [critic for critic in value if critic not in EXCLUDED]
        if name == "batch_size":
            value = batch
        if isinstance(value, dict):
            if name in EXCLUDED:
                continue
            for key, nested in value.items():
                if isinstance(nested, (bool, int, float, str)):
                    result[f"FollowPath.{name}.{key}"] = nested
        elif isinstance(value, (bool, int, float, str, list)):
            result[f"FollowPath.{name}"] = value
    return result


def costmap_parameters(profile):
    source = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    result = {}
    for name, value in source.items():
        if isinstance(value, dict):
            for key, nested in value.items():
                if isinstance(nested, (bool, int, float, str)):
                    result[f"{name}.{key}"] = nested
        elif isinstance(value, (bool, int, float, str, list)):
            result[name] = value
    return result


def controls_file(path, arrays, count, steps):
    controls = np.stack(arrays, axis=-1).astype("<f4")
    if controls.shape != (count, steps, 3):
        raise ValueError("sampled control dimensions differ")
    with path.open("xb") as output:
        output.write(struct.pack("<3I", 0x43545231, count, steps))
        output.write(controls.tobytes())


def run(cycle, output_dir, seed=None, batch=300, profile_path=None):
    if output_dir.exists():
        raise FileExistsError(output_dir)
    meta, arrays = analyze.read_cycle(cycle)
    if profile_path is None:
        profile_path = cycle.parent.parent / "profile.yaml"
    profile = yaml.safe_load(profile_path.read_text())
    if seed is None:
        if batch != 300:
            raise ValueError("captured replay requires original 300 batch")
        sampled = tuple(analyze.last(arrays, "sampled." + axis)
                        for axis in AXES)
        export_map(cycle, output_dir, [], profile_path)
        map_data_file = "captured.bin"
    else:
        sampled, _ = sample_omni(meta, arrays, 2000, seed)
        sampled = tuple(sampled[axis][:batch] for axis in ("vx", "vy", "wz"))
        export_map(cycle, output_dir, [seed], profile_path)
        map_data_file = f"seed_{seed}.bin"
    controls_file(output_dir / "controls.bin", sampled, batch,
                  analyze.event(meta, "settings")["steps"])
    payload = {"pose": meta["pose"], "speed": meta["speed"],
               "path": meta["path"], "map_frame": meta["map"]["frame"],
               "parameters": parameters(profile, batch),
               "costmap_parameters": costmap_parameters(profile),
               "seed": seed, "batch": batch, "map_data_file": map_data_file}
    (output_dir / "meta.json").write_text(json.dumps(payload, indent=2) + "\n")
    return {"output_dir": str(output_dir), "batch": batch, "seed": seed}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycle", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--batch", type=int, default=300)
    parser.add_argument("--profile", type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.cycle, args.output_dir, args.seed,
                         args.batch, args.profile)))
