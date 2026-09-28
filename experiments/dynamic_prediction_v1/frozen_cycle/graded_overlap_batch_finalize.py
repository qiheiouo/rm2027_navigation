#!/usr/bin/env python3
"""Join graded-overlap frozen aggregate results with native CostCritic mask."""
import argparse
import json
from pathlib import Path

import numpy as np

from batch_sampling_probe import digest


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    report = json.loads(args.summary.read_text())
    if digest(args.fixture) != report["static_fixture_sha256"]:
        raise ValueError("aggregate fixture changed")
    mask = np.loadtxt(args.mask, dtype=bool)
    if mask.shape != (len(report["rows"]),):
        raise ValueError("native CostCritic mask row count differs")
    for row, collision in zip(report["rows"], mask):
        geometry = row[args.geometry_key]
        geometry["costcritic_collision"] = bool(collision)
        geometry["joint_clearance_gate_met"] = bool(
            geometry["dynamic_clearance_gate_met"] and not collision)
    report["native_static_check"] = {
        "geometry_key": args.geometry_key,
        "checker_sha256": digest(args.checker),
        "fixture_sha256": digest(args.fixture),
        "mask_sha256": digest(args.mask),
        "costcritic_collision_count": int(mask.sum()),
        "joint_clearance_gate_pass_count": sum(
            row[args.geometry_key]["joint_clearance_gate_met"]
            for row in report["rows"]),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report["native_static_check"]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("summary", "fixture", "mask", "checker", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--geometry-key", default="graded_geometry")
    run(parser.parse_args())
