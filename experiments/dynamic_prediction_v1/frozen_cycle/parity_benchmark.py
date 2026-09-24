#!/usr/bin/env python3
"""Compare all selected-cycle overlap scores with C++ and time scoring only."""
import argparse
import json
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

import numpy as np
import yaml

import analyze
import occupancy_rank_probe


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def run(evidence):
    meta, arrays = analyze.read_cycle(evidence / "cycle_162.json")
    prediction = analyze.event(meta, "prediction.input")
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("single-box fixture required")
    track = tracks[0]
    settings = analyze.event(meta, "settings")
    params = yaml.safe_load((evidence / "profile.yaml").read_text())[
        "controller_server"]["ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    expected = occupancy_rank_probe.expected_overlap(meta, arrays, params, order=5)
    x, y, yaw = (analyze.last(arrays, "rollout." + name)
                 for name in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(np.floor(params["horizon"] / settings["dt"] + 1e-9)))
    data = [" ".join(str(value) for value in (
        len(x), steps, len(meta["padded_footprint"]), prediction["source_age_s"],
        settings["dt"], params["object_width"], params["object_height"],
        params["reference_acceleration"], *track["xy"], *track["vxy"],
        *track["size_xy"]))]
    data += [f"{px:.17g} {py:.17g}" for px, py in meta["padded_footprint"]]
    data += [f"{float(x[i,j]):.17g} {float(y[i,j]):.17g} {float(yaw[i,j]):.17g}"
             for i in range(len(x)) for j in range(steps)]
    stream = "\n".join(data) + "\n"
    includes = ROOT / "experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/include"
    with TemporaryDirectory(prefix="rm2027_overlap_parity_") as directory:
        binary = Path(directory) / "overlap_score_probe"
        subprocess.run(["g++", "-std=c++17", "-O3", "-I", str(includes),
                        str(HERE / "overlap_score_probe.cpp"), "-o", str(binary)],
                       check=True)
        results = {}
        for batch in (300, 600, 1000, 2000):
            output = subprocess.check_output([str(binary), str(batch), "3"],
                                             input=stream, text=True)
            lines = output.splitlines()
            elapsed, checksum = map(float, lines[0].split())
            actual = np.asarray([float(value) for value in lines[1:]])
            results[str(batch)] = {"score_only_ms_per_batch": elapsed,
                                   "checksum": checksum,
                                   "duplicate_of_frozen_300": batch > 300}
            if batch == 300:
                results[str(batch)]["max_abs_cpp_python_score_error"] = float(
                    np.max(np.abs(actual - expected)))
                if results[str(batch)]["max_abs_cpp_python_score_error"] > 1e-10:
                    raise ValueError("C++/Python collision rank mismatch")
    return {"schema": "rm_dynamic_prediction_uniform_overlap_parity/v1",
            "cycle_sha256": analyze.sha(evidence / "cycle_162.json"),
            "geometry_source_sha256": analyze.sha(includes /
                "rm_dynamic_prediction_critic/geometry.hpp"),
            "score_steps": steps, "gauss_legendre_order_per_axis": 5,
            "benchmark_repetitions": 3,
            "scope": "Single-thread C++ geometry scoring only. Larger batches duplicate the frozen 300 trajectories; neither MPPI cost nor safe-rollout count is measured.",
            "batches": results}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("output exists")
    result = run(args.evidence)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
