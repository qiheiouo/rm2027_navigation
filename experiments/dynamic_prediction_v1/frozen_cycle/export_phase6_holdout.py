#!/usr/bin/env python3
"""Export the single selected phase-6 trial and reproduce its raw-cycle audit."""
import argparse
import gzip
import json
from pathlib import Path
import shutil
import tempfile

import analyze
import select_cycle


def export(trial, output):
    if output.exists():
        raise FileExistsError(output)
    selection = json.loads((trial / "selection.json").read_text())
    if select_cycle.select(trial, "/work/dynamic_prediction_trace_head_20260929/install/") != selection:
        raise ValueError("pre-registered cycle selection changed")
    cycle_id = selection["selected_cycle_id"]
    raw = trial / f"analysis_cycle_{cycle_id}"
    filtered = trial / f"filtered_analysis_{cycle_id}"
    fixture = trial / f"filtered_fixture_{cycle_id}"
    transfer = trial / f"fixed_transfer_{cycle_id}"
    if json.loads((raw / "summary.json").read_text())["cycle_id"] != cycle_id or \
            json.loads((filtered / "summary.json").read_text())["cycle_id"] != cycle_id or \
            json.loads((transfer / "summary.json").read_text())["cycle_id"] != cycle_id:
        raise ValueError("analysis cycle differs from frozen selection")
    sources = {
        "plan.json": trial.parent / "plan.json",
        "profile.yaml": trial / "profile.yaml",
        "selection.json": trial / "selection.json",
        "runtime_summary.json": trial / "observation/summary.json",
        "events.jsonl": trial / "observation/events.jsonl",
        "writer_status.json": trial / "mppi_cycles/writer_status.json",
        "loaded_maps.txt": trial / "mppi_cycles/loaded_maps.txt",
        "docker_exit.txt": trial / "docker_exit.txt",
        "observer_exit.txt": trial / "observer_exit.txt",
        "raw_summary.json": raw / "summary.json",
        "raw_rollouts.csv": raw / "rollouts.csv",
        "filtered_summary.json": filtered / "summary.json",
        "filtered_candidates.csv": filtered / "candidates.csv",
        "filtered_inputs.json": fixture / "inputs.json",
        "filtered_meta.json": fixture / "filtered_meta.json",
        "filtered_map_and_poses.bin": fixture / "filtered_map_and_poses.bin",
        "filtered_controls.bin": fixture / "filtered_controls.bin",
        "filtered_native_scores.bin": fixture / "filtered_native_scores.bin",
        "filtered_static_mask.txt": fixture / "filtered_static_mask.txt",
        "raw_native_meta.json": trial / f"native_raw_cycle_{cycle_id}/meta.json",
        "raw_native_map.bin": trial / f"native_raw_cycle_{cycle_id}/captured.bin",
        "raw_native_controls.bin": trial / f"native_raw_cycle_{cycle_id}/controls.bin",
        "raw_native_scores.bin": trial / f"native_raw_cycle_{cycle_id}/raw_scores.bin",
        "transfer_summary.json": transfer / "summary.json",
        "transfer_candidates.csv": transfer / "candidates.csv",
        "transfer_aggregate_map.bin": transfer / "aggregate_map.bin",
        "transfer_aggregate_static_mask.txt": transfer / "aggregate_static_mask.txt",
        "observer.log": trial / "observer.log",
    }
    for predecessor in range(cycle_id - 4, cycle_id + 1):
        for suffix in (".json", ".bin"):
            name = f"cycle_{predecessor}{suffix}"
            sources[name] = trial / "mppi_cycles" / name
    output.mkdir(parents=True)
    for name, path in sources.items():
        shutil.copyfile(path, output / name)
    compressed = {}
    for name, path in {
            "gazebo_poses.jsonl": trial / "gazebo_poses.jsonl",
            "predictions.jsonl": trial / "predictions.jsonl",
            "scans.jsonl": trial / "observation/scans.jsonl",
            "trajectory.jsonl": trial / "observation/trajectory.jsonl",
            "launch.log": trial / "launch.log",
            "diagnostics.jsonl": trial / "diagnostics.jsonl",
    }.items():
        target = output / (name + ".gz")
        with target.open("xb") as file:
            with gzip.GzipFile(filename="", fileobj=file, mode="wb", mtime=0) as stream:
                with path.open("rb") as original:
                    shutil.copyfileobj(original, stream)
        compressed[name] = {"source_sha256": analyze.sha(path),
                            "compressed_sha256": analyze.sha(target)}
    with tempfile.TemporaryDirectory() as temp:
        truth = Path(temp) / "gazebo_poses.jsonl"
        with gzip.open(output / "gazebo_poses.jsonl.gz", "rb") as file:
            truth.write_bytes(file.read())
        reproduced, records = analyze.analyze(output / f"cycle_{cycle_id}.json",
                                               output / "profile.yaml", truth)
    original = json.loads((raw / "summary.json").read_text())
    if {key: value for key, value in reproduced.items() if key != "source"} != {
            key: value for key, value in original.items() if key != "source"} or \
            len(records) != 300 or analyze.sha(trial / "gazebo_poses.jsonl") != \
            selection["gazebo_truth_sha256"]:
        raise ValueError("exported inputs do not reproduce raw 300-rollout audit")
    manifest = {
        "schema": "rm_dynamic_prediction_phase6_holdout_export/v1",
        "cycle_id": cycle_id,
        "run_commit": json.loads((trial.parent / "plan.json").read_text())["run_commit"],
        "evaluation_commit": json.loads((transfer / "summary.json").read_text())["evaluation_commit"],
        "selected_cycle_reproduced": True,
        "raw_analysis_reproduced_from_export": True,
        "raw_rollout_count": len(records),
        "original_trial": str(trial),
        "compressed_streams": compressed,
        "files_sha256": {path.name: analyze.sha(path) for path in sorted(output.iterdir())
                         if path.is_file()},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2,
                                                sort_keys=True) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = export(args.trial, args.output)
    print(json.dumps({"cycle_id": result["cycle_id"],
                      "files": len(result["files_sha256"]),
                      "raw_rollouts_reproduced": result["raw_rollout_count"]}))
