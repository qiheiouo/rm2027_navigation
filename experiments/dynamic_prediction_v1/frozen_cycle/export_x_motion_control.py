#!/usr/bin/env python3
"""Archive the one-shot X-motion control trial, including incomplete truth window."""
import argparse
import gzip
import json
from pathlib import Path
import shutil

import analyze
import select_cycle


def export(trial, output):
    if output.exists():
        raise FileExistsError(output)
    selection = json.loads((trial / "selection.json").read_text())
    if select_cycle.select(trial, "/work/dynamic_prediction_trace_head_20260929/install/") != selection:
        raise ValueError("frozen selection changed")
    writer = json.loads((trial / "mppi_cycles/writer_status.json").read_text())
    if not writer["closed"] or writer["dropped"] or writer["errors"] or \
            writer["attempted"] != writer["written"]:
        raise ValueError("incomplete MPPI trace writer")
    rows = analyze.rows_from_transport(trial / "gazebo_poses.jsonl")
    available = rows[-1]["t"] - selection["selected_cycle_sim_s"]
    if available >= .9:
        raise ValueError("this archive is for the predefined incomplete-horizon result")
    cycle_id = selection["selected_cycle_id"]
    sources = {
        "plan.json": trial.parent / "plan.json",
        "profile.yaml": trial / "profile.yaml",
        "selection.json": trial / "selection.json",
        "writer_status.json": trial / "mppi_cycles/writer_status.json",
        "loaded_maps.txt": trial / "mppi_cycles/loaded_maps.txt",
        "runtime_summary.json": trial / "observation/summary.json",
        "events.jsonl": trial / "observation/events.jsonl",
        "docker_exit.txt": trial / "docker_exit.txt",
        "observer_exit.txt": trial / "observer_exit.txt",
        "observer.log": trial / "observer.log",
        "x_axis_obstacle.sdf": trial.parent / "models/x_axis_obstacle.sdf",
        "dynamic_x.launch.py": trial.parent / "dynamic_x.launch.py",
        "run_x.sh": trial.parent / "run_x.sh",
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
    }.items():
        target = output / (name + ".gz")
        with target.open("xb") as file:
            with gzip.GzipFile(filename="", fileobj=file, mode="wb", mtime=0) as stream:
                with path.open("rb") as original:
                    shutil.copyfileobj(original, stream)
        compressed[name] = {"source_sha256": analyze.sha(path),
                            "compressed_sha256": analyze.sha(target)}
    manifest = {
        "schema": "rm_dynamic_prediction_x_motion_control_export/v1",
        "scope": "First and only X-motion control trial; selected cycle has insufficient physical future for candidate evaluation.",
        "run_commit": json.loads((trial.parent / "plan.json").read_text())["run_commit"],
        "selected_cycle_id": cycle_id,
        "selected_cycle_reproduced": True,
        "writer_status": writer,
        "physical_truth_start_s": rows[0]["t"],
        "physical_truth_end_s": rows[-1]["t"],
        "selected_cycle_sim_s": selection["selected_cycle_sim_s"],
        "available_future_s": available,
        "required_v1_future_s": .9,
        "required_mppi_future_s": 3.0,
        "candidate_geometry_evaluable": False,
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
    print(json.dumps({"cycle_id": result["selected_cycle_id"],
                      "available_future_s": result["available_future_s"],
                      "files": len(result["files_sha256"])}))
