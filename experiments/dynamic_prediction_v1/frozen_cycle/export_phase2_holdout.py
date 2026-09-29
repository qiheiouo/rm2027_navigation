#!/usr/bin/env python3
"""Export reproducible selected-cycle evidence from the phase-2 holdout."""
import argparse
import gzip
import json
from pathlib import Path
import shutil
import tempfile

import analyze
import select_cycle


HERE = Path(__file__).resolve().parent


def export(trial, failed_startup, output):
    if output.exists():
        raise FileExistsError(output)
    selection = json.loads((trial / "selection.json").read_text())
    repeated = select_cycle.select(trial, "/work/dynamic_prediction_trace_head_20260929/install/")
    if repeated != selection:
        raise ValueError("pre-registered selection no longer reproduces")
    cycle_id = selection["selected_cycle_id"]
    raw = trial / f"analysis_cycle_{cycle_id}_v2"
    filtered = trial / f"filtered_analysis_{cycle_id}"
    fixture = trial / f"filtered_fixture_{cycle_id}"
    raw_summary = json.loads((raw / "summary.json").read_text())
    filtered_summary = json.loads((filtered / "summary.json").read_text())
    if raw_summary["cycle_id"] != cycle_id or filtered_summary["cycle_id"] != cycle_id:
        raise ValueError("selected cycle analysis differs")
    if sum(1 for _ in (raw / "rollouts.csv").open()) != 301 or \
            analyze.sha(filtered / "candidates.csv") != filtered_summary["detail_sha256"]:
        raise ValueError("candidate tables differ")
    sources = {
        "plan.json": trial.parent / "plan.json",
        "profile.yaml": trial / "profile.yaml",
        "selection.json": trial / "selection.json",
        "runtime_summary.json": trial / "observation/summary.json",
        "events.jsonl": trial / "observation/events.jsonl",
        "writer_status.json": trial / "mppi_cycles/writer_status.json",
        "raw_summary.json": raw / "summary.json",
        "rank_probe_v2.json": trial / "rank_probe_v2.json",
        "filtered_summary.json": filtered / "summary.json",
        "filtered_candidates.csv": filtered / "candidates.csv",
        "filtered_inputs.json": fixture / "inputs.json",
        "filtered_meta.json": fixture / "filtered_meta.json",
        "filtered_map_and_poses.bin": fixture / "filtered_map_and_poses.bin",
        "filtered_controls.bin": fixture / "filtered_controls.bin",
        "filtered_native_scores.bin": fixture / "filtered_native_scores.bin",
        "filtered_static_mask.txt": fixture / "filtered_static_mask.txt",
        "raw_native_scores.bin": trial / f"native_raw_cycle_{cycle_id}/raw_scores.bin",
        "startup_failed_plan.json": failed_startup.parent / "plan.json",
        "startup_failed_observer.log": failed_startup / "observer.log",
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
            "loaded_maps.txt": trial / "mppi_cycles/loaded_maps.txt",
            "raw_rollouts.csv": raw / "rollouts.csv",
    }.items():
        target = output / (name + ".gz")
        with target.open("xb") as file:
            with gzip.GzipFile(filename="", fileobj=file, mode="wb", mtime=0) as stream:
                with path.open("rb") as original:
                    shutil.copyfileobj(original, stream)
        compressed[name] = {"source_sha256": analyze.sha(path),
                            "compressed_sha256": analyze.sha(target)}
    with tempfile.TemporaryDirectory() as temporary:
        truth = Path(temporary) / "gazebo_poses.jsonl"
        with gzip.open(output / "gazebo_poses.jsonl.gz", "rb") as source:
            truth.write_bytes(source.read())
        reproduced, records = analyze.analyze(output / f"cycle_{cycle_id}.json",
                                               output / "profile.yaml", truth)
    if {key: value for key, value in reproduced.items() if key != "source"} != {
            key: value for key, value in raw_summary.items() if key != "source"} or \
            len(records) != 300 or analyze.sha(trial / "gazebo_poses.jsonl") != \
            selection["gazebo_truth_sha256"]:
        raise ValueError("exported source cannot reproduce selected-cycle analysis")
    manifest = {
        "schema": "rm_dynamic_prediction_phase2_holdout_export/v1",
        "cycle_id": cycle_id, "all_rollouts_reproduced": len(records),
        "analysis_reproduced_from_export": True,
        "source_commit": json.loads((trial.parent / "plan.json").read_text())["run_commit"],
        "tools_sha256": {name: analyze.sha(HERE / name) for name in (
            "select_cycle.py", "analyze.py", "phase2_filtered_audit.py",
            "export_phase2_holdout.py")},
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
    parser.add_argument("failed_startup", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = export(args.trial, args.failed_startup, args.output)
    print(json.dumps({"cycle_id": result["cycle_id"],
                      "files": len(result["files_sha256"]),
                      "rollouts_reproduced": result["all_rollouts_reproduced"]}))
