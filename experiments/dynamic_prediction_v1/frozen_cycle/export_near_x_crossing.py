#!/usr/bin/env python3
"""Export reproducible near-X selected-cycle evidence with hashes."""
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
    if select_cycle.select(trial, "/work/dynamic_prediction_trace_head_20260929/install/") != selection or \
            selection["selected_cycle_id"] != 73:
        raise ValueError("preselected near-X cycle changed")
    output.mkdir(parents=True)
    files = {}

    def add(source, name, compress=False):
        if not source.is_file():
            raise FileNotFoundError(source)
        target = output / (name + ".gz" if compress else name)
        target.parent.mkdir(parents=True, exist_ok=True)
        if compress:
            with target.open("xb") as result:
                with gzip.GzipFile(filename="", fileobj=result,
                                   mode="wb", mtime=0) as stream:
                    with source.open("rb") as original:
                        shutil.copyfileobj(original, stream)
        else:
            shutil.copyfile(source, target)
        files[str(target.relative_to(output))] = {
            "source": str(source), "source_sha256": analyze.sha(source),
            "artifact_sha256": analyze.sha(target), "gzip": compress}

    root = trial.parent
    top = {"plan.json": root / "plan.json",
           "profile.yaml": trial / "profile.yaml",
           "selection.json": trial / "selection.json",
           "runtime_summary.json": trial / "observation/summary.json",
           "events.jsonl": trial / "observation/events.jsonl",
           "writer_status.json": trial / "mppi_cycles/writer_status.json",
           "physical_tail.log": trial / "physical_tail.log",
           "physical_tail_exit.txt": trial / "physical_tail_exit.txt",
           "docker_exit.txt": trial / "docker_exit.txt",
           "observer_exit.txt": trial / "observer_exit.txt",
           "raw_summary.json": trial / "analysis_cycle_73/summary.json",
           "raw_rollouts.csv": trial / "analysis_cycle_73/rollouts.csv",
           "filtered_summary.json": trial / "filtered_analysis_73/summary.json",
           "filtered_candidates.csv": trial / "filtered_analysis_73/candidates.csv",
           "batch_result.json": trial / "batch_sensitivity_result_73.json",
           "first_conflict_result.json": trial / "first_conflict_result_73.json",
           "boundary_result.json": trial / "boundary_grid_result_73.json",
           "noise_result.json": trial / "noise_scale_result_73.json",
           "scale4_score_summary.json": trial / "scale4_score_result_73/summary.json",
           "scan_transfer_summary.json": trial / "fixed_scan_transfer_73/summary.json",
           "scan_transfer_candidates.csv": trial / "fixed_scan_transfer_73/candidates.csv"}
    for name, source in top.items():
        add(source, name)
    for index in range(69, 74):
        for suffix in ("json", "bin"):
            name = f"cycle_{index}.{suffix}"
            add(trial / "mppi_cycles" / name, "cycles/" + name, suffix == "bin")
    for name, source in {
            "gazebo_poses.jsonl": trial / "gazebo_poses.jsonl",
            "predictions.jsonl": trial / "predictions.jsonl",
            "scans.jsonl": trial / "observation/scans.jsonl",
            "trajectory.jsonl": trial / "observation/trajectory.jsonl",
            "diagnostics.jsonl": trial / "diagnostics.jsonl",
            "launch.log": trial / "launch.log",
            "observer.log": trial / "observer.log",
    }.items():
        add(source, "streams/" + name, True)
    for name in ("worlds/crossing.sdf", "models/x_axis_obstacle.sdf",
                 "base_gazebo.launch.py", "comparison.launch.py",
                 "reference.launch.py", "dynamic_near_x.launch.py",
                 "run_near_x.sh"):
        add(root / name, "scenario/" + name)
    groups = {
        "filtered_fixture_73": False,
        "native_raw_cycle_73": False,
        "fixed_scan_transfer_73": False,
        "batch_sensitivity_inputs_73": False,
        "batch_sensitivity_masks_73": False,
        "boundary_grid_inputs_73": False,
        "noise_scale_inputs_73": False,
        "noise_scale_masks_73": False,
        "scale4_score_inputs_73": False,
        "scale4_native_scores_73": False,
        "scale4_score_result_73": False,
    }
    for directory in groups:
        for source in sorted((trial / directory).rglob("*")):
            if not source.is_file() or source.name in ("summary.json", "transfer.json"):
                continue
            relative = source.relative_to(trial)
            add(source, "derived/" + str(relative),
                source.suffix in (".bin", ".npz"))
    native = root.parent / "phase2_native_mask_head_20260929/install/costmap_mask_probe_cpp/lib/costmap_mask_probe_cpp"
    for name in ("costmap_mask_probe", "frozen_critic_score"):
        add(native / name, "native/" + name, True)
    with tempfile.TemporaryDirectory() as temp:
        truth = Path(temp) / "truth.jsonl"
        cycle = Path(temp) / "cycle_73.json"
        shutil.copyfile(output / "cycles/cycle_73.json", cycle)
        with gzip.open(output / "cycles/cycle_73.bin.gz", "rb") as stream:
            cycle.with_suffix(".bin").write_bytes(stream.read())
        with gzip.open(output / "streams/gazebo_poses.jsonl.gz", "rb") as stream:
            truth.write_bytes(stream.read())
        raw, rows = analyze.analyze(cycle,
                                    output / "profile.yaml", truth)
    original = json.loads((trial / "analysis_cycle_73/summary.json").read_text())
    if {k: v for k, v in raw.items() if k != "source"} != \
            {k: v for k, v in original.items() if k != "source"} or len(rows) != 300:
        raise ValueError("exported raw-cycle replay differs")
    manifest = {"schema": "rm_dynamic_prediction_near_x_export/v1",
                "run_commit": json.loads((root / "plan.json").read_text())["run_commit"],
                "selected_cycle": 73, "raw_analysis_reproduced": True,
                "raw_rollouts": len(rows), "files": files}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2,
                                               sort_keys=True) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = export(args.trial, args.output)
    print(json.dumps({"files": len(result["files"]),
                      "raw_rollouts": result["raw_rollouts"]}))
