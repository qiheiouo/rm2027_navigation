#!/usr/bin/env python3
"""Export the one-shot south-start selected cycle and offline diagnostics."""
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
    selected = json.loads((trial / "selection.json").read_text())
    if selected["selected_cycle_id"] != 65 or \
            select_cycle.select(trial, "/work/dynamic_prediction_trace_head_20260929/install/") != selected:
        raise ValueError("opposite-side selected cycle changed")
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
           "docker_exit.txt": trial / "docker_exit.txt",
           "observer_exit.txt": trial / "observer_exit.txt",
           "raw_summary.json": trial / "analysis_cycle_65/summary.json",
           "raw_rollouts.csv": trial / "analysis_cycle_65/rollouts.csv",
           "batch_result.json": trial / "batch_result_65.json",
           "noise_result.json": trial / "noise_result_65.json",
           "scale4_v1_hits.json": trial / "scale4_v1_hits_65.json",
           "score_prefix_result.json": trial / "score_prefix_result_65.json"}
    for name, source in top.items():
        add(source, name)
    for index in range(61, 66):
        for suffix in ("json", "bin"):
            name = f"cycle_{index}.{suffix}"
            add(trial / "mppi_cycles" / name, "cycles/" + name,
                suffix == "bin")
    for name, source in {
            "gazebo_poses.jsonl": trial / "gazebo_poses.jsonl",
            "predictions.jsonl": trial / "predictions.jsonl",
            "scans.jsonl": trial / "observation/scans.jsonl",
            "trajectory.jsonl": trial / "observation/trajectory.jsonl",
            "diagnostics.jsonl": trial / "diagnostics.jsonl",
            "launch.log": trial / "launch.log",
            "observer.log": trial / "observer.log"}.items():
        add(source, "streams/" + name, True)
    for name in ("worlds/crossing.sdf", "models/x_axis_obstacle.sdf",
                 "base_gazebo.launch.py", "comparison.launch.py",
                 "reference.launch.py", "dynamic_near_x.launch.py",
                 "run_near_x.sh"):
        add(root / name, "scenario/" + name)
    for directory in ("native_raw_cycle_65", "filtered_fixture_65",
                      "batch_inputs_65", "batch_masks_65",
                      "noise_inputs_65", "noise_masks_65", "score_prefix_65"):
        for source in sorted((trial / directory).rglob("*")):
            if source.is_file():
                add(source, "derived/" + str(source.relative_to(trial)),
                    source.suffix in (".bin", ".npz"))
    native = root.parent / "phase2_native_mask_head_20260929/install/costmap_mask_probe_cpp/lib/costmap_mask_probe_cpp"
    for name in ("costmap_mask_probe", "frozen_critic_score"):
        add(native / name, "native/" + name, True)
    with tempfile.TemporaryDirectory() as temp:
        cycle = Path(temp) / "cycle_65.json"
        shutil.copyfile(output / "cycles/cycle_65.json", cycle)
        with gzip.open(output / "cycles/cycle_65.bin.gz", "rb") as stream:
            cycle.with_suffix(".bin").write_bytes(stream.read())
        truth = Path(temp) / "truth.jsonl"
        with gzip.open(output / "streams/gazebo_poses.jsonl.gz", "rb") as stream:
            truth.write_bytes(stream.read())
        raw, rows = analyze.analyze(cycle, output / "profile.yaml", truth)
    original = json.loads((trial / "analysis_cycle_65/summary.json").read_text())
    if {key: value for key, value in raw.items() if key != "source"} != \
            {key: value for key, value in original.items() if key != "source"} or \
            len(rows) != 300:
        raise ValueError("exported south-start raw analysis differs")
    manifest = {"schema": "rm_dynamic_prediction_near_x_south_export/v1",
                "run_commit": json.loads((root / "plan.json").read_text())["run_commit"],
                "selected_cycle": 65, "raw_analysis_reproduced": True,
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
