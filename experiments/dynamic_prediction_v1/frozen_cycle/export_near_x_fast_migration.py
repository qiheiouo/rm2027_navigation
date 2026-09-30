#!/usr/bin/env python3
"""Export the selected fast-X trial for git-based device migration."""
import argparse
import gzip
import json
from pathlib import Path
import shutil
import tempfile

import analyze
import select_cycle


EXPECTED_CYCLE = 47
EXPECTED_TRIAL_SHA = "b5de3aaa439d999b4eec1ab2b142a1bb26516dd00703c218d2044342547c902f"
PREFIX = "/work/dynamic_prediction_trace_head_20260929/install/"


def export(series, output):
    if output.exists():
        raise FileExistsError(output)
    trial = series / "candidate_navfn_1"
    plan = json.loads((series / "plan.json").read_text())
    selected = json.loads((trial / "selection.json").read_text())
    if plan["scenario"] != "north-start-x-slider-period-6s" or \
            plan["moving_box_period_s"] != 6.0 or \
            selected["selected_cycle_id"] != EXPECTED_CYCLE or \
            select_cycle.select(trial, PREFIX) != selected or \
            analyze.sha(trial / "gazebo_poses.jsonl") != EXPECTED_TRIAL_SHA:
        raise ValueError("fixed fast-X selection or physical source changed")
    writer = json.loads((trial / "mppi_cycles/writer_status.json").read_text())
    tail = json.loads((trial / "physical_tail.log").read_text())
    if writer != {"attempted": 538, "closed": True, "dropped": 0,
                  "errors": 0, "written": 538} or not tail["tail_complete"]:
        raise ValueError("writer or physical tail changed")
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

    top = {"plan.json": series / "plan.json",
           "profile.yaml": trial / "profile.yaml",
           "selection.json": trial / "selection.json",
           "writer_status.json": trial / "mppi_cycles/writer_status.json",
           "runtime_summary.json": trial / "observation/summary.json",
           "events.jsonl": trial / "observation/events.jsonl",
           "physical_tail.log": trial / "physical_tail.log",
           "docker_exit.txt": trial / "docker_exit.txt",
           "observer_exit.txt": trial / "observer_exit.txt",
           "raw_summary.json": trial / "analysis_cycle_47/summary.json",
           "raw_rollouts.csv": trial / "analysis_cycle_47/rollouts.csv",
           "coverage_result.json": trial / "coverage_result_47.json"}
    for name, source in top.items():
        add(source, name)
    for index in range(43, 48):
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
        add(series / name, "scenario/" + name)
    for directory in ("native_raw_cycle_47", "coverage_inputs_47",
                      "coverage_masks_47"):
        for source in sorted((trial / directory).rglob("*")):
            if source.is_file():
                add(source, "derived/" + str(source.relative_to(trial)),
                    source.suffix in (".bin", ".npz"))
    with tempfile.TemporaryDirectory() as temp:
        temp_root = Path(temp)
        cycle = temp_root / "cycle_47.json"
        shutil.copyfile(output / "cycles/cycle_47.json", cycle)
        with gzip.open(output / "cycles/cycle_47.bin.gz", "rb") as stream:
            cycle.with_suffix(".bin").write_bytes(stream.read())
        truth = temp_root / "truth.jsonl"
        with gzip.open(output / "streams/gazebo_poses.jsonl.gz", "rb") as stream:
            truth.write_bytes(stream.read())
        raw, rows = analyze.analyze(cycle, output / "profile.yaml", truth)
    original = json.loads((trial / "analysis_cycle_47/summary.json").read_text())
    if {key: value for key, value in raw.items() if key != "source"} != \
            {key: value for key, value in original.items() if key != "source"} or \
            len(rows) != 300:
        raise ValueError("exported fast-X raw analysis differs")
    report = {"schema": "rm_dynamic_prediction_fast_x_migration_export/v1",
              "run_commit": plan["run_commit"],
              "selected_cycle": EXPECTED_CYCLE,
              "raw_analysis_reproduced": True,
              "raw_rollouts": len(rows), "files": files}
    (output / "manifest.json").write_text(json.dumps(report, indent=2,
                                               sort_keys=True) + "\n")
    print(json.dumps({"files": len(files), "raw_rollouts": len(rows)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    export(args.series, args.output)
