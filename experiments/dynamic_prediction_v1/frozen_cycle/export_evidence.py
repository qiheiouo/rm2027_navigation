#!/usr/bin/env python3
"""Export the one selected frozen cycle and the truth window for reproduction."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import analyze
from dynamic_metrics import stamp


ROOT = Path(__file__).resolve().parents[3]


def export(trial, output):
    if output.exists():
        raise FileExistsError("evidence directory exists")
    selection = json.loads((trial / "selection.json").read_text())
    cycle = ROOT / selection["selected_cycle_json"]
    cycle_id = selection["selected_cycle_id"]
    analysis_dir = trial / f"analysis_cycle_{cycle_id}_v2"
    summary = json.loads((analysis_dir / "summary.json").read_text())
    if analyze.sha(cycle) != selection["selected_cycle_sha256"] or \
            analyze.sha(cycle.with_suffix(".bin")) != selection["selected_binary_sha256"]:
        raise ValueError("selected cycle changed after audit")
    if summary["cycle_id"] != selection["selected_cycle_id"]:
        raise ValueError("analysis and selection differ")
    output.mkdir(parents=True)
    sources = {
        "plan.json": trial.parent / "plan.json",
        "profile.yaml": trial / "profile.yaml",
        "selection.json": trial / "selection.json",
        cycle.name: cycle,
        cycle.with_suffix(".bin").name: cycle.with_suffix(".bin"),
        "summary.json": analysis_dir / "summary.json",
        "rollouts.csv": analysis_dir / "rollouts.csv",
        "rank_probe_v2.json": trial / "rank_probe_v2.json",
        "writer_status.json": trial / "mppi_cycles/writer_status.json",
        "loaded_maps.txt": trial / "mppi_cycles/loaded_maps.txt",
    }
    for name, source in sources.items():
        shutil.copyfile(source, output / name)
    meta, _ = analyze.read_cycle(cycle)
    dt = analyze.event(meta, "settings")["dt"]
    t0 = summary["consumer_sim_s"] - .2
    t1 = summary["consumer_sim_s"] + summary["steps"] * dt + .2
    count = 0
    with (output / "gazebo_poses_window.jsonl").open("x") as dest:
        for line in (trial / "gazebo_poses.jsonl").open():
            t = stamp(json.loads(line))
            if t0 <= t <= t1:
                dest.write(line)
                count += 1
    if count < 2:
        raise ValueError("selected truth window empty")
    # Ensure the exported window, rather than the full untracked trial, can
    # reproduce every selected-cycle metric and all 300 rollout records.
    reproduced, records = analyze.analyze(output / cycle.name,
                                           output / "profile.yaml",
                                           output / "gazebo_poses_window.jsonl")
    for key, value in summary.items():
        if key in ("source", "limits"):
            continue
        if reproduced[key] != value:
            raise ValueError(f"exported evidence changed analysis: {key}")
    if len(records) != summary["batch_size"]:
        raise ValueError("rollout count changed")
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(output.iterdir()) if p.is_file()}
    manifest = {"schema": "rm_dynamic_prediction_selected_cycle_export/v1",
                "selected_cycle_id": selection["selected_cycle_id"],
                "truth_window_lines": count,
                "all_rollouts_reproduced": len(records),
                "metrics_reproduced_from_export": True,
                "files_sha256": hashes}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = export(args.trial, args.output)
    print(json.dumps({k: v for k, v in result.items() if k != "files_sha256"},
                     indent=2))
