#!/usr/bin/env python3
"""Export the timeout's first half-meter cycle and its replayable truth window."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import analyze
from dynamic_metrics import stamp
import numpy as np


def export(trial, output):
    trial, output = Path(trial), Path(output)
    if output.exists():
        raise FileExistsError("evidence directory exists")
    audit = json.loads((trial / "runtime_audit.json").read_text())
    cycle_id = audit["first_cycle_within_0_5m_goal"]["cycle_id"]
    cycle = trial / "mppi_cycles" / f"cycle_{cycle_id}.json"
    analysis_dir = trial / f"analysis_cycle_{cycle_id}"
    summary = json.loads((analysis_dir / "summary.json").read_text())
    if summary["cycle_id"] != cycle_id or summary["source"]["cycle_sha256"] != analyze.sha(cycle):
        raise ValueError("diagnostic cycle and analysis differ")
    output.mkdir(parents=True)
    sources = {
        "plan.json": trial.parent / "plan.json",
        "profile.yaml": trial / "profile.yaml",
        "runtime_audit.json": trial / "runtime_audit.json",
        "observer_summary.json": trial / "observation/summary.json",
        "writer_status.json": trial / "mppi_cycles/writer_status.json",
        "loaded_maps.txt": trial / "mppi_cycles/loaded_maps.txt",
        cycle.name: cycle,
        cycle.with_suffix(".bin").name: cycle.with_suffix(".bin"),
        "summary.json": analysis_dir / "summary.json",
        "rollouts.csv": analysis_dir / "rollouts.csv",
    }
    for name, source in sources.items():
        shutil.copyfile(source, output / name)
    meta, _ = analyze.read_cycle(cycle)
    dt = analyze.event(meta, "settings")["dt"]
    t0, t1 = summary["consumer_sim_s"] - .2, summary["consumer_sim_s"] + summary["steps"] * dt + .2
    count = 0
    with (output / "gazebo_poses_window.jsonl").open("x") as dest:
        for line in (trial / "gazebo_poses.jsonl").open():
            if t0 <= stamp(json.loads(line)) <= t1:
                dest.write(line)
                count += 1
    reproduced, records = analyze.analyze(output / cycle.name, output / "profile.yaml",
                                           output / "gazebo_poses_window.jsonl")
    if count < 2 or len(records) != 300:
        raise ValueError("incomplete exported cycle or truth")
    for key, value in summary.items():
        if key not in ("source", "limits") and reproduced[key] != value:
            raise ValueError(f"exported evidence changed analysis: {key}")
    _, arrays = analyze.read_cycle(output / cycle.name)
    goal = audit["navigation_result"]["goal"]
    final_distance = np.hypot(analyze.last(arrays, "rollout.x")[:, -1] - goal[0],
                              analyze.last(arrays, "rollout.y")[:, -1] - goal[1])
    final_yaw_error = np.abs(np.remainder(analyze.last(arrays, "rollout.yaw")[:, -1]
                                          - goal[2] + np.pi, 2 * np.pi) - np.pi)
    close = [r for r in records if final_distance[r["rollout"]] <= .15]
    reachability = {
        "criterion": "rollout final position within 0.15 m of goal; yaw assessed separately",
        "goal_position_rollouts": len(close),
        "goal_position_and_yaw_0_2rad_rollouts": int(np.count_nonzero(
            (final_distance <= .15) & (final_yaw_error <= .2))),
        "goal_position_truth_safe_rollouts": sum(r["truth_dynamic_safe"] and
                                                   not r["costcritic_collision"] for r in close),
        "goal_position_predicted_collision_rollouts": sum(
            r["first_predicted_conflict_s"] is not None for r in close),
        "goal_position_probability_mass": sum(r["mppi_probability"] for r in close),
        "best_goal_position_total_score": min((r["total_score"] for r in close), default=None),
        "best_all_total_score": min(r["total_score"] for r in records),
    }
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(output.iterdir()) if p.is_file()}
    root = Path(__file__).resolve().parents[3]
    code = ("experiments/dynamic_prediction_v1/frozen_cycle/analyze.py",
            "experiments/dynamic_prediction_v1/frozen_cycle/runtime_audit.py",
            "experiments/dynamic_prediction_v1/frozen_cycle/export_rank_evidence.py")
    manifest = {
        "schema": "rm_dynamic_prediction_uniform_rank_runtime_evidence/v1",
        "scope": "One phase-4 trial timed out. Selected cycle is the first captured pose within 0.5 m of the goal, chosen to diagnose the timeout.",
        "diagnostic_cycle_id": cycle_id,
        "truth_window_lines": count,
        "all_rollouts_reproduced": len(records),
        "metrics_reproduced_from_export": True,
        "goal_reachability": reachability,
        "files_sha256": hashes,
        "analysis_code_sha256": {name: analyze.sha(root / name) for name in code},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = export(args.trial, args.output)
    print(json.dumps({key: value for key, value in result.items() if key != "files_sha256"}, indent=2))
