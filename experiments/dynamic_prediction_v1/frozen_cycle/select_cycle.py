#!/usr/bin/env python3
"""Apply the pre-registered single-cycle selection rule to one completed trial."""
import argparse
import json
from pathlib import Path

import yaml

from analyze import event, sha, rows_from_transport, placed, obstacle_polygon, polygon_distance
from read_trace import read_cycle


def select(trial):
    cycle_dir = trial / "mppi_cycles"
    status = json.loads((cycle_dir / "writer_status.json").read_text())
    if not status["closed"] or status["dropped"] or status["errors"] or \
            status["attempted"] != status["written"]:
        raise ValueError(f"incomplete cycle writer: {status}")
    maps = (cycle_dir / "loaded_maps.txt").read_text()
    prefix = "/work/dynamic_prediction_trace_v3/install/"
    for library in ("libmppi_controller.so", "libmppi_critics.so",
                    "librm_dynamic_prediction_critic.so"):
        if not any(prefix in line and library in line for line in maps.splitlines()):
            raise ValueError(f"wrong diagnostic library mapping: {library}")
    profile = yaml.safe_load((trial / "profile.yaml").read_text())
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    if abs(float(local["footprint_padding"]) - .03) > 1e-9:
        raise ValueError("footprint padding changed")
    truth_file = trial / "gazebo_poses.jsonl"
    rows = rows_from_transport(truth_file)
    box = obstacle_polygon()
    samples = [(row["t"], polygon_distance(placed(body, row["robot"]),
                                          placed(box, row["obstacle"]))) for row in rows]
    first_breach = next(((t, gap) for t, gap in samples if gap < .05), None)
    witness = first_breach if first_breach else min(samples, key=lambda item: item[1])
    cutoff = witness[0] - .25
    candidates = []
    all_files = list(cycle_dir.glob("cycle_*.json"))
    if len(all_files) != status["written"]:
        raise ValueError("cycle JSON count differs from writer status")
    for path in all_files:
        meta, arrays = read_cycle(path)
        if meta.get("unwinding_exception") or not path.with_suffix(".bin").is_file():
            continue
        kinds = [e["kind"] for e in meta["events"]]
        if not all(k in kinds for k in ("settings", "prediction.input", "output")):
            continue
        if event(meta, "prediction.input").get("status") != "accepted":
            continue
        if not all(any(name == required for name, _ in arrays) for required in
                   ("locked.raw_map", "rollout.x", "rollout.y", "rollout.yaw",
                    "scored.costs", "weighted.probability")):
            continue
        t = meta["sim_ns"] / 1e9
        if t <= cutoff:
            candidates.append((t, path, meta))
    if not candidates:
        raise ValueError("no complete accepted cycle before fixed cutoff")
    t, path, meta = max(candidates, key=lambda item: item[0])
    return {
        "schema": "rm_dynamic_prediction_cycle_selection/v1",
        "rule": "first sampled body gap <0.05 m, else sampled minimum; latest complete accepted cycle >=0.25 s earlier",
        "witness_kind": "first_breach" if first_breach else "minimum",
        "witness_sim_s": witness[0], "witness_body_gap_m": witness[1],
        "cutoff_sim_s": cutoff, "selected_cycle_sim_s": t,
        "selected_cycle_id": meta["cycle_id"], "selected_cycle_json": str(path),
        "selected_cycle_sha256": sha(path),
        "selected_binary_sha256": sha(path.with_suffix(".bin")),
        "gazebo_truth_sha256": sha(truth_file),
        "all_cycle_count": len(all_files), "accepted_before_cutoff_count": len(candidates),
        "writer_status": status,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    args = parser.parse_args()
    result = select(args.trial)
    path = args.trial / "selection.json"
    with path.open("x") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(json.dumps(result, indent=2))
