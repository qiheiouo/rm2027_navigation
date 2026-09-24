#!/usr/bin/env python3
"""Audit one completed ranking trial without rerunning or changing its inputs."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
import yaml

from analyze import (critic_deltas, event, obstacle_polygon, placed,
                     polygon_distance, read_cycle, rows_from_transport, sha)
from dynamic_metrics import geometry_metrics
from envelope import predicted_box


def audit(trial):
    trial = Path(trial)
    plan = json.loads((trial.parent / "plan.json").read_text())
    profile_file = trial / "profile.yaml"
    if sha(profile_file) != plan["derived_profile_sha256"]:
        raise ValueError("profile changed after pre-registration")
    profile = yaml.safe_load(profile_file.read_text())
    follow = profile["controller_server"]["ros__parameters"]["FollowPath"]
    params = follow["PredictionV1Critic"]
    if params["collision_rank_mode"] != "uniform_center_overlap":
        raise ValueError("wrong collision rank mode")
    cycle_dir = trial / "mppi_cycles"
    writer = json.loads((cycle_dir / "writer_status.json").read_text())
    files = sorted(cycle_dir.glob("cycle_*.json"),
                   key=lambda p: int(p.stem.split("_")[1]))
    if not writer["closed"] or writer["dropped"] or writer["errors"] or \
            writer["attempted"] != writer["written"] or len(files) != writer["written"]:
        raise ValueError("incomplete trace writer")
    if [int(p.stem.split("_")[1]) for p in files] != list(range(len(files))):
        raise ValueError("missing or repeated cycle ID")
    maps = (cycle_dir / "loaded_maps.txt").read_text()
    for library, prefix in (("libmppi_controller.so", "dynamic_prediction_trace_v3"),
                            ("libmppi_critics.so", "dynamic_prediction_trace_v3"),
                            ("librm_dynamic_prediction_critic.so", "dynamic_prediction_rank_trace_v1")):
        if not any(f"/work/{prefix}/install/" in line and library in line
                   for line in maps.splitlines()):
            raise ValueError(f"wrong loaded library: {library}")
    events = [json.loads(line) for line in (trial / "observation/events.jsonl").open()]
    preflight = next(e for e in events if e.get("event") == "preflight")
    result = next(e for e in events if e.get("event") == "navigation_result")
    rows = [row for row in rows_from_transport(trial / "gazebo_poses.jsonl")
            if preflight["t"] <= row["t"] <= result["t"]]
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    if float(local["footprint_padding"]) != .03:
        raise ValueError("fixture padding changed")
    geometry = geometry_metrics(rows, body, .03, obstacle_polygon())
    durations = []
    statuses = Counter()
    near_goal = []
    goal_overlap = [0, 0]
    saturated = 0
    first_within_half_meter = None
    goal = result["goal"]
    for path in files:
        meta, arrays = read_cycle(path)
        if meta.get("unwinding_exception") or not path.with_suffix(".bin").is_file():
            raise ValueError(f"incomplete control cycle {path.name}")
        duration = (meta["finish_steady_ns"] - meta["request_steady_ns"]) / 1e6
        durations.append(duration)
        inputs = [entry["value"] for entry in meta["events"]
                  if entry["kind"] == "prediction.input"]
        if len(inputs) > 1:
            raise ValueError(f"multiple prediction inputs in {path.name}")
        prediction = inputs[0] if inputs else None
        statuses[prediction["status"] if prediction else "missing"] += 1
        distance = math.hypot(meta["pose"][0] - goal[0], meta["pose"][1] - goal[1])
        if meta["sim_ns"] / 1e9 >= 40:
            near_goal.append(distance)
            if prediction and prediction["status"] == "accepted":
                track = next((track for track in prediction["tracks"]
                              if track["state"] == 2), None)
                if track is not None:
                    dt = event(meta, "settings")["dt"]
                    box = predicted_box(track["xy"], track["vxy"], track["size_xy"],
                                        (params["object_width"], params["object_height"]),
                                        prediction["source_age_s"], dt,
                                        params["reference_acceleration"])
                    goal_shape = placed(meta["padded_footprint"], tuple(goal))
                    goal_overlap[1] += 1
                    goal_overlap[0] += polygon_distance(goal_shape, box.polygon()) <= 1e-9
        if distance <= .5 and first_within_half_meter is None:
            first_within_half_meter = {"cycle_id": meta["cycle_id"],
                                       "sim_s": meta["sim_ns"] / 1e9,
                                       "goal_distance_m": distance}
        if prediction and prediction["status"] == "accepted":
            terms, _ = critic_deltas(arrays)
            score = terms["FollowPath.PredictionV1Critic"]
            saturated += bool(np.all(score > 1000.))
    duration_array = np.asarray(durations)
    return {
        "schema": "rm_dynamic_prediction_uniform_rank_runtime_audit/v1",
        "trial": str(trial), "plan_sha256": sha(trial.parent / "plan.json"),
        "profile_sha256": sha(profile_file), "truth_sha256": sha(trial / "gazebo_poses.jsonl"),
        "writer_status": writer, "loaded_libraries_verified": True,
        "observer": json.loads((trial / "observation/summary.json").read_text()),
        "navigation_result": result, "dynamic_geometry": geometry,
        "prediction_status_counts": dict(statuses),
        "all_300_hard_predicted_collision_cycles": saturated,
        "full_compute_cycle_ms": {"p50": float(np.percentile(duration_array, 50)),
                                  "p95": float(np.percentile(duration_array, 95)),
                                  "p99": float(np.percentile(duration_array, 99)),
                                  "max": float(duration_array.max()),
                                  "over_100ms": int(np.count_nonzero(duration_array > 100))},
        "first_cycle_within_0_5m_goal": first_within_half_meter,
        "t_ge_40s_goal_distance_min_m": min(near_goal),
        "t_ge_40s_goal_padded_footprint_intersects_first_predicted_box":
            {"count": goal_overlap[0], "accepted_cycles": goal_overlap[1]},
        "limits": "One phase-4 trial; geometry uses sampled planar poses and a linear interpolation bound. This trial cannot establish phase-independent success or attribute the goal timeout to one critic."
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("audit output exists")
    result = audit(args.trial)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ("observer", "dynamic_geometry")}, indent=2))
