#!/usr/bin/env python3
"""Test a soft occupancy rank over the same conservative physical support.

The hard V1 envelope is unchanged. A physical box center is integrated over
the center locations compatible with the visible cluster and model mismatch.
This is a read-only ranking study, not an online probability guarantee.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
import rank_probe
from envelope import AxisBox


def expected_overlap(meta, arrays, params, order=5):
    prediction = analyze.event(meta, "prediction.input")
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("single-box fixture required")
    track = tracks[0]
    dt = analyze.event(meta, "settings")["dt"]
    x, y, yaw = (analyze.last(arrays, "rollout." + key)
                 for key in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(np.floor(float(params["horizon"]) / dt + 1e-9)))
    footprint = meta["padded_footprint"]
    robot_area = rank_probe.area(footprint)
    if robot_area <= 0:
        raise ValueError("invalid footprint")
    nodes, weights = np.polynomial.legendre.leggauss(order)
    weights /= 2.
    extent = (float(params["object_width"]), float(params["object_height"]))
    score = np.zeros(x.shape[0], dtype=np.float64)
    for j in range(steps):
        duration = prediction["source_age_s"] + (j + 1) * dt
        mismatch = .5 * float(params["reference_acceleration"]) * duration ** 2
        center = [track["xy"][axis] + track["vxy"][axis] * duration
                  for axis in (0, 1)]
        support = [track["size_xy"][axis] / 2 + extent[axis] / 2 + mismatch
                   for axis in (0, 1)]
        physical_boxes = []
        for nx, wx in zip(nodes, weights):
            for ny, wy in zip(nodes, weights):
                cx, cy = center[0] + nx * support[0], center[1] + ny * support[1]
                physical_boxes.append((AxisBox(
                    cx - extent[0] / 2, cy - extent[1] / 2,
                    cx + extent[0] / 2, cy + extent[1] / 2), wx * wy))
        for i in range(x.shape[0]):
            polygon = analyze.placed(footprint, (float(x[i, j]), float(y[i, j]),
                                                 float(yaw[i, j])))
            score[i] += sum(weight * rank_probe.overlap_area(polygon, box)
                            for box, weight in physical_boxes) / robot_area / steps
    return score


def evaluate(trial):
    selection = json.loads((trial / "selection.json").read_text())
    start, end = selection["witness_sim_s"] - 2., selection["witness_sim_s"]
    profile = trial / "profile.yaml"
    truth = trial / "gazebo_poses.jsonl"
    params = yaml.safe_load(profile.read_text())["controller_server"][
        "ros__parameters"]["FollowPath"]["PredictionV1Critic"]
    rows = []
    for path in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta, arrays = analyze.read_cycle(path)
        t = meta["sim_ns"] / 1e9
        if not start <= t <= end or not any(e["kind"] == "prediction.input"
                                            for e in meta["events"]):
            continue
        summary, records = analyze.analyze(path, profile, truth)
        if summary["predicted_collision_rollouts"] != summary["batch_size"] or \
                not summary["safe_truth_and_costmap_rollouts"]:
            continue
        safe = np.asarray([r["truth_dynamic_safe"] and not r["costcritic_collision"]
                           for r in records], dtype=bool)
        hard = rank_probe.overlap_fractions(meta, arrays, params)
        soft = expected_overlap(meta, arrays, params)

        def auc(values):
            good, bad = values[safe], values[~safe]
            return float((np.count_nonzero(good[:, None] < bad[None, :]) +
                          .5 * np.count_nonzero(good[:, None] == bad[None, :])) /
                         (len(good) * len(bad)))

        rows.append({"cycle_id": summary["cycle_id"],
                     "cycle_sha256": summary["source"]["cycle_sha256"],
                     "safe_count": int(safe.sum()),
                     "hard_envelope_overlap_auc": auc(hard),
                     "uniform_center_overlap_auc": auc(soft),
                     "hard_top30_safe": int(safe[np.argsort(hard)[:30]].sum()),
                     "uniform_top30_safe": int(safe[np.argsort(soft)[:30]].sum()),
                     "uniform_score_span": float(np.ptp(soft))})
    return {"schema": "rm_dynamic_prediction_soft_occupancy_rank_probe/v1",
            "scope": "One captured trial, all saturated cycles with true-safe candidates in fixed 2 s window. Five-point Gauss-Legendre quadrature per axis; no online changes.",
            "window_sim_s": [start, end], "cycles": rows,
            "uniform_auc_better_cycles": sum(r["uniform_center_overlap_auc"] >
                                             r["hard_envelope_overlap_auc"] for r in rows),
            "uniform_auc_worse_cycles": sum(r["uniform_center_overlap_auc"] <
                                            r["hard_envelope_overlap_auc"] for r in rows)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("output exists")
    result = evaluate(args.trial)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
