#!/usr/bin/env python3
"""Prove or leave inconclusive step-2 predicted contact for all clipped MPPI samples."""
import argparse
import json
import math
from pathlib import Path
import subprocess

import numpy as np

import analyze
from batch_sampling_probe import digest
from phase2_filtered_audit import candidates, selected
from replay_ranking import FILTER

ROOT = Path(__file__).resolve().parents[3]


def first_filtered_bounds(history, lower, upper):
    fixed = float(np.sum(np.asarray(history, dtype=np.float32) * FILTER[:4],
                         dtype=np.float32))
    coefficients = FILTER[4:]
    minimum = fixed + sum(float(c * (lower if c >= 0 else upper))
                          for c in coefficients)
    maximum = fixed + sum(float(c * (upper if c >= 0 else lower))
                          for c in coefficients)
    return minimum, maximum


def inradius(polygon):
    if len(polygon) < 3:
        raise ValueError("not a polygon")
    radii = []
    crosses = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        ax, ay = a
        bx, by = b
        cross = ax * by - ay * bx
        crosses.append(cross)
        radii.append(abs(cross) / math.hypot(bx - ax, by - ay))
    if min(radii) <= 0 or not (all(x > 0 for x in crosses) or
                               all(x < 0 for x in crosses)):
        raise ValueError("origin is not strictly inside polygon")
    return min(radii)


def distance_to_box(x, y, box):
    dx = max(box["min_x"] - x, 0., x - box["max_x"])
    dy = max(box["min_y"] - y, 0., y - box["max_y"])
    return math.hypot(dx, dy)


def run(trial, rank_evidence, output):
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT,
                               text=True).strip():
        raise ValueError("reachability rule must be committed before evaluation")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                     text=True).strip()
    cycle, meta, arrays, settings, _, _ = selected(trial)
    if meta["cycle_id"] != 613 or settings["steps"] != 30:
        raise ValueError("wrong frozen cycle")
    summary_file = rank_evidence / "summary.json"
    rank = json.loads(summary_file.read_text())
    if rank["cycle_json_sha256"] != digest(cycle) or \
            rank["cycle_binary_sha256"] != digest(cycle.with_suffix(".bin")) or \
            rank["predicted_hits_by_step"][1] != 300:
        raise ValueError("frozen rank probe changed")
    if digest(rank_evidence / "candidates.csv") != rank["details_sha256"]:
        raise ValueError("candidate rank details changed")
    controls, _, poses, history = candidates(trial, meta, arrays, settings)
    limits = ((settings["vx_min"], settings["vx_max"]),
              (-settings["vy_max"], settings["vy_max"]),
              (-settings["wz_max"], settings["wz_max"]))
    bounds = [first_filtered_bounds(history[:, axis], *limits[axis])
              for axis in range(3)]
    dt = settings["dt"]
    current = np.asarray(meta["speed"], dtype=np.float32)
    first = analyze.integrate_omni(
        *(np.asarray([current[axis]], dtype=np.float32) for axis in range(3)),
        meta["pose"], dt)
    x1, y1, yaw1 = (float(axis[0]) for axis in first)
    if float(np.max(np.abs(poses[:, 0, :] - np.asarray((x1, y1, yaw1))))) > 1e-5:
        raise ValueError("saved filtered candidates do not share current-speed first step")
    box = rank["prediction_boxes"][1]
    corners = []
    for vx in bounds[0]:
        for vy in bounds[1]:
            x2 = x1 + dt * (vx * math.cos(yaw1) - vy * math.sin(yaw1))
            y2 = y1 + dt * (vx * math.sin(yaw1) + vy * math.cos(yaw1))
            corners.append({"vx": vx, "vy": vy, "center_x": x2, "center_y": y2,
                            "center_to_box_m": distance_to_box(x2, y2, box)})
    xlo, xhi = min(c["center_x"] for c in corners), max(c["center_x"] for c in corners)
    ylo, yhi = min(c["center_y"] for c in corners), max(c["center_y"] for c in corners)
    second = poses[:, 1, :2]
    if not np.all((second[:, 0] >= xlo - 1e-5) &
                  (second[:, 0] <= xhi + 1e-5) &
                  (second[:, 1] >= ylo - 1e-5) &
                  (second[:, 1] <= yhi + 1e-5)):
        raise ValueError("saved candidate escapes derived second-step center bounds")
    radius = inradius(meta["padded_footprint"])
    maximum_distance = max(c["center_to_box_m"] for c in corners)
    report = {"schema": "rm_dynamic_prediction_cycle613_step2_reachability/v1",
              "scope": "Deterministic overapproximation of all clipped sample sequences on one already inspected frozen cycle. A strict inradius witness proves predicted contact only, not physical collision.",
              "evaluation_commit": commit,
              "input_sha256": {"cycle_json": digest(cycle),
                               "cycle_binary": digest(cycle.with_suffix(".bin")),
                               "rank_summary": digest(summary_file),
                               "rank_candidates": digest(rank_evidence / "candidates.csv"),
                               "previous_cycles": {
                                   str(i): digest(trial / f"mppi_cycles/cycle_{i}.json")
                                   for i in range(609, 613)}},
              "first_pose_from_current_speed": [x1, y1, yaw1],
              "first_filtered_velocity_bounds": {axis: list(bounds[i]) for i, axis in
                                                  enumerate(("vx", "vy", "wz"))},
              "second_step_box": box,
              "reachable_center_corners": corners,
              "observed_second_center_bounds": {
                  "x": [float(np.min(second[:, 0])), float(np.max(second[:, 0]))],
                  "y": [float(np.min(second[:, 1])), float(np.max(second[:, 1]))]},
              "padded_polygon_inradius_m": radius,
              "max_reachable_center_to_box_m": maximum_distance,
              "strict_all_controls_predicted_contact_proved": maximum_distance < radius}
    output.mkdir(parents=True)
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({key: report[key] for key in (
        "first_pose_from_current_speed", "first_filtered_velocity_bounds",
        "second_step_box", "observed_second_center_bounds",
        "padded_polygon_inradius_m", "max_reachable_center_to_box_m",
        "strict_all_controls_predicted_contact_proved")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("trial", "rank_evidence", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    run(args.trial, args.rank_evidence, args.output)
