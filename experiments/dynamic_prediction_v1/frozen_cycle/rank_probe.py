#!/usr/bin/env python3
"""Offline, read-only test of a graded overlap signal on the captured cycles.

This does not alter the running critic or tune a weight. Future Gazebo poses are
used only to label which sampled rollouts satisfy the unchanged 0.05 m gate.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze


def area(polygon):
    return abs(sum(a[0] * b[1] - a[1] * b[0]
                   for a, b in zip(polygon, polygon[1:] + polygon[:1]))) / 2 \
        if len(polygon) >= 3 else 0.


def clip(polygon, axis, limit, keep_above):
    def inside(point):
        return point[axis] >= limit if keep_above else point[axis] <= limit

    result = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        a_inside, b_inside = inside(a), inside(b)
        if a_inside != b_inside:
            fraction = (limit - a[axis]) / (b[axis] - a[axis])
            result.append((a[0] + fraction * (b[0] - a[0]),
                           a[1] + fraction * (b[1] - a[1])))
        if b_inside:
            result.append(b)
    return result


def overlap_area(polygon, box):
    polygon = list(polygon)
    for axis, limit, keep_above in ((0, box.min_x, True),
                                    (0, box.max_x, False),
                                    (1, box.min_y, True),
                                    (1, box.max_y, False)):
        if not polygon:
            return 0.
        polygon = clip(polygon, axis, limit, keep_above)
    return area(polygon)


def overlap_fractions(meta, arrays, params):
    prediction = analyze.event(meta, "prediction.input")
    dt = analyze.event(meta, "settings")["dt"]
    x, y, yaw = (analyze.last(arrays, "rollout." + key)
                 for key in ("x", "y", "yaw"))
    steps = min(x.shape[1], int(np.floor(float(params["horizon"]) / dt + 1e-9)))
    footprint = meta["padded_footprint"]
    footprint_area = area(footprint)
    if footprint_area <= 0:
        raise ValueError("invalid padded footprint")
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("this ranking probe requires the single-box fixture")
    track = tracks[0]
    scores = np.zeros(x.shape[0], dtype=np.float64)
    for j in range(steps):
        box = analyze.predicted_box(
            track["xy"], track["vxy"], track["size_xy"],
            (params["object_width"], params["object_height"]),
            prediction["source_age_s"], (j + 1) * dt,
            params["reference_acceleration"])
        for i in range(x.shape[0]):
            robot = analyze.placed(footprint, (float(x[i, j]), float(y[i, j]),
                                               float(yaw[i, j])))
            scores[i] += overlap_area(robot, box) / footprint_area / steps
    return scores


def grade(cycle, profile, truth):
    summary, records = analyze.analyze(cycle, profile, truth)
    meta, arrays = analyze.read_cycle(cycle)
    params = yaml.safe_load(profile.read_text())["controller_server"]["ros__parameters"][
        "FollowPath"]["PredictionV1Critic"]
    scores = overlap_fractions(meta, arrays, params)
    safe = np.array([row["truth_dynamic_safe"] and not row["costcritic_collision"]
                     for row in records], dtype=bool)
    order = np.argsort(scores, kind="stable")
    # Fixed scale: reuse the existing 300-unit near-sample penalty. This is
    # an offline counterfactual, not a fitted parameter or runtime change.
    gradient = (3.81 / 254.) * 300. * scores
    weighted = np.array([row["weighted_score"] for row in records]) + gradient
    counterfactual_order = np.argsort(weighted, kind="stable")
    temperature = analyze.event(meta, "settings")["temperature"]
    probability = np.exp(-(weighted - np.min(weighted)) / temperature)
    probability /= probability.sum()
    good, bad = scores[safe], scores[~safe]
    auc = None if not len(good) or not len(bad) else float(
        (np.count_nonzero(good[:, None] < bad[None, :]) +
         .5 * np.count_nonzero(good[:, None] == bad[None, :])) /
        (len(good) * len(bad)))
    return {"cycle_id": summary["cycle_id"], "sim_s": summary["consumer_sim_s"],
            "cycle_sha256": summary["source"]["cycle_sha256"],
            "safe_count": int(safe.sum()),
            "predicted_collision_count": summary["predicted_collision_rollouts"],
            "original_v1_score_span": summary["prediction_score_span"],
            "overlap_fraction_span": float(np.ptp(scores)),
            "safe_in_overlap_top_30": int(safe[order[:30]].sum()),
            "safe_in_fixed_scale_counterfactual_top_30": int(
                safe[counterfactual_order[:30]].sum()),
            "safe_probability_mass_fixed_scale_counterfactual": float(
                probability[safe].sum()),
            "safe_probability_mass_original": summary["safe_probability_mass"],
            "safe_in_original_total_top_30": int(sum(
                row["truth_dynamic_safe"] and not row["costcritic_collision"]
                for row in sorted(records, key=lambda row: row["total_score"])[:30])),
            "overlap_safe_auc": auc}


def probe(trial):
    selection = json.loads((trial / "selection.json").read_text())
    window_start = selection["witness_sim_s"] - 2.
    window_end = selection["witness_sim_s"]
    profile = trial / "profile.yaml"
    truth = trial / "gazebo_poses.jsonl"
    cycles = []
    rejected = []
    for path in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda item: int(item.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        t = meta["sim_ns"] / 1e9
        if not window_start <= t <= window_end:
            continue
        if not any(e["kind"] == "prediction.input" for e in meta["events"]):
            rejected.append({"cycle": str(path), "reason": "prediction not accepted"})
            continue
        cycles.append(grade(path, profile, truth))
    saturated_with_safe = [row for row in cycles if row["safe_count"] and
                           row["predicted_collision_count"] == 300]
    return {"schema": "rm_dynamic_prediction_graded_overlap_probe/v1",
            "scope": "All captured control cycles in the fixed 2 s window before the first body-clearance breach; offline signal only.",
            "window_sim_s": [window_start, window_end],
            "selected_cycle_id": selection["selected_cycle_id"],
            "cycles": cycles, "rejected": rejected,
            "saturated_cycles_with_true_safe_rollout": len(saturated_with_safe),
            "cycles_where_overlap_top_30_beats_original_top_30": sum(
                row["safe_in_overlap_top_30"] > row["safe_in_original_total_top_30"]
                for row in saturated_with_safe),
            "cycles_where_overlap_top_30_loses": sum(
                row["safe_in_overlap_top_30"] < row["safe_in_original_total_top_30"]
                for row in saturated_with_safe),
            "cycles_where_fixed_scale_safe_mass_increases": sum(
                row["safe_probability_mass_fixed_scale_counterfactual"] >
                row["safe_probability_mass_original"]
                for row in saturated_with_safe)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("--output", default="rank_probe.json")
    args = parser.parse_args()
    result = probe(args.trial)
    output = args.trial / args.output
    with output.open("x") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(json.dumps({k: v for k, v in result.items() if k != "cycles"}, indent=2))
    for row in result["cycles"]:
        print(json.dumps({key: row[key] for key in ("cycle_id", "safe_count",
            "predicted_collision_count", "safe_in_overlap_top_30",
            "safe_in_original_total_top_30", "safe_in_fixed_scale_counterfactual_top_30",
            "safe_probability_mass_original", "safe_probability_mass_fixed_scale_counterfactual",
            "overlap_safe_auc")}))
