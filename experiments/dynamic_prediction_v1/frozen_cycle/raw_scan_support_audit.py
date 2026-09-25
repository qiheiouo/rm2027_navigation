#!/usr/bin/env python3
"""Reconstruct source-scan clusters and audit geometry on the known fixture.

Uses the existing static map and candidate/cluster filters. Gazebo robot pose
or the recorded simulation odometry transforms raw scan rays; box truth is
used only to score selected clusters.
No tracker/KF update is implemented or changed.
"""
import argparse
import bisect
from collections import defaultdict
import json
import math
from pathlib import Path
import sys

from analyze import interpolated_pose, rows_from_transport
from heldout_geometry_audit import (digest, recorded_messages, side,
                                    training_messages)

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_tracker_probe_20260924"))
from probe import interpolator, scan_points, static_map
from rm_dynamic_obstacle_tracking.core import (cluster_points,
                                               dynamic_candidates,
                                               filter_detections_near_static)


EXTENT = (.45, .55)
RANGE_ERROR_ALLOWANCE = .03
REFERENCE_ACCELERATION = .5551652475612764


def components(points, tolerance):
    """Offline membership recovery for core.cluster_points' detections."""
    unused = set(range(len(points)))
    groups = []
    threshold = tolerance * tolerance
    while unused:
        seed = unused.pop()
        group = [seed]
        queue = [seed]
        while queue:
            current = queue.pop()
            joined = [index for index in unused if
                      (points[current].x - points[index].x) ** 2 +
                      (points[current].y - points[index].y) ** 2 <= threshold]
            unused.difference_update(joined)
            queue.extend(joined)
            group.extend(joined)
        groups.append([points[index] for index in group])
    return groups


def percentile(values, quantile):
    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[round((len(ordered) - 1) * quantile)]


def coordinates_error(estimated, truth):
    return (estimated[0] - truth[0], estimated[1] - truth[1])


def near_full_span(row):
    return all(row["raw_span"][axis] >= EXTENT[axis] - .05
               for axis in (0, 1))


def fitted_velocity(history):
    times = [row["source_t"] for row in history]
    mean_t = sum(times) / len(times)
    denominator = sum((t - mean_t) ** 2 for t in times)
    if denominator <= 1e-9:
        return None
    return tuple(sum((t - mean_t) * row["raw_bbox_mid"][axis]
                     for t, row in zip(times, history)) / denominator
                 for axis in (0, 1))


def trajectory_pose(rows, times, t):
    index = bisect.bisect_right(times, t)
    if index == 0 or index == len(times):
        return None
    a, b = rows[index - 1], rows[index]
    fraction = (t - a["t"]) / (b["t"] - a["t"])
    yaw_delta = math.remainder(b["yaw"] - a["yaw"], 2 * math.pi)
    return (a["x"] + fraction * (b["x"] - a["x"]),
            a["y"] + fraction * (b["y"] - a["y"]),
            a["yaw"] + fraction * yaw_delta)


def one_trial(trial, messages, occupancy, return_source_rows=False,
              pose_source="gazebo"):
    if pose_source not in ("gazebo", "recorded_odom"):
        raise ValueError("unknown scan pose source")
    rows = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in rows]
    at = interpolator(rows)
    trajectory = ([json.loads(line) for line in
                   (trial / "observation/trajectory.jsonl").open()]
                  if pose_source == "recorded_odom" else None)
    trajectory_times = ([row["t"] for row in trajectory]
                        if trajectory is not None else None)
    scans = {round(scan["t"], 6): scan for line in
             (trial / "observation/scans.jsonl").open()
             if (scan := json.loads(line))}
    groups = defaultdict(list)
    all_samples = []
    seen = set()
    counts = defaultdict(int)
    examples = {}
    pose_translation_differences = []
    pose_yaw_differences = []
    for message in messages:
        confirmed = [track for track in message["tracks"] if track["state"] == 2]
        if len(confirmed) != 1 or not message.get("complete"):
            continue
        track = confirmed[0]
        t = float(message["source_t"])
        key = (round(t, 6), track["id"])
        if key in seen:
            continue
        seen.add(key)
        counts["unique_confirmed_sources"] += 1
        if not times[0] < t < times[-1]:
            counts["truth_time_missing"] += 1
            continue
        scan = scans.get(round(t, 6))
        if scan is None:
            counts["scan_missing"] += 1
            continue
        actual = interpolated_pose(rows, times, t)
        if abs(actual[2]) > 1e-4:
            raise ValueError("rotated physical box requires oriented support")
        if scan["frame"] != "sim_lidar_link":
            raise ValueError("scan frame requires an explicit calibrated transform")
        pose = at(t)
        robot = (trajectory_pose(trajectory, trajectory_times, t)
                 if trajectory is not None else pose["robot"])
        if robot is None:
            counts["recorded_odom_time_missing"] += 1
            continue
        pose_translation_differences.append(math.hypot(
            robot[0] - pose["robot"][0],
            robot[1] - pose["robot"][1]))
        pose_yaw_differences.append(abs(math.remainder(
            robot[2] - pose["robot"][2], 2 * math.pi)))
        endpoints = scan_points(scan, robot)
        candidates = dynamic_candidates(endpoints, occupancy, .25, True)
        detections = filter_detections_near_static(
            cluster_points(candidates, .20, 3, 1.5), occupancy, .35)
        if not detections:
            counts["no_reconstructed_detection"] += 1
            continue
        detection = min(detections, key=lambda item:
                        (item.centroid.x - track["xy"][0]) ** 2 +
                        (item.centroid.y - track["xy"][1]) ** 2)
        if math.hypot(detection.centroid.x - track["xy"][0],
                      detection.centroid.y - track["xy"][1]) > .6:
            counts["no_track_matched_detection"] += 1
            continue
        matches = [component for component in components(candidates, .20)
                   if len(component) == detection.point_count and
                   abs(sum(point.x for point in component) / len(component) -
                       detection.centroid.x) < 1e-6 and
                   abs(sum(point.y for point in component) / len(component) -
                       detection.centroid.y) < 1e-6]
        if len(matches) != 1:
            raise ValueError("could not uniquely recover existing core cluster")
        observed = [(point.x, point.y) for point in matches[0]]
        group = side(robot[0], track["xy"][0])
        raw_mean = tuple(sum(point[axis] for point in observed) / len(observed)
                         for axis in (0, 1))
        if (abs(raw_mean[0] - detection.centroid.x) > 1e-6 or
                abs(raw_mean[1] - detection.centroid.y) > 1e-6):
            raise ValueError("recovered component disagrees with existing cluster")
        low = [min(point[axis] for point in observed) for axis in (0, 1)]
        high = [max(point[axis] for point in observed) for axis in (0, 1)]
        bbox_mid = tuple((low[axis] + high[axis]) / 2 for axis in (0, 1))
        support_lower = [high[axis] - RANGE_ERROR_ALLOWANCE - EXTENT[axis] / 2
                         for axis in (0, 1)]
        support_upper = [low[axis] + RANGE_ERROR_ALLOWANCE + EXTENT[axis] / 2
                         for axis in (0, 1)]
        deficit = max(0., *(support_lower[axis] - actual[axis]
                            for axis in (0, 1)),
                      *(actual[axis] - support_upper[axis]
                        for axis in (0, 1)))
        covered = deficit <= 1e-9
        future_samples = 0
        future_misses = 0
        future_max_deficit = 0.
        for step in range(11):
            future_t = t + step * .1
            if future_t >= times[-1]:
                break
            future_actual = interpolated_pose(rows, times, future_t)
            mismatch = .5 * REFERENCE_ACCELERATION * (step * .1) ** 2
            future_deficit = max(0., *(
                support_lower[axis] + track["vxy"][axis] * (step * .1) -
                mismatch - future_actual[axis] for axis in (0, 1)), *(
                future_actual[axis] - support_upper[axis] -
                track["vxy"][axis] * (step * .1) - mismatch
                for axis in (0, 1)))
            future_samples += 1
            future_misses += future_deficit > 1e-9
            future_max_deficit = max(future_max_deficit, future_deficit)
        row = {"source_t": t, "side": group, "point_count": len(observed),
               "detection_to_track_m": math.hypot(
                   detection.centroid.x - track["xy"][0],
                   detection.centroid.y - track["xy"][1]),
               "track_error": coordinates_error(track["xy"], actual),
               "raw_mean_error": coordinates_error(raw_mean, actual),
               "raw_bbox_mid_error": coordinates_error(bbox_mid, actual),
               "raw_span": [high[axis] - low[axis] for axis in (0, 1)],
               "support_width": [support_upper[axis] - support_lower[axis]
                                 for axis in (0, 1)],
               "support_contains_truth": covered,
               "support_uncovered_edge_m": deficit,
               "future_source_anchored_samples": future_samples,
               "future_source_anchored_misses": future_misses,
               "future_source_anchored_max_uncovered_edge_m": future_max_deficit,
               "actual_center": actual[:2], "track_xy": track["xy"],
               "track_vxy": track["vxy"],
               "raw_mean": raw_mean, "raw_bbox_mid": bbox_mid,
               "raw_min_xy": low, "raw_max_xy": high,
               "support_lower": support_lower, "support_upper": support_upper}
        groups[group].append(row)
        all_samples.append(row)
        if t in (46.201, 49.633):
            examples[str(t)] = row
    output = {"trial": str(trial), "pose_source": pose_source,
              "counts": dict(counts),
              "pose_source_vs_gazebo": {
                  "sample_count": len(pose_translation_differences),
                  "max_translation_m": max(pose_translation_differences,
                                           default=None),
                  "max_yaw_rad": max(pose_yaw_differences, default=None)},
              "source_sha256": {"gazebo_poses": digest(trial / "gazebo_poses.jsonl"),
                                "scans": digest(trial / "observation/scans.jsonl")},
              "by_side": {}, "selected_examples": examples}
    if (trial / "predictions.jsonl").exists():
        output["source_sha256"]["predictions"] = digest(trial / "predictions.jsonl")
    if (trial / "profile.yaml").exists():
        output["source_sha256"]["profile"] = digest(trial / "profile.yaml")
    if trajectory is not None:
        output["source_sha256"]["recorded_odom_trajectory"] = digest(
            trial / "observation/trajectory.jsonl")
    recent = []
    for row in sorted(all_samples, key=lambda item: item["source_t"]):
        if not near_full_span(row):
            continue
        recent = [old for old in recent if
                  row["source_t"] - old["source_t"] <= .4]
        recent.append(row)
        history = recent[-5:]
        if len(history) < 3:
            continue
        velocity = fitted_velocity(history)
        if velocity is None:
            continue
        row["scan_center_velocity"] = velocity
        horizon = 1.
        future_t = row["source_t"] + horizon
        if future_t >= times[-1]:
            continue
        future = interpolated_pose(rows, times, future_t)
        predictions = {
            "track_cv": tuple(row["track_xy"][axis] +
                              row["track_vxy"][axis] * horizon
                              for axis in (0, 1)),
            "scan_center_track_velocity": tuple(
                row["raw_bbox_mid"][axis] +
                row["track_vxy"][axis] * horizon for axis in (0, 1)),
            "scan_center_scan_velocity": tuple(
                row["raw_bbox_mid"][axis] + velocity[axis] * horizon
                for axis in (0, 1)),
        }
        row["one_second_prediction_errors_m"] = {
            label: math.hypot(predicted[0] - future[0], predicted[1] - future[1])
            for label, predicted in predictions.items()}
    for group, samples in sorted(groups.items()):
        stats = {"source_count": len(samples),
                 "support_coverage_failures": sum(
                     not row["support_contains_truth"] for row in samples),
                 "support_max_uncovered_edge_m": max(
                     row["support_uncovered_edge_m"] for row in samples),
                 "support_empty_intervals": sum(
                     min(row["support_width"]) < 0 for row in samples),
                 "detection_to_track_m_p95": percentile(
                     [row["detection_to_track_m"] for row in samples], .95),
                 "median_visible_points": percentile(
                     [row["point_count"] for row in samples], .5)}
        for label in ("track", "raw_mean", "raw_bbox_mid"):
            errors = [row[f"{label}_error"] for row in samples]
            norms = [math.hypot(*error) for error in errors]
            stats[f"{label}_error_m"] = {
                "median": percentile(norms, .5),
                "p95": percentile(norms, .95), "max": max(norms),
                "x_min": min(error[0] for error in errors),
                "x_max": max(error[0] for error in errors)}
        stats["raw_support_width_median_xy"] = [
            percentile([row["support_width"][axis] for row in samples], .5)
            for axis in (0, 1)]
        stats["raw_span_median_xy"] = [
            percentile([row["raw_span"][axis] for row in samples], .5)
            for axis in (0, 1)]
        full_span = [row for row in samples if near_full_span(row)]
        velocity_rows = [row for row in full_span if
                         "one_second_prediction_errors_m" in row]
        stats["near_full_span"] = {
            "count": len(full_span),
            "track_error_median": percentile([
                math.hypot(*row["track_error"]) for row in full_span], .5),
            "track_error_p95": percentile([
                math.hypot(*row["track_error"]) for row in full_span], .95),
            "velocity_estimation_sources": len(velocity_rows),
            "one_second_prediction_error_m": {
                label: {
                    "median": percentile([
                        row["one_second_prediction_errors_m"][label]
                        for row in velocity_rows], .5),
                    "p95": percentile([
                        row["one_second_prediction_errors_m"][label]
                        for row in velocity_rows], .95),
                    "max": max((
                        row["one_second_prediction_errors_m"][label]
                        for row in velocity_rows), default=None),
                } for label in ("track_cv", "scan_center_track_velocity",
                                "scan_center_scan_velocity")},
            "future_source_anchored_samples": sum(
                row["future_source_anchored_samples"] for row in full_span),
            "future_source_anchored_misses": sum(
                row["future_source_anchored_misses"] for row in full_span),
            "future_source_anchored_max_uncovered_edge_m": max((
                row["future_source_anchored_max_uncovered_edge_m"]
                for row in full_span), default=None),
            "raw_bbox_mid_error_median": percentile([
                math.hypot(*row["raw_bbox_mid_error"]) for row in full_span], .5),
            "raw_bbox_mid_error_p95": percentile([
                math.hypot(*row["raw_bbox_mid_error"]) for row in full_span], .95),
            "raw_bbox_mid_error_max": max((
                math.hypot(*row["raw_bbox_mid_error"]) for row in full_span),
                default=None),
            "support_coverage_failures": sum(
                not row["support_contains_truth"] for row in full_span),
        }
        if stats["support_coverage_failures"]:
            stats["worst_support_miss"] = max(
                samples, key=lambda sample: sample["support_uncovered_edge_m"])
        output["by_side"][group] = stats
    return (output, all_samples) if return_source_rows else output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--training-trial", type=Path, required=True)
    parser.add_argument("--heldout-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    occupancy, _ = static_map()
    training = one_trial(args.training_trial,
                         training_messages(args.training_trial), occupancy)
    heldout = []
    for path in sorted(args.heldout_root.rglob("predictions.jsonl")):
        trial = path.parent
        if not path.stat().st_size or not (trial / "profile.yaml").exists():
            continue
        if not (trial / "observation/scans.jsonl").exists():
            continue
        import yaml
        profile = yaml.safe_load((trial / "profile.yaml").read_text())
        if "PredictionV1Critic" not in profile["controller_server"][
                "ros__parameters"]["FollowPath"]:
            continue
        heldout.append(one_trial(trial, recorded_messages(trial), occupancy))
    output = {"schema": "rm_raw_scan_support_oracle_diagnostic/v1",
              "scope": "Existing static-map subtraction and clustering reconstructed from source scans; nearest detection to confirmed track associated without box truth. Gazebo robot pose transforms rays and box truth scores geometry. Diagnostic only; no certified range-noise bound.",
              "physical_extent_xy_m": EXTENT,
              "hypothetical_range_error_allowance_m": RANGE_ERROR_ALLOWANCE,
              "cv_mismatch_reference_acceleration_mps2": REFERENCE_ACCELERATION,
              "training": training, "heldout": heldout}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    for result in [training, *heldout]:
        print(result["trial"], {side: (stats["source_count"],
                                     stats["support_coverage_failures"],
                                     round(stats["track_error_m"]["median"], 3),
                                     round(stats["raw_bbox_mid_error_m"]["median"], 3))
                                for side, stats in result["by_side"].items()})


if __name__ == "__main__":
    main()
