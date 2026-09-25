#!/usr/bin/env python3
"""Test a frozen-trial directional box proposal on older prediction recordings.

Recorder messages are not the exact messages consumed by the MPPI critic.
The proposal is fitted only on the named training trial's Gazebo truth. It is
never written into runtime configuration or used as a safety bound.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import yaml

from analyze import interpolated_pose, rows_from_transport
from envelope import predicted_box


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def cycle_metadata_digest(trial):
    h = hashlib.sha256()
    paths = sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                   key=lambda p: int(p.stem.split("_")[1]))
    for path in paths:
        h.update(path.name.encode())
        h.update(b"\0")
        h.update(path.read_bytes())
    return len(paths), h.hexdigest()


def truth_at(rows, times, t):
    if t <= times[0] or t >= times[-1]:
        return None
    return interpolated_pose(rows, times, t)


def side(robot_x, track_x):
    dx = robot_x - track_x
    return "east" if dx > .6 else "west" if dx < -.6 else "middle"


def accepted_samples(message, rows, times, step_count):
    confirmed = [track for track in message["tracks"] if track["state"] == 2]
    if len(confirmed) != 1 or not message.get("complete"):
        return None
    track = confirmed[0]
    source_t = float(message["source_t"])
    if not times[0] < source_t < times[-1]:
        return None
    robot_x = robot_at(rows, times, source_t)[0]
    group = side(robot_x, track["xy"][0])
    samples = []
    for j in range(step_count + 1):
        dt = j * float(message["prediction_dt"])
        t = source_t + dt
        actual = truth_at(rows, times, t)
        if actual is None:
            break
        if abs(actual[2]) > 1e-4:
            raise ValueError("oriented physical box requires a separate audit")
        nominal = [track["xy"][i] + track["vxy"][i] * dt for i in (0, 1)]
        samples.append({"step": j, "source_t": source_t, "t": t,
                        "actual": actual, "nominal": nominal,
                        "residual": [actual[i] - nominal[i] for i in (0, 1)]})
    if not samples:
        return None
    return group, track, samples


def robot_at(rows, times, t):
    # The obstacle interpolation helper is intentionally specific to obstacle.
    import bisect
    i = bisect.bisect_right(times, t)
    a, b = rows[i - 1], rows[i]
    fraction = (t - a["t"]) / (b["t"] - a["t"])
    return tuple(a["robot"][axis] + fraction * (b["robot"][axis] - a["robot"][axis])
                 for axis in (0, 1))


def training_messages(trial):
    for path in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda p: int(p.stem.split("_")[1])):
        meta = json.loads(path.read_text())
        entries = [event["value"] for event in meta["events"]
                   if event["kind"] == "prediction.input"]
        if len(entries) != 1 or entries[0].get("status") != "accepted":
            continue
        value = entries[0]
        yield {"source_t": value["source_stamp_ns"] / 1e9,
               "source_age_s": value["source_age_s"],
               "prediction_dt": value["prediction_dt"],
               "complete": value["complete"], "tracks": value["tracks"]}


def recorded_messages(trial):
    with (trial / "predictions.jsonl").open() as stream:
        for line in stream:
            yield json.loads(line)


def summarize(trial, messages, step_count, proposal=None):
    rows = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in rows]
    result = {"trial": str(trial), "recorded_messages": 0,
              "accepted_messages": 0, "by_side": {}, "source_sha256": {
                  "gazebo_poses": digest(trial / "gazebo_poses.jsonl")}}
    ages = []
    if (trial / "predictions.jsonl").exists():
        result["source_sha256"]["predictions"] = digest(trial / "predictions.jsonl")
    if (trial / "profile.yaml").exists():
        result["source_sha256"]["profile"] = digest(trial / "profile.yaml")
    groups = defaultdict(list)
    for message in messages:
        result["recorded_messages"] += 1
        if "source_age_s" in message and message["source_age_s"] is not None:
            ages.append(float(message["source_age_s"]))
        accepted = accepted_samples(message, rows, times, step_count)
        if accepted is None:
            continue
        group, track, samples = accepted
        result["accepted_messages"] += 1
        groups[group].append((track, samples))
    if ages:
        result["maximum_source_age_s"] = max(ages)
    extent = (.45, .55)
    for group, items in sorted(groups.items()):
        residuals = [sample["residual"] for _, samples in items for sample in samples]
        source = [samples[0]["residual"] for _, samples in items]
        row = {"messages": len(items), "samples": len(residuals),
               "source_residual_x_min_m": min(v[0] for v in source),
               "source_residual_x_max_m": max(v[0] for v in source),
               "future_residual_x_min_m": min(v[0] for v in residuals),
               "future_residual_x_max_m": max(v[0] for v in residuals),
               "positive_x_samples": sum(v[0] > 0 for v in residuals),
               "v1_uncovered_samples": 0, "v1_max_uncovered_edge_m": 0.,
               "proposal_uncovered_samples": 0,
               "proposal_max_uncovered_edge_m": 0., "first_proposal_miss": None}
        for track, samples in items:
            for sample in samples:
                actual = sample["actual"]
                dt = sample["t"] - sample["source_t"]
                box = predicted_box(track["xy"], track["vxy"],
                                    track["size_xy"], extent, 0., dt,
                                    .5551652475612764)
                miss = edge_deficit(box, actual, extent)
                if miss > 1e-9:
                    row["v1_uncovered_samples"] += 1
                    row["v1_max_uncovered_edge_m"] = max(
                        row["v1_max_uncovered_edge_m"], miss)
                if proposal is not None and group == "east":
                    lower, upper = proposal
                    nominal = sample["nominal"]
                    from envelope import AxisBox
                    candidate = AxisBox(nominal[0] + lower[0] - extent[0] / 2,
                                        nominal[1] + lower[1] - extent[1] / 2,
                                        nominal[0] + upper[0] + extent[0] / 2,
                                        nominal[1] + upper[1] + extent[1] / 2)
                    miss = edge_deficit(candidate, actual, extent)
                    if miss > 1e-9:
                        row["proposal_uncovered_samples"] += 1
                        row["proposal_max_uncovered_edge_m"] = max(
                            row["proposal_max_uncovered_edge_m"], miss)
                        if row["first_proposal_miss"] is None:
                            row["first_proposal_miss"] = {
                                "source_t": sample["source_t"],
                                "step": sample["step"], "residual": sample["residual"],
                                "uncovered_edge_m": miss}
        result["by_side"][group] = row
    return result, groups


def edge_deficit(box, actual, extent):
    return max(0., box.min_x - (actual[0] - extent[0] / 2),
               (actual[0] + extent[0] / 2) - box.max_x,
               box.min_y - (actual[1] - extent[1] / 2),
               (actual[1] + extent[1] / 2) - box.max_y)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--training-trial", type=Path, required=True)
    parser.add_argument("--heldout-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # Source age was below 0.1 s in the training run; its 9 scored future
    # steps can therefore reach nearly 1.0 s after the message source stamp.
    steps = 10
    training, grouped = summarize(args.training_trial,
                                  training_messages(args.training_trial), steps)
    count, metadata_sha = cycle_metadata_digest(args.training_trial)
    training["cycle_metadata_count"] = count
    training["source_sha256"]["cycle_metadata_stream"] = metadata_sha
    if training["maximum_source_age_s"] > .1:
        raise ValueError("source-time audit does not span the actual critic horizon")
    east = [sample["residual"] for _, samples in grouped["east"]
            for sample in samples]
    if len(east) < 100:
        raise ValueError("insufficient east-side training observations")
    margin = .05
    lower = [min(value[axis] for value in east) - margin for axis in (0, 1)]
    upper = [max(value[axis] for value in east) + margin for axis in (0, 1)]
    heldout = []
    for prediction_path in sorted(args.heldout_root.rglob("predictions.jsonl")):
        trial = prediction_path.parent
        if not prediction_path.stat().st_size or not (trial / "profile.yaml").exists():
            continue
        profile = yaml.safe_load((trial / "profile.yaml").read_text())
        follow = profile["controller_server"]["ros__parameters"]["FollowPath"]
        config = follow.get("PredictionV1Critic")
        if not config:
            continue
        if abs(float(follow["model_dt"]) - .1) > 1e-9 or int(follow["time_steps"]) != 30:
            raise ValueError(f"different historical MPPI horizon: {trial}")
        expected = (.45, .55, .5551652475612764, 1.)
        actual = tuple(float(config[key]) for key in
                       ("object_width", "object_height", "reference_acceleration", "horizon"))
        if any(abs(a - b) > 1e-9 for a, b in zip(actual, expected)):
            raise ValueError(f"different historical prediction config: {trial}")
        row, _ = summarize(trial, recorded_messages(trial), steps,
                           (lower, upper))
        heldout.append(row)
    output = {"schema": "rm_directional_support_heldout_audit/v1",
              "scope": "Source-stamp anchored recorder messages; historical recordings are not exact MPPI-consumed messages. All future truth is used only for validation.",
              "training": training, "proposal_east_residual_lower_m": lower,
              "proposal_east_residual_upper_m": upper,
              "proposal_extra_margin_m": margin, "future_steps": steps,
              "heldout": heldout}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"training_east_messages": training["by_side"]["east"]["messages"],
                      "proposal": [lower, upper],
                      "heldout_east": [{"trial": row["trial"],
                                        **row["by_side"].get("east", {})}
                                       for row in heldout]}, indent=2))


if __name__ == "__main__":
    main()
