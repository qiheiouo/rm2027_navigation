#!/usr/bin/env python3
"""Describe visible-surface bias in actually consumed phase-2 tracker sources."""
import argparse
import bisect
import csv
import json
from pathlib import Path
import statistics

import analyze


def interpolate_y(rows, times, stamp, key):
    index = bisect.bisect_right(times, stamp)
    if not 0 < index < len(rows):
        raise ValueError("source stamp outside physical truth")
    before, after = rows[index - 1], rows[index]
    ratio = (stamp - before["t"]) / (after["t"] - before["t"])
    return before[key][1] + ratio * (after[key][1] - before[key][1])


def summarize(rows):
    if not rows:
        return {"unique_sources": 0}
    bias = [row["visible_center_minus_physical_center_y_m"] for row in rows]
    return {"unique_sources": len(rows), "bias_median_m": statistics.median(bias),
            "bias_min_m": min(bias), "bias_max_m": max(bias)}


def audit(trial, output):
    if output.exists():
        raise FileExistsError(output)
    selection = json.loads((trial / "selection.json").read_text())
    witness = selection["witness_sim_s"]
    truth_path = trial / "gazebo_poses.jsonl"
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    seen = {}
    accepted_cycles = 0
    for path in sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                       key=lambda item: int(item.stem.removeprefix("cycle_"))):
        meta = json.loads(path.read_text())
        events = [event["value"] for event in meta["events"]
                  if event["kind"] == "prediction.input"]
        if len(events) != 1 or events[0].get("status") != "accepted":
            continue
        event = events[0]
        tracks = [track for track in event["tracks"] if track["state"] == 2]
        if len(tracks) != 1:
            continue
        accepted_cycles += 1
        stamp = event["source_stamp_ns"] / 1e9
        track = tracks[0]
        signature = (track["xy"], track["vxy"], track["size_xy"])
        if stamp in seen:
            if signature != seen[stamp]["signature"]:
                raise ValueError(f"same source stamp has different track state: {stamp}")
            seen[stamp]["consumer_cycle_count"] += 1
            continue
        robot_y = interpolate_y(truth, times, stamp, "robot")
        physical_y = interpolate_y(truth, times, stamp, "obstacle")
        seen[stamp] = {"source_stamp_s": stamp, "first_consumer_sim_s":
                       event["consumer_sim_s"], "first_consumer_cycle_id": meta["cycle_id"],
                       "first_consumer_cycle_sha256": analyze.sha(path),
                       "consumer_cycle_count": 1,
                       "visible_center_y_m": track["xy"][1],
                       "visible_span_y_m": track["size_xy"][1],
                       "source_velocity_y_mps": track["vxy"][1],
                       "physical_center_y_m": physical_y, "robot_y_m": robot_y,
                       "visible_center_minus_physical_center_y_m":
                           track["xy"][1] - physical_y,
                       "online_view_side": ("north" if robot_y > track["xy"][1]
                                            else "south"),
                       "truth_view_side": ("north" if robot_y > physical_y
                                           else "south"),
                       "signature": signature}
    rows = sorted(seen.values(), key=lambda row: row["source_stamp_s"])
    for row in rows:
        del row["signature"]
    small = lambda row: row["visible_span_y_m"] < .275
    near = lambda row: witness - 2 <= row["first_consumer_sim_s"] <= witness
    groups = {
        "all": summarize(rows),
        "north_visible_span_below_half_height": summarize([
            row for row in rows if row["truth_view_side"] == "north" and small(row)]),
        "north_visible_span_at_least_half_height": summarize([
            row for row in rows if row["truth_view_side"] == "north" and not small(row)]),
        "south_visible_span_below_half_height": summarize([
            row for row in rows if row["truth_view_side"] == "south" and small(row)]),
        "south_visible_span_at_least_half_height": summarize([
            row for row in rows if row["truth_view_side"] == "south" and not small(row)]),
        "two_seconds_before_breach_north_narrow": summarize([
            row for row in rows if near(row) and row["truth_view_side"] == "north"
            and small(row)]),
    }
    selected_meta = json.loads(Path(selection["selected_cycle_json"]).read_text())
    selected_events = [event["value"] for event in selected_meta["events"]
                       if event["kind"] == "prediction.input"]
    if len(selected_events) != 1:
        raise ValueError("selected cycle has no unique consumed source")
    selected_stamp = selected_events[0]["source_stamp_ns"] / 1e9
    selected = next(row for row in rows if row["source_stamp_s"] == selected_stamp)
    output.mkdir(parents=True)
    details = output / "consumed_sources.csv"
    with details.open("x", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    report = {"schema": "rm_dynamic_prediction_phase2_source_surface_audit/v1",
              "scope": "Descriptive post hoc physical labels on unique actually consumed tracker source stamps. Repeated source frames and adjacent frames are correlated; no model parameter is fit here.",
              "accepted_cycles_with_one_confirmed_track": accepted_cycles,
              "unique_consumed_sources": len(rows),
              "online_truth_view_side_disagreements": sum(
                  row["online_view_side"] != row["truth_view_side"] for row in rows),
              "known_box_height_m": .55, "half_height_split_m": .275,
              "first_dynamic_breach_sim_s": witness,
              "group_descriptions": groups,
              "selected_cycle_source": selected,
              "inputs_sha256": {"selection": analyze.sha(trial / "selection.json"),
                                "physical_truth": analyze.sha(truth_path)},
              "details_sha256": analyze.sha(details)}
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                               sort_keys=True) + "\n")
    print(json.dumps({"unique_sources": len(rows),
                      "selected_bias_m": selected[
                          "visible_center_minus_physical_center_y_m"],
                      "groups": groups}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    audit(args.trial, args.output)
