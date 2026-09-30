#!/usr/bin/env python3
"""Decompose frozen nine-step Y error with physical source velocity oracle."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quantile(values, p):
    ordered = sorted(values)
    if not ordered:
        return None
    index = p * (len(ordered) - 1)
    lower = int(index)
    return ordered[lower] + (index - lower) * (ordered[min(lower + 1, len(ordered) - 1)]
                                              - ordered[lower])


def magnitude(values, unit="m"):
    values = [abs(value) for value in values]
    return {"n": len(values), f"median_abs_{unit}": statistics.median(values) if values else None,
            f"p90_abs_{unit}": quantile(values, 0.9),
            f"max_abs_{unit}": max(values) if values else None}


def summarize(rows):
    if not rows:
        return {"source_steps": 0}
    misses = [row for row in rows if not row["oracle_y_covers_physical_box"]]
    return {"source_steps": len(rows),
            "tracker_y_covered": sum(row["tracker_y_covers_physical_box"] for row in rows),
            "oracle_y_covered": len(rows) - len(misses),
            "oracle_excess_max_m": max(row["oracle_excess_m"] for row in rows),
            "source_component": magnitude(row["source_center_component_m"] for row in rows),
            "tracker_velocity_component": magnitude(row["tracker_velocity_component_m"] for row in rows),
            "oracle_motion_component": magnitude(row["oracle_motion_component_m"] for row in rows),
            "first_oracle_miss": ({key: misses[0][key] for key in
                ("side", "source_t", "step", "horizon_s", "oracle_excess_m")}
                if misses else None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_evidence", type=Path)
    parser.add_argument("nine_step_evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    evidence, nine, output = (path.resolve() for path in
                              (args.source_evidence, args.nine_step_evidence, args.output))
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip():
        raise ValueError("oracle evaluation plan is not committed")
    evaluation_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    manifest = json.loads((evidence / "manifest.json").read_text())
    nine_summary = json.loads((nine / "summary.json").read_text())
    if digest(nine / "source_steps.csv") != nine_summary["details_sha256"]:
        raise ValueError("frozen nine-step table changed")
    if nine_summary["source_run_commit"] != manifest["run_commit"]:
        raise ValueError("evidence from different run")
    sources = {}
    input_hashes = {"source_steps.csv": digest(nine / "source_steps.csv")}
    for side in ("south", "north"):
        path = evidence / f"{side}_sources.csv"
        input_hashes[path.name] = digest(path)
        if input_hashes[path.name] != manifest["packaged_sha256"][path.name]:
            raise ValueError(f"source table changed: {side}")
        with path.open() as stream:
            for item in csv.DictReader(stream):
                sources[(side, round(float(item["source_t"]), 6))] = item
    with (nine / "source_steps.csv").open() as stream:
        original = list(csv.DictReader(stream))
    rows = []
    for item in original:
        side = item["side"]
        stamp = float(item["source_t"])
        source = sources.get((side, round(stamp, 6)))
        if source is None:
            raise ValueError(f"missing physical source velocity: {side} {stamp}")
        horizon = float(item["horizon_s"])
        tracker_vy = float(source["tracker_vy_mps"])
        physical_vy = float(source["physical_vy_mps"])
        physical_source_y = float(source["physical_box_y_m"])
        tracker_forecast = float(item["corrected_forecast_center_y_m"])
        corrected_source_y = tracker_forecast - tracker_vy * horizon
        physical_future_y = float(item["true_box_center_y_m"])
        source_component = corrected_source_y - physical_source_y
        velocity_component = (tracker_vy - physical_vy) * horizon
        motion_component = physical_source_y + physical_vy * horizon - physical_future_y
        reconstructed = source_component + velocity_component + motion_component
        captured = float(item["corrected_center_error_m"])
        if abs(reconstructed - captured) > 1e-9:
            raise ValueError(f"error decomposition mismatch: {side} {stamp} {horizon}")
        allowed = float(item["candidate_allowed_center_error_m"])
        oracle_error = source_component + motion_component
        oracle_excess = max(0.0, abs(oracle_error) - allowed)
        turning = item["truth_turns_within_0_9_s"] == "True"
        rows.append({"side": side, "source_t": stamp, "step": int(item["step"]),
                     "horizon_s": horizon, "truth_turns_within_0_9_s": turning,
                     "tracker_vy_mps": tracker_vy, "physical_source_vy_mps": physical_vy,
                     "source_center_component_m": source_component,
                     "tracker_velocity_component_m": velocity_component,
                     "oracle_motion_component_m": motion_component,
                     "tracker_center_error_m": captured,
                     "oracle_center_error_m": oracle_error,
                     "allowed_center_error_m": allowed,
                     "tracker_y_covers_physical_box":
                         item["candidate_y_covers_physical_box"] == "True",
                     "oracle_y_covers_physical_box": oracle_excess <= 1e-9,
                     "oracle_excess_m": oracle_excess})
    output.mkdir(parents=True)
    with (output / "source_steps.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    result = {}
    for side in ("south", "north"):
        selected = [row for row in rows if row["side"] == side]
        first = [row for row in selected if row["step"] == 1]
        result[side] = {"unique_sources": len(first),
                        "source_velocity_error_mps": magnitude(
                            (row["tracker_vy_mps"] - row["physical_source_vy_mps"]
                             for row in first), unit="mps"),
                        "all": summarize(selected),
                        "turn_window": summarize([row for row in selected
                                                  if row["truth_turns_within_0_9_s"]]),
                        "no_turn_window": summarize([row for row in selected
                                                     if not row["truth_turns_within_0_9_s"]]),
                        "steps": {str(step): summarize([row for row in selected
                                                        if row["step"] == step])
                                  for step in range(1, 10)}}
    summary = {"schema": "rm_dynamic_prediction_oracle_velocity_probe/v1",
               "scope": "Offline physical source velocity oracle only. Already inspected same-box Y-only fixture; never a runtime input or MPPI safety claim.",
               "evaluation_commit": evaluation_commit,
               "source_run_commit": manifest["run_commit"],
               "input_sha256": input_hashes, "results": result,
               "oracle_still_misses": any(result[side]["all"]["oracle_y_covered"] <
                                          result[side]["all"]["source_steps"]
                                          for side in ("south", "north")),
               "details_sha256": digest(output / "source_steps.csv")}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"oracle_still_misses": summary["oracle_still_misses"],
                      "results": {side: {"velocity_error_mps": result[side]["source_velocity_error_mps"],
                                         "all": {key: result[side]["all"][key] for key in
                                                 ("source_steps", "tracker_y_covered", "oracle_y_covered",
                                                  "oracle_excess_max_m")}}
                                  for side in ("south", "north")}}, indent=2))


if __name__ == "__main__":
    main()
