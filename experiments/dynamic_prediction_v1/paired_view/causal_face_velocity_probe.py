#!/usr/bin/env python3
"""Test a four-source, past-only near-face velocity on frozen nine-step data."""
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


def p90(values):
    ordered = sorted(values)
    if not ordered:
        return None
    position = .9 * (len(ordered) - 1)
    index = int(position)
    return ordered[index] + (position - index) * (ordered[min(index + 1, len(ordered) - 1)] - ordered[index])


def speed_errors(values):
    absolute = [abs(value) for value in values]
    return {"n": len(absolute),
            "median_abs_mps": statistics.median(absolute) if absolute else None,
            "p90_abs_mps": p90(absolute),
            "max_abs_mps": max(absolute) if absolute else None}


def slope(four):
    ts = [t for t, _ in four]
    ys = [y for _, y in four]
    mean_t, mean_y = statistics.mean(ts), statistics.mean(ys)
    denominator = sum((t - mean_t) ** 2 for t in ts)
    if denominator <= 0:
        raise ValueError("non-increasing source time")
    return sum((t - mean_t) * (y - mean_y) for t, y in four) / denominator


def summarize(rows):
    if not rows:
        return {"source_steps": 0}
    valid = [row for row in rows if row["causal_vy_mps"] is not None]
    misses = [row for row in valid if not row["causal_y_covers_physical_box"]]
    return {"source_steps": len(rows), "causal_velocity_available_steps": len(valid),
            "causal_y_covered_all": sum(row["causal_y_covers_physical_box"] for row in rows),
            "causal_y_covered_valid": len(valid) - len(misses),
            "tracker_y_covered_valid": sum(row["tracker_y_covers_physical_box"] for row in valid),
            "oracle_y_covered_valid": sum(row["oracle_y_covers_physical_box"] for row in valid),
            "causal_excess_max_m": max((row["causal_excess_m"] for row in valid), default=None),
            "first_causal_miss": ({key: misses[0][key] for key in
                ("side", "source_t", "step", "horizon_s", "causal_excess_m")}
                if misses else None)}


def verified_rows(path, expected):
    if digest(path) != expected:
        raise ValueError(f"frozen table changed: {path}")
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source_evidence", "face_evidence", "nine_step_evidence",
                 "oracle_evidence", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    evidence, face, nine, oracle, output = (
        getattr(args, name).resolve() for name in
        ("source_evidence", "face_evidence", "nine_step_evidence",
         "oracle_evidence", "output"))
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip():
        raise ValueError("causal rule not committed")
    evaluation_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    manifest = json.loads((evidence / "manifest.json").read_text())
    face_summary = json.loads((face / "summary.json").read_text())
    nine_summary = json.loads((nine / "summary.json").read_text())
    oracle_summary = json.loads((oracle / "summary.json").read_text())
    face_path = face / "sources.csv"
    nine_path = nine / "source_steps.csv"
    oracle_path = oracle / "source_steps.csv"
    face_rows = verified_rows(face_path, face_summary["details_sha256"])
    nine_rows = verified_rows(nine_path, nine_summary["details_sha256"])
    oracle_rows = verified_rows(oracle_path, oracle_summary["details_sha256"])
    if len(nine_rows) != len(oracle_rows) or not (
        manifest["run_commit"] == nine_summary["source_run_commit"] ==
        oracle_summary["source_run_commit"]):
        raise ValueError("frozen source runs differ")
    hashes = {"face_sources": digest(face_path), "nine_steps": digest(nine_path),
              "oracle_steps": digest(oracle_path)}
    face_by_side = {}
    for row in face_rows:
        face_by_side.setdefault(row["side"], []).append(
            (float(row["source_t"]), float(row["near_face_center_y_m"])))
    centers, velocities = {}, {}
    for side, observations in face_by_side.items():
        observations.sort()
        for index, (stamp, center) in enumerate(observations):
            key = (side, round(stamp, 6))
            centers[key] = center
            if index < 3:
                velocities[key] = None
                continue
            recent = observations[index - 3:index + 1]
            span = recent[-1][0] - recent[0][0]
            velocities[key] = slope(recent) if 0.15 <= span <= 0.30 else None
    physical_velocity = {}
    for side in ("south", "north"):
        path = evidence / f"{side}_sources.csv"
        rows = verified_rows(path, manifest["packaged_sha256"][path.name])
        hashes[path.name] = digest(path)
        for row in rows:
            physical_velocity[(side, round(float(row["source_t"]), 6))] = float(
                row["physical_vy_mps"])
    output_rows = []
    for row, reference in zip(nine_rows, oracle_rows):
        key = (row["side"], round(float(row["source_t"]), 6))
        if key != (reference["side"], round(float(reference["source_t"]), 6)) or \
                row["step"] != reference["step"]:
            raise ValueError("nine-step and oracle rows do not align")
        if key not in centers or key not in physical_velocity:
            raise ValueError(f"missing source history: {key}")
        vy = velocities[key]
        horizon = float(row["horizon_s"])
        allowed = float(row["candidate_allowed_center_error_m"])
        if allowed < .05:
            raise ValueError("prediction budget changed")
        forecast = centers[key] + vy * horizon if vy is not None else None
        residual = forecast - float(row["true_box_center_y_m"]) if forecast is not None else None
        excess = max(0.0, abs(residual) - allowed) if residual is not None else None
        output_rows.append({"side": key[0], "source_t": float(row["source_t"]),
                            "step": int(row["step"]), "horizon_s": horizon,
                            "truth_turns_within_0_9_s": row["truth_turns_within_0_9_s"] == "True",
                            "causal_vy_mps": vy,
                            "physical_source_vy_mps": physical_velocity[key],
                            "causal_velocity_error_mps": vy - physical_velocity[key] if vy is not None else None,
                            "causal_center_error_m": residual,
                            "causal_excess_m": excess,
                            "causal_y_covers_physical_box": excess is not None and excess <= 1e-9,
                            "tracker_y_covers_physical_box": row["candidate_y_covers_physical_box"] == "True",
                            "oracle_y_covers_physical_box": reference["oracle_y_covers_physical_box"] == "True"})
    output.mkdir(parents=True)
    with (output / "source_steps.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)
    results = {}
    for side in ("south", "north"):
        selected = [row for row in output_rows if row["side"] == side]
        first = [row for row in selected if row["step"] == 1]
        valid = [row for row in first if row["causal_vy_mps"] is not None]
        results[side] = {"total_sources": len(first), "valid_sources": len(valid),
                         "missing_sources": len(first) - len(valid),
                         "velocity_error_mps": speed_errors(row["causal_velocity_error_mps"]
                                                              for row in valid),
                         "all": summarize(selected),
                         "turn_window": summarize([row for row in selected
                                                   if row["truth_turns_within_0_9_s"]]),
                         "no_turn_window": summarize([row for row in selected
                                                      if not row["truth_turns_within_0_9_s"]]),
                         "steps": {str(step): summarize([row for row in selected
                                                         if row["step"] == step])
                                   for step in range(1, 10)}}
    report = {"schema": "rm_dynamic_prediction_causal_face_velocity_probe/v1",
              "scope": "Four-source past-only OLS on already inspected same-box Y data. No tracker modification, future input, runtime or MPPI claim.",
              "evaluation_commit": evaluation_commit,
              "source_run_commit": manifest["run_commit"],
              "history_sources": 4, "history_span_s": [0.15, 0.30],
              "input_sha256": hashes, "results": results,
              "nine_step_y_coverage_failed": any(
                  results[side]["all"]["causal_y_covered_all"] <
                  results[side]["all"]["source_steps"] for side in ("south", "north")),
              "details_sha256": digest(output / "source_steps.csv")}
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"failed": report["nine_step_y_coverage_failed"],
                      "results": {side: {"total_sources": results[side]["total_sources"],
                                         "valid_sources": results[side]["valid_sources"],
                                         "velocity_error_mps": results[side]["velocity_error_mps"],
                                         "all": results[side]["all"]}
                                  for side in ("south", "north")}}, indent=2))


if __name__ == "__main__":
    main()
