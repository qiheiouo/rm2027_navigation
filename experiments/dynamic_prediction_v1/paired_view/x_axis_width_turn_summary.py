#!/usr/bin/env python3
"""Describe the frozen X-motion interval widths and pre-existing turn split."""
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


def read_csv(path, expected):
    if digest(path) != expected:
        raise ValueError(f"frozen table changed: {path}")
    with path.open() as stream:
        return list(csv.DictReader(stream))


def summarize(rows):
    if not rows:
        return {"sources": 0, "source_steps": 0}
    source_ids = {row["source_t"] for row in rows}
    first = [row for row in rows if int(row["step"]) == 1]
    ninth = [row for row in rows if int(row["step"]) == 9]
    if len(rows) != 9 * len(source_ids) or not (
            len(first) == len(ninth) == len(source_ids)):
        raise ValueError("incomplete nine-step source set")
    def widths(items, axis):
        return [float(r[f"pred_max_{axis}"]) - float(r[f"pred_min_{axis}"])
                for r in items]
    return {"sources": len(source_ids), "source_steps": len(rows),
            "x_covered": sum(r["x_covered"] == "True" for r in rows),
            "y_covered": sum(r["y_covered"] == "True" for r in rows),
            "xy_covered": sum(r["xy_covered"] == "True" for r in rows),
            "x_width_step1_median_m": statistics.median(widths(first, "x")),
            "x_width_step9_median_m": statistics.median(widths(ninth, "x")),
            "y_width_step1_median_m": statistics.median(widths(first, "y")),
            "y_width_step9_median_m": statistics.median(widths(ninth, "y"))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    evidence, output = args.evidence.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT,
                               text=True).strip():
        raise ValueError("descriptive summary rule must be committed")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                     text=True).strip()
    manifest = json.loads((evidence / "manifest.json").read_text())
    audited = json.loads((evidence / "summary.json").read_text())
    if not audited["pair_input_gate_passed"] or audited["pair_full_nine_step_gate_passed"]:
        raise ValueError("unexpected primary audit outcome")
    groups, hashes = {}, {"manifest": digest(evidence / "manifest.json"),
                         "primary_summary": digest(evidence / "summary.json")}
    for side in ("south", "north"):
        source_file = evidence / f"{side}_sources.csv"
        step_file = evidence / f"{side}_source_steps.csv"
        sources = read_csv(source_file, manifest["packaged_sha256"][source_file.name])
        steps = read_csv(step_file, manifest["packaged_sha256"][step_file.name])
        hashes[source_file.name] = digest(source_file)
        hashes[step_file.name] = digest(step_file)
        source_by_t = {r["source_t"]: r for r in sources}
        by_t = {}
        for row in steps:
            by_t.setdefault(row["source_t"], []).append(row)
        turn = set()
        for stamp, sequence in by_t.items():
            sequence.sort(key=lambda row: int(row["step"]))
            if [int(r["step"]) for r in sequence] != list(range(1, 10)):
                raise ValueError("source step sequence incomplete")
            xs = [float(source_by_t[stamp]["physical_x_m"])] + [
                float(row["physical_x_m"]) for row in sequence]
            velocities = [(b - a) / .1 for a, b in zip(xs, xs[1:])]
            if any(v > .05 for v in velocities) and any(v < -.05 for v in velocities):
                turn.add(stamp)
        groups[side] = {"all_early": summarize(steps),
                        "turn_window": summarize([r for r in steps if r["source_t"] in turn]),
                        "no_turn_window": summarize([r for r in steps if r["source_t"] not in turn])}
    result = {"schema": "rm_dynamic_prediction_x_axis_width_turn_summary/v1",
              "scope": "Post-capture descriptive widths and truth-labeled turn split on predeclared early subset only. Primary full-window nine-step gate remains failed; no model fitting.",
              "evaluation_commit": commit,
              "turn_rule": "Among physical X positions at source and steps 1..9, adjacent 0.1-s velocities include both >+0.05 and <-0.05 m/s; same rule as previous Y-motion probe.",
              "input_sha256": hashes,
              "primary_full_nine_step_gate_passed": False,
              "groups": groups}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(groups, indent=2))


if __name__ == "__main__":
    main()
