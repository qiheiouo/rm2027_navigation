#!/usr/bin/env python3
"""Audit the one-shot near X-slider observation against the frozen full window."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import statistics
import subprocess

import yaml

from audit_pair import PROFILE, RAW_FILES
from audit_x_axis_pair import audit_side
from dynamic_metrics import obstacle_polygon

ROOT = Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_csv(path, rows):
    if not rows:
        raise ValueError(f"empty near-view evidence table: {path}")
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def compress(source, target):
    with source.open("rb") as inp, target.open("xb") as out:
        with gzip.GzipFile(fileobj=out, filename="", mode="wb", mtime=0) as zipped:
            while chunk := inp.read(1024 * 1024):
                zipped.write(chunk)


def groups(sources, steps):
    by_t = {}
    for step in steps:
        by_t.setdefault(step["source_t"], []).append(step)
    source_by_t = {row["source_t"]: row for row in sources}
    turn = set()
    for stamp, sequence in by_t.items():
        sequence.sort(key=lambda row: row["step"])
        if [row["step"] for row in sequence] != list(range(1, 10)):
            raise ValueError("incomplete nine-step near source")
        xs = [source_by_t[stamp]["physical_x_m"]] + [row["physical_x_m"]
                                                      for row in sequence]
        velocities = [(b - a) / .1 for a, b in zip(xs, xs[1:])]
        if any(v > .05 for v in velocities) and any(v < -.05 for v in velocities):
            turn.add(stamp)
    def describe(rows):
        if not rows:
            return {"sources": 0, "steps": 0, "x_covered": 0,
                    "y_covered": 0, "xy_covered": 0}
        ids = {row["source_t"] for row in rows}
        first = [row for row in rows if row["step"] == 1]
        ninth = [row for row in rows if row["step"] == 9]
        if len(rows) != 9 * len(ids) or not (len(first) == len(ninth) == len(ids)):
            raise ValueError("group source/step counts differ")
        def width(items, axis):
            return statistics.median(row[f"pred_max_{axis}"] - row[f"pred_min_{axis}"]
                                     for row in items)
        return {"sources": len(ids), "steps": len(rows),
                "x_covered": sum(row["x_covered"] for row in rows),
                "y_covered": sum(row["y_covered"] for row in rows),
                "xy_covered": sum(row["xy_covered"] for row in rows),
                "x_width_step1_median_m": width(first, "x"),
                "x_width_step9_median_m": width(ninth, "x"),
                "y_width_step1_median_m": width(first, "y"),
                "y_width_step9_median_m": width(ninth, "y")}
    return {"all": describe(steps),
            "turn": describe([row for row in steps if row["source_t"] in turn]),
            "no_turn": describe([row for row in steps if row["source_t"] not in turn])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    series, output = args.series.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT,
                               text=True).strip():
        raise ValueError("near audit must be committed before evaluation")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                     text=True).strip()
    plan = json.loads((series / "plan.json").read_text())
    if plan["schema"] != "rm_dynamic_prediction_x_axis_near/v1" or \
            plan["stop_sim_s"] != 45.0 or plan["source_window_s"] != [16.0, 43.9]:
        raise ValueError("wrong frozen near-view plan")
    world = series / "worlds/north.sdf"
    model = series / "models/x_axis_obstacle.sdf"
    if digest(world) != plan["world_sha256"] or digest(model) != plan["model_sha256"]:
        raise ValueError("derived near fixture changed")
    profile = yaml.safe_load(PROFILE.read_text())
    body = [tuple(v) for v in yaml.safe_load(profile["local_costmap"]["local_costmap"]
                                            ["ros__parameters"]["footprint"])]
    box = obstacle_polygon()
    result, sources, steps = audit_side(series / "north", "north", body, box,
                                        full_future_end=43.9, min_stop_sim_s=45.0)
    count = result["confirmed_sources_in_window"]
    full_input = (result["input_gate_passed"] and
                  result["exact_scan_sources"] == count and
                  result["online_pose_sources"] == count and
                  result["source_x_valid"] == count and
                  result["source_x_center_covered"] == count and
                  result["history_valid_sources"] == count and
                  result["future_label_available_sources"] == count and
                  result["source_y_center_abs_error_max_m"] is not None and
                  result["source_y_center_abs_error_max_m"] <= .05)
    splits = groups(sources, steps)
    full_nine = (full_input and result["full_preregistered_nine_step_gate_passed"] and
                 splits["all"]["sources"] == count and
                 splits["all"]["xy_covered"] == 9 * count)
    output.mkdir(parents=True)
    (output / "plan.json").write_bytes((series / "plan.json").read_bytes())
    write_csv(output / "sources.csv", sources)
    write_csv(output / "source_steps.csv", steps)
    manifest = {"schema": "rm_dynamic_prediction_x_axis_near_evidence/v1",
                "source_root": str(series), "run_commit": plan["run_commit"],
                "evaluation_commit": commit, "profile_sha256": digest(PROFILE),
                "source_sha256": {}, "packaged_sha256": {}}
    for name in RAW_FILES:
        source = series / "north" / name
        manifest["source_sha256"][name] = digest(source)
        target = output / f"{name}.gz"
        compress(source, target)
        manifest["packaged_sha256"][target.name] = digest(target)
    for name, source in (("world.sdf", world), ("model.sdf", model)):
        (output / name).write_bytes(source.read_bytes())
        manifest["source_sha256"][name] = digest(source)
        manifest["packaged_sha256"][name] = digest(output / name)
    summary = {"schema": "rm_dynamic_prediction_x_axis_near_audit/v1",
               "scope": "One new near static X-motion view with complete 0.9-s future labels, same preregistered scan interval rule. No MPPI/control claim.",
               "evaluation_commit": commit, "plan_sha256": digest(series / "plan.json"),
               "profile_sha256": digest(PROFILE), "input_gate_passed": full_input,
               "full_nine_step_gate_passed": full_nine,
               "observation": result, "groups": splits}
    (output / "summary.json").write_text(json.dumps(summary, indent=2,
                                                    sort_keys=True) + "\n")
    for name in ("plan.json", "sources.csv", "source_steps.csv", "summary.json"):
        manifest["packaged_sha256"][name] = digest(output / name)
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2,
                                                     sort_keys=True) + "\n")
    print(json.dumps({"input_gate_passed": full_input,
                      "full_nine_step_gate_passed": full_nine,
                      "observation": result, "groups": splits}, indent=2))


if __name__ == "__main__":
    main()
