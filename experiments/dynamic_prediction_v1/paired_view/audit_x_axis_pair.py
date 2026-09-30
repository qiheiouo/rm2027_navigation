#!/usr/bin/env python3
"""Audit frozen X-slider paired views and the fixed nine-step XY interval."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_tracker_probe_20260924"))
from dynamic_metrics import obstacle_polygon, placed, rows_from_transport
from probe import static_map
from rm_dynamic_obstacle_tracking.core import dynamic_candidates
from audit_pair import PROFILE, RAW_FILES, at, geometry
from visible_face_probe import odom_at, scan_points

A = .5551652475612764
SIGMA = .01
SOURCE_BEGIN = 16.0
SOURCE_END = 43.9
FULL_FUTURE_END = 43.0


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def slope(rows, key):
    times = [r["source_t"] for r in rows]
    values = [r[key] for r in rows]
    mt, mv = statistics.mean(times), statistics.mean(values)
    sxx = sum((t - mt) ** 2 for t in times)
    return (sum((t - mt) * (v - mv) for t, v in zip(times, values)) / sxx,
            3 * SIGMA / math.sqrt(sxx) + A * (times[-1] - mt))


def source_predictions(path):
    found = {}
    for row in read_rows(path):
        t = round(row["source_t"], 6)
        confirmed = [track for track in row["tracks"] if track["state"] == 2]
        if len(confirmed) != 1 or not row["complete"] or row["frame"] != "odom":
            continue
        signature = (confirmed[0]["xy"], confirmed[0]["vxy"], confirmed[0]["size_xy"])
        if t in found:
            if found[t]["signature"] != signature:
                raise ValueError(f"conflicting repeated tracker source: {t}")
            continue
        found[t] = {"track": confirmed[0], "signature": signature}
    return {t: row["track"] for t, row in found.items()}


def physical_box_at(poses, times, t):
    return at(poses, times, t)["obstacle"]


def audit_side(trial, side, body, box, full_future_end=FULL_FUTURE_END,
               min_stop_sim_s=44.0):
    observation = json.loads((trial / "observation_summary.json").read_text())
    if observation["actual_stop_sim_s"] < min_stop_sim_s or \
            (trial / "docker_exit.txt").read_text().strip() != "0":
        raise ValueError(f"incomplete one-shot capture: {side}")
    poses = rows_from_transport(trial / "gazebo_poses.jsonl")
    times = [row["t"] for row in poses]
    odom = read_rows(trial / "odometry.jsonl")
    odom_times = [row["t"] for row in odom]
    scans = {round(row["t"], 6): row for row in read_rows(trial / "scans.jsonl")}
    tracks = source_predictions(trial / "predictions.jsonl")
    occupancy, _ = static_map()
    source_rows, step_rows, valid_history = [], [], []
    for stamp in sorted(tracks):
        if stamp > SOURCE_END:
            continue
        track = tracks[stamp]
        scan = scans.get(stamp)
        try:
            robot = odom_at(odom, odom_times, stamp)
        except ValueError:
            robot = None
        points = []
        if scan is not None and robot is not None:
            points = [p for p in dynamic_candidates(
                scan_points(scan, robot), occupancy, .25, True)
                if abs(p.x - track["xy"][0]) <= .35 and
                   abs(p.y - track["xy"][1]) <= .50]
        online_side = ("north" if robot[1] > track["xy"][1] else "south") if robot else None
        xlo = max(p.x for p in points) - .225 - .05 if points else None
        xhi = min(p.x for p in points) + .225 + .05 if points else None
        interval_valid = xlo is not None and xlo <= xhi
        y = ((max(p.y for p in points) - .275) if online_side == "north" else
             (min(p.y for p in points) + .275)) if points else None
        xmid = (xlo + xhi) / 2 if interval_valid else None
        if interval_valid and y is not None:
            valid_history.append({"source_t": stamp, "x_mid": xmid, "y_center": y})
        if not SOURCE_BEGIN <= stamp <= SOURCE_END:
            continue
        physical = at(poses, times, stamp)
        actual_x, actual_y = physical["obstacle"][:2]
        truth_side = "north" if physical["robot"][1] > actual_y else "south"
        pose_error = math.hypot(robot[0] - physical["robot"][0],
                                robot[1] - physical["robot"][1]) if robot else None
        physical_vx = (physical_box_at(poses, times, stamp + .1)[0] -
                       physical_box_at(poses, times, stamp - .1)[0]) / .2
        history = valid_history[-4:] if len(valid_history) >= 4 else None
        if history and not .15 <= history[-1]["source_t"] - history[0]["source_t"] <= .30:
            history = None
        vx, ux = slope(history, "x_mid") if history else (None, None)
        vy, uy = slope(history, "y_center") if history else (None, None)
        future_available = stamp + .9 <= times[-1] + 1e-9
        source = {"side": side, "source_t": stamp,
                  "scan_exact_match": scan is not None, "online_pose_available": robot is not None,
                  "selected_points": len(points), "online_side": online_side,
                  "truth_side": truth_side,
                  "online_pose_error_m": pose_error,
                  "physical_x_m": actual_x, "physical_y_m": actual_y,
                  "physical_vx_mps": physical_vx,
                  "tracker_x_m": track["xy"][0], "tracker_y_m": track["xy"][1],
                  "source_x_lower_m": xlo, "source_x_upper_m": xhi,
                  "source_x_valid": interval_valid,
                  "source_x_covers_center": bool(interval_valid and xlo <= actual_x <= xhi),
                  "source_y_center_m": y,
                  "source_y_error_m": y - actual_y if y is not None else None,
                  "history_valid": history is not None,
                  "causal_vx_mps": vx, "causal_vy_mps": vy,
                  "velocity_halfwidth_x_mps": ux, "velocity_halfwidth_y_mps": uy,
                  "future_nine_step_label_available": future_available}
        source_rows.append(source)
        if not future_available or stamp > full_future_end:
            continue
        for step in range(1, 10):
            h = round(step * .1, 1)
            true_pose = physical_box_at(poses, times, stamp + h)
            true_x, true_y = true_pose[:2]
            true_shape = placed(box, true_pose)
            physical_xlo = min(p[0] for p in true_shape)
            physical_xhi = max(p[0] for p in true_shape)
            physical_ylo = min(p[1] for p in true_shape)
            physical_yhi = max(p[1] for p in true_shape)
            if history:
                extra = .5 * A * h * h
                pred_xlo = xlo + vx * h - .225 - ux * h - extra
                pred_xhi = xhi + vx * h + .225 + ux * h + extra
                pred_yc = y + vy * h
                yh = .275 + .05 + uy * h + extra
                pred_ylo, pred_yhi = pred_yc - yh, pred_yc + yh
                x_excess = max(0., pred_xlo - physical_xlo,
                               physical_xhi - pred_xhi)
                y_excess = max(0., pred_ylo - physical_ylo,
                               physical_yhi - pred_yhi)
            else:
                pred_xlo = pred_xhi = pred_ylo = pred_yhi = None
                x_excess = y_excess = None
            step_rows.append({"side": side, "source_t": stamp, "step": step,
                              "horizon_s": h, "physical_x_m": true_x,
                              "physical_y_m": true_y,
                              "physical_min_x": physical_xlo, "physical_max_x": physical_xhi,
                              "physical_min_y": physical_ylo, "physical_max_y": physical_yhi,
                              "history_valid": history is not None,
                              "pred_min_x": pred_xlo, "pred_max_x": pred_xhi,
                              "pred_min_y": pred_ylo, "pred_max_y": pred_yhi,
                              "x_excess_m": x_excess, "y_excess_m": y_excess,
                              "x_covered": x_excess is not None and x_excess <= 1e-9,
                              "y_covered": y_excess is not None and y_excess <= 1e-9,
                              "xy_covered": x_excess is not None and
                                            x_excess <= 1e-9 and y_excess <= 1e-9})
    phase = {"positive_gt_0_3": sum(r["physical_vx_mps"] > .3 for r in source_rows),
             "negative_lt_minus_0_3": sum(r["physical_vx_mps"] < -.3 for r in source_rows),
             "turn_abs_le_0_2": sum(abs(r["physical_vx_mps"]) <= .2 for r in source_rows)}
    body_gap = geometry(poses, body, box)
    input_gate = (observation["scan_count"] >= 100 and observation["odom_count"] >= 100 and
                  len(poses) > 0 and len(source_rows) >= 40 and
                  all(value > 0 for value in phase.values()) and
                  body_gap["at_least_0_05_m"] and
                  all(r["online_pose_error_m"] is not None and
                      r["online_pose_error_m"] <= .03 and
                      r["online_side"] == r["truth_side"] == side
                      for r in source_rows))
    valid_steps = [r for r in step_rows if r["history_valid"]]
    early = [r for r in source_rows if r["source_t"] <= full_future_end]
    return {"side": side, "observation": observation,
            "physical_pose_rows": len(poses), "confirmed_sources_in_window": len(source_rows),
            "phase_counts": phase, "physical_body_gap": body_gap,
            "input_gate_passed": input_gate,
            "exact_scan_sources": sum(r["scan_exact_match"] for r in source_rows),
            "online_pose_sources": sum(r["online_pose_available"] for r in source_rows),
            "source_x_valid": sum(r["source_x_valid"] for r in source_rows),
            "source_x_center_covered": sum(r["source_x_covers_center"] for r in source_rows),
            "source_y_center_abs_error_max_m": max((abs(r["source_y_error_m"]) for r in source_rows
                                                  if r["source_y_error_m"] is not None), default=None),
            "history_valid_sources": sum(r["history_valid"] for r in source_rows),
            "future_label_available_sources": sum(r["future_nine_step_label_available"] for r in source_rows),
            "future_label_missing_sources": sum(not r["future_nine_step_label_available"] for r in source_rows),
            "diagnostic_early_source_count": len(early),
            "diagnostic_early_history_valid_sources": sum(r["history_valid"] for r in early),
            "diagnostic_valid_step_count": len(valid_steps),
            "diagnostic_x_covered_valid_steps": sum(r["x_covered"] for r in valid_steps),
            "diagnostic_y_covered_valid_steps": sum(r["y_covered"] for r in valid_steps),
            "diagnostic_xy_covered_valid_steps": sum(r["xy_covered"] for r in valid_steps),
            "diagnostic_x_excess_max_m": max((r["x_excess_m"] for r in valid_steps), default=None),
            "diagnostic_y_excess_max_m": max((r["y_excess_m"] for r in valid_steps), default=None),
            "full_preregistered_nine_step_gate_passed": (
                input_gate and len(source_rows) == len(early) and
                all(r["history_valid"] for r in source_rows) and
                all(r["xy_covered"] for r in step_rows))}, source_rows, step_rows


def compress(source, target):
    with source.open("rb") as inp, target.open("xb") as out:
        with gzip.GzipFile(fileobj=out, filename="", mode="wb", mtime=0) as zipped:
            while chunk := inp.read(1024 * 1024):
                zipped.write(chunk)


def write_csv(path, rows):
    if not rows:
        raise ValueError(f"empty evidence table: {path}")
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


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
        raise ValueError("audit rule and protocol note must be committed")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                     text=True).strip()
    plan = json.loads((series / "plan.json").read_text())
    if plan["schema"] != "rm_dynamic_prediction_x_axis_pair/v1":
        raise ValueError("wrong series plan")
    for name, expected in plan["derived_world_sha256"].items():
        if digest(series / "worlds" / f"{name}.sdf") != expected:
            raise ValueError(f"derived world changed: {name}")
    if digest(series / "models/x_axis_obstacle.sdf") != plan["derived_model_sha256"]:
        raise ValueError("derived moving model changed")
    profile = yaml.safe_load(PROFILE.read_text())
    body = [tuple(v) for v in yaml.safe_load(profile["local_costmap"]["local_costmap"]
                                            ["ros__parameters"]["footprint"])]
    box = obstacle_polygon()
    results, tables = {}, {}
    for side in ("south", "north"):
        result, sources, steps = audit_side(series / side, side, body, box)
        results[side] = result
        tables[side] = (sources, steps)
    output.mkdir(parents=True)
    (output / "plan.json").write_bytes((series / "plan.json").read_bytes())
    manifest = {"schema": "rm_dynamic_prediction_x_axis_evidence/v1",
                "source_root": str(series), "run_commit": plan["run_commit"],
                "evaluation_commit": commit, "profile_sha256": digest(PROFILE),
                "source_sha256": {}, "packaged_sha256": {}}
    for side in ("south", "north"):
        sources, steps = tables[side]
        for name, rows in (("sources.csv", sources), ("source_steps.csv", steps)):
            target = output / f"{side}_{name}"
            write_csv(target, rows)
            manifest["packaged_sha256"][target.name] = digest(target)
        for name in RAW_FILES:
            source = series / side / name
            key = f"{side}/{name}"
            manifest["source_sha256"][key] = digest(source)
            target = output / f"{side}_{name}.gz"
            compress(source, target)
            manifest["packaged_sha256"][target.name] = digest(target)
    for name in ("models/x_axis_obstacle.sdf", "worlds/south.sdf", "worlds/north.sdf"):
        source = series / name
        target = output / source.name.replace(".sdf", f"_{name.split('/')[0]}.sdf")
        target.write_bytes(source.read_bytes())
        manifest["source_sha256"][name] = digest(source)
        manifest["packaged_sha256"][target.name] = digest(target)
    report = {"schema": "rm_dynamic_prediction_x_axis_pair_audit/v1",
              "scope": "Independent X-motion simulator observation. Last 0.9 s of preregistered source window cannot be fully labeled; full nine-step gate fails, early subset is diagnostic only. No runtime critic change.",
              "plan_sha256": digest(series / "plan.json"),
              "evaluation_commit": commit, "results": results,
              "pair_input_gate_passed": all(r["input_gate_passed"] for r in results.values()),
              "pair_full_nine_step_gate_passed": all(
                  r["full_preregistered_nine_step_gate_passed"] for r in results.values())}
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    for name in ("plan.json", "summary.json"):
        manifest["packaged_sha256"][name] = digest(output / name)
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2,
                                                     sort_keys=True) + "\n")
    print(json.dumps({"pair_input_gate_passed": report["pair_input_gate_passed"],
                      "pair_full_nine_step_gate_passed": report["pair_full_nine_step_gate_passed"],
                      "results": results}, indent=2))


if __name__ == "__main__":
    main()
