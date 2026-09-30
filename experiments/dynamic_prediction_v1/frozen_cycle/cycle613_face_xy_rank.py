#!/usr/bin/env python3
"""Read-only pre-registered scan-box and V1 ranking probe on phase-2 cycle 613."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from native_critic_sensitivity import aggregate, open_loop_geometry
from phase2_filtered_audit import candidates, selected

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "experiments/dynamic_prediction_v1/paired_view"))
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_tracker_probe_20260924"))
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
from moving_view_crosscheck import interpolate_robot, scan_points
from source_x_interval_probe import confirmed_predictions
from probe import static_map
from rm_dynamic_obstacle_tracking.core import dynamic_candidates
from dynamic_metrics import rows_from_transport

A = 0.5551652475612764
SIGMA_RANGE = .01
HALF_WIDTH = .225
HALF_HEIGHT = .275
POINT_ALLOWANCE = .05
SOURCE_ALLOWANCE_Y = .05
HARD = 1_000_000.
NEAR = 300.


def load_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def validate_inputs(trial, evidence):
    manifest = json.loads((evidence / "manifest.json").read_text())
    paths = {"predictions.jsonl": trial / "predictions.jsonl",
             "scans.jsonl": trial / "observation/scans.jsonl",
             "trajectory.jsonl": trial / "observation/trajectory.jsonl",
             "gazebo_poses.jsonl": trial / "gazebo_poses.jsonl"}
    hashes = {}
    for name, path in paths.items():
        hashes[name] = digest(path)
        if hashes[name] != manifest["compressed_streams"][name]["source_sha256"]:
            raise ValueError(f"frozen trial stream changed: {name}")
    fixed = json.loads((evidence / "filtered_summary.json").read_text())
    for name, key in (("filtered_candidates.csv", "detail_sha256"),):
        hashes[name] = digest(evidence / name)
        if hashes[name] != fixed[key]:
            raise ValueError(f"frozen candidate table changed: {name}")
    for name, key in (("filtered_native_scores.bin", "filtered_native_scores"),
                      ("filtered_static_mask.txt", "filtered_static_mask")):
        hashes[name] = digest(evidence / name)
        if hashes[name] != fixed["input_sha256"][key]:
            raise ValueError(f"frozen native result changed: {name}")
    if digest(evidence / "filtered_inputs.json") != fixed["input_sha256"]["inputs"]:
        raise ValueError("frozen fixture manifest changed")
    return paths, hashes, fixed


def four_sources(paths, stamp, consumed_track):
    sources = confirmed_predictions(paths["predictions.jsonl"])
    key = round(stamp, 6)
    if key not in sources or sources[key] != consumed_track:
        raise ValueError("critic-consumed track differs from recorded source")
    prior = sorted(t for t in sources if t <= key)[-4:]
    if len(prior) != 4 or prior[-1] != key or not .15 <= prior[-1] - prior[0] <= .30:
        raise ValueError("four-frame historical span unavailable")
    scans = {round(row["t"], 6): row for row in load_rows(paths["scans.jsonl"])}
    trajectory = load_rows(paths["trajectory.jsonl"])
    traj_times = [row["t"] for row in trajectory]
    occupancy, _ = static_map()
    output = []
    for t in prior:
        if t not in scans:
            raise ValueError(f"no exact source scan: {t}")
        robot = interpolate_robot(trajectory, traj_times, t)
        if robot is None:
            raise ValueError(f"no online pose at source: {t}")
        track = sources[t]
        side = "north" if robot[1] > track["xy"][1] else "south"
        dynamic = dynamic_candidates(scan_points(scans[t], robot), occupancy, .25, True)
        points = [p for p in dynamic if abs(p.x - track["xy"][0]) <= .35 and
                  abs(p.y - track["xy"][1]) <= .50]
        if not points:
            raise ValueError(f"source dynamic points absent: {t}")
        xlo = max(p.x for p in points) - HALF_WIDTH - POINT_ALLOWANCE
        xhi = min(p.x for p in points) + HALF_WIDTH + POINT_ALLOWANCE
        if not math.isfinite(xlo + xhi) or xlo > xhi:
            raise ValueError(f"invalid source X interval: {t}")
        y = (max(p.y for p in points) - HALF_HEIGHT if side == "north" else
             min(p.y for p in points) + HALF_HEIGHT)
        output.append({"source_t": t, "side": side, "selected_points": len(points),
                       "x_center_lower_m": xlo, "x_center_upper_m": xhi,
                       "x_mid_m": (xlo + xhi) / 2, "y_center_m": y,
                       "robot_pose": robot, "tracker_xy": track["xy"]})
    if len({item["side"] for item in output}) != 1:
        raise ValueError("observation side changed within velocity history")
    return output


def slope(rows, key):
    times = [row["source_t"] for row in rows]
    values = [row[key] for row in rows]
    mt, mv = statistics.mean(times), statistics.mean(values)
    sxx = sum((t - mt) ** 2 for t in times)
    if sxx <= 0:
        raise ValueError("degenerate source history")
    velocity = sum((t - mt) * (v - mv) for t, v in zip(times, values)) / sxx
    half_velocity = 3 * SIGMA_RANGE / math.sqrt(sxx) + A * (times[-1] - mt)
    return velocity, half_velocity


def boxes(history, age, dt):
    last = history[-1]
    vx, vx_half = slope(history, "x_mid_m")
    vy, vy_half = slope(history, "y_center_m")
    output = []
    for step in range(1, 10):
        h = age + step * dt
        extra = .5 * A * h * h
        xlo = last["x_center_lower_m"] + vx * h - HALF_WIDTH - vx_half * h - extra
        xhi = last["x_center_upper_m"] + vx * h + HALF_WIDTH + vx_half * h + extra
        yc = last["y_center_m"] + vy * h
        yhalf = HALF_HEIGHT + SOURCE_ALLOWANCE_Y + vy_half * h + extra
        box = (xlo, xhi, yc - yhalf, yc + yhalf)
        if not all(math.isfinite(v) for v in box) or xlo > xhi:
            raise ValueError("invalid future occupancy")
        output.append({"step": step, "horizon_from_source_s": h,
                       "min_x": xlo, "max_x": xhi, "min_y": yc - yhalf,
                       "max_y": yc + yhalf})
    return output, {"x_slope_mps": vx, "x_velocity_half_width_mps": vx_half,
                    "y_slope_mps": vy, "y_velocity_half_width_mps": vy_half}


def polygon(box):
    return ((box["min_x"], box["min_y"]), (box["max_x"], box["min_y"]),
            (box["max_x"], box["max_y"]), (box["min_x"], box["max_y"]))


def rank(scores, safe):
    order = np.argsort(scores, kind="stable")
    found = np.flatnonzero(safe[order])
    return {"best_joint_safe_rank": int(found[0] + 1) if len(found) else None,
            "joint_safe_top_10": int(np.count_nonzero(safe[order[:10]])),
            "top_rollout": int(order[0]), "top_joint_safe": bool(safe[order[0]])}


def run(trial, evidence, output):
    if output.exists():
        raise FileExistsError(output)
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).strip():
        raise ValueError("rank probe and preregistration must be committed")
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    trial, evidence = trial.resolve(), evidence.resolve()
    paths, hashes, prior, = validate_inputs(trial, evidence)
    cycle, meta, arrays, settings, profile_path, truth_path = selected(trial)
    if meta["cycle_id"] != 613:
        raise ValueError("wrong frozen cycle")
    prediction = analyze.event(meta, "prediction.input")
    tracks = [t for t in prediction["tracks"] if t["state"] == 2]
    if len(tracks) != 1 or prediction["status"] != "accepted":
        raise ValueError("single accepted confirmed track required")
    stamp = prediction["source_stamp_ns"] / 1e9
    source_history = four_sources(paths, stamp, tracks[0])
    future, movement = boxes(source_history, prediction["source_age_s"], settings["dt"])
    controls, filtered, poses, history = candidates(trial, meta, arrays, settings)
    with (evidence / "filtered_candidates.csv").open() as stream:
        candidate_rows = list(csv.DictReader(stream))
    if len(candidate_rows) != 300 or any(int(row["rollout"]) != i for i, row in
                                         enumerate(candidate_rows)):
        raise ValueError("candidate table is not the frozen full batch")
    safe = np.asarray([row["filtered_joint_safe"] == "1" for row in candidate_rows])
    if int(safe.sum()) != prior["filtered_joint_safe_count"]:
        raise ValueError("frozen safe count changed")
    native = np.fromfile(evidence / "filtered_native_scores.bin", dtype="<f4")
    if native.shape != (300,) or not np.isfinite(native).all():
        raise ValueError("native standard critic scores invalid")
    first_hit = np.zeros(300, dtype=np.int16)
    near_count = np.zeros(300, dtype=np.int16)
    minimum_gap = np.full(300, np.inf)
    hits = []
    for item in future:
        step = item["step"]
        physical_box = polygon(item)
        count = 0
        for i in range(300):
            robot = analyze.placed(meta["padded_footprint"],
                                   tuple(float(x) for x in poses[i, step - 1]))
            gap = analyze.polygon_distance(robot, physical_box)
            minimum_gap[i] = min(minimum_gap[i], gap)
            if gap <= 1e-9:
                count += 1
                if first_hit[i] == 0:
                    first_hit[i] = step
            elif gap < .02:
                near_count[i] += 1
        hits.append(count)
    score = np.asarray((3.81 / 254.) / 9 * np.where(
        first_hit > 0, HARD, NEAR * near_count), dtype=np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in ("vx", "vy", "wz")], axis=-1)
    combined = aggregate(native + score, controls, initial, settings, history)
    truth = rows_from_transport(truth_path)
    truth_times = [row["t"] for row in truth]
    full_coverage = []
    for item in future:
        t = prediction["consumer_sim_s"] + item["step"] * settings["dt"]
        real = analyze.placed(analyze.obstacle_polygon(),
                              analyze.interpolated_pose(truth, truth_times, t))
        xs, ys = zip(*real)
        full_coverage.append(item["min_x"] <= min(xs) + 1e-9 and
                             item["max_x"] >= max(xs) - 1e-9 and
                             item["min_y"] <= min(ys) + 1e-9 and
                             item["max_y"] >= max(ys) - 1e-9)
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, truth_times,
                  prediction["consumer_sim_s"] + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    aggregate_gap = open_loop_geometry(combined["filtered_sequence"], meta, settings,
                                       actual, body, meta["padded_footprint"])
    output.mkdir(parents=True)
    with (output / "candidates.csv").open("x", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("rollout", "joint_safe", "truth_body_gap_m", "native_static_collision",
                         "first_prediction_hit_step", "min_prediction_padded_gap_m", "near_steps",
                         "new_v1_score", "native_standard_score", "new_total_weighted_score",
                         "new_probability"))
        for i, row in enumerate(candidate_rows):
            writer.writerow((i, int(safe[i]), row["filtered_truth_body_gap_m"],
                             row["native_static_collision"], first_hit[i], minimum_gap[i],
                             near_count[i], score[i], native[i], combined["weighted"][i],
                             combined["probability"][i]))
    report = {"schema": "rm_dynamic_prediction_cycle613_face_xy_rank/v1",
              "scope": "Fixed already-inspected cycle; offline input/score/aggregation mechanism diagnostic only, not blind validation or runtime deployment.",
              "evaluation_commit": commit,
              "cycle_id": 613, "cycle_json_sha256": digest(cycle),
              "cycle_binary_sha256": digest(cycle.with_suffix(".bin")),
              "profile_sha256": digest(profile_path),
              "input_sha256": hashes,
              "source_stamp_s": stamp, "consumer_sim_s": prediction["consumer_sim_s"],
              "source_age_s": prediction["source_age_s"],
              "source_history": source_history, "movement": movement,
              "prediction_boxes": future,
              "nine_step_full_physical_box_covered": full_coverage,
              "predicted_hits_by_step": hits,
              "predicted_any_hit": int(np.count_nonzero(first_hit)),
              "original_v1_hits_by_step": prior["prediction_hits_by_step"],
              "score_span": float(np.ptp(score)),
              "joint_safe_count": int(safe.sum()),
              "joint_safe_ids": np.flatnonzero(safe).tolist(),
              "v1_only_rank": rank(score, safe),
              "native_plus_v1_rank": rank(native + score, safe),
              "weighted_total_rank": rank(combined["weighted"], safe),
              "probability_on_joint_safe": float(combined["probability"][safe].sum()),
              "effective_sample_size": float(1. / np.sum(combined["probability"] ** 2)),
              "returned_control": combined["returned_control"],
              "aggregate_truth": aggregate_gap,
              "details_sha256": digest(output / "candidates.csv")}
    (output / "summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({key: report[key] for key in (
        "source_stamp_s", "movement", "predicted_hits_by_step", "predicted_any_hit",
        "nine_step_full_physical_box_covered", "score_span", "joint_safe_ids",
        "v1_only_rank", "native_plus_v1_rank", "weighted_total_rank",
        "probability_on_joint_safe", "aggregate_truth")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("trial", "evidence", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    run(args.trial, args.evidence, args.output)
