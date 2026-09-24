#!/usr/bin/env python3
"""Audit one *captured* MPPI control cycle; never infer missing rollouts from logs.

The trace must contain an in-critic `prediction.input` event. A separate topic
recorder cannot prove which message the critic actually consumed.
"""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922"))
sys.path.insert(0, str(ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922"))
sys.path.insert(0, str(ROOT / "docs/dynamic_navigation/evidence/v1_extent_shadow_20260924"))
from read_trace import read_cycle, last
from dynamic_metrics import rows_from_transport, placed, obstacle_polygon, polygon_distance
from envelope import predicted_box


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def event(meta, kind):
    rows = [e["value"] for e in meta["events"] if e["kind"] == kind]
    if len(rows) != 1:
        raise ValueError(f"expected exactly one {kind} event; found {len(rows)}")
    return rows[0]


def critic_deltas(arrays):
    """MPPI trace stores costs after each critic; subtraction gives each term."""
    result = {}
    prior = None
    for name, values in arrays:
        if not name.startswith("critic."):
            continue
        key = name.removeprefix("critic.")
        if key in result:
            raise ValueError(f"duplicate critic score: {key}")
        current = np.asarray(values, dtype=np.float64)
        if current.ndim != 1:
            raise ValueError(f"bad critic shape: {key}")
        result[key] = current if prior is None else current - prior
        prior = current
    if not result:
        raise ValueError("trace has no per-critic scores")
    return result, prior


def interpolated_pose(rows, times, t):
    i = bisect.bisect_right(times, t)
    if i == 0 or i == len(rows):
        raise ValueError(f"truth stream does not cover rollout time {t:.6f}")
    a, b = rows[i - 1], rows[i]
    f = (t - a["t"]) / (b["t"] - a["t"])
    pa, pb = a["obstacle"], b["obstacle"]
    yaw_delta = math.remainder(pb[2] - pa[2], 2 * math.pi)
    return (pa[0] + f * (pb[0] - pa[0]),
            pa[1] + f * (pb[1] - pa[1]),
            pa[2] + f * yaw_delta)


def integrate_omni(vx, vy, wz, pose, dt):
    """Mirror Nav2 1.1.20's float Omni integration and previous-yaw translation."""
    vx, vy, wz = (np.asarray(v, dtype=np.float32) for v in (vx, vy, wz))
    if vx.shape != vy.shape or vx.shape != wz.shape or vx.ndim not in (1, 2):
        raise ValueError("bad Omni velocity tensor")
    initial_yaw = np.float32(pose[2])
    yaws = np.cumsum(wz * np.float32(dt), axis=-1, dtype=np.float32) + initial_yaw
    previous_yaws = np.concatenate((np.full((*vx.shape[:-1], 1), initial_yaw,
                                              dtype=np.float32), yaws[..., :-1]), axis=-1)
    dx = vx * np.cos(previous_yaws) - vy * np.sin(previous_yaws)
    dy = vx * np.sin(previous_yaws) + vy * np.cos(previous_yaws)
    xs = np.float32(pose[0]) + np.cumsum(dx * np.float32(dt), axis=-1,
                                          dtype=np.float32)
    ys = np.float32(pose[1]) + np.cumsum(dy * np.float32(dt), axis=-1,
                                          dtype=np.float32)
    return xs, ys, yaws


def analyze(cycle_path, profile_path, truth_path):
    meta, arrays = read_cycle(cycle_path)
    if meta.get("schema") not in ("tdt_mppi_cycle/v1", "rm_dynamic_prediction_cycle/v1"):
        raise ValueError("unknown trace schema")
    raw = last(arrays, "locked.raw_map")
    if raw.shape != (meta["map"]["height"], meta["map"]["width"]):
        raise ValueError("raw map dimensions do not match locked metadata")
    if not meta["path"] or len(meta["padded_footprint"]) < 3:
        raise ValueError("missing path or footprint")
    prediction = event(meta, "prediction.input")
    if prediction.get("status") != "accepted":
        raise ValueError(f"prediction was not accepted: {prediction.get('status')}")
    if prediction.get("schema") != "rm_dynamic_obstacle_predictions/v1" or not prediction.get("complete"):
        raise ValueError("bad prediction schema/completeness")
    if prediction.get("frame") != meta["map"]["frame"] or prediction.get("authority") != "shadow_only":
        raise ValueError("prediction frame/authority mismatch")
    age = float(prediction["source_age_s"])
    consumed_t = float(prediction["consumer_sim_s"])
    if not (math.isfinite(age) and 0 <= age <= .4):
        raise ValueError("invalid source age")
    profile = yaml.safe_load(Path(profile_path).read_text())
    follow = profile["controller_server"]["ros__parameters"]["FollowPath"]
    if int(follow["iteration_count"]) != 1:
        raise ValueError("this analysis requires the frozen single-iteration MPPI profile")
    params = follow["PredictionV1Critic"]
    settings = event(meta, "settings")
    # Nav2 stores model_dt as float. The V1 critic divides by that exact
    # value, so YAML's decimal 0.1 would produce a different floor() result.
    dt = float(settings["dt"])
    horizon_steps = min(int(math.floor(float(params["horizon"]) / dt + 1e-9)),
                        int(last(arrays, "rollout.x").shape[1]))
    if horizon_steps < 1 or age > float(params["max_age"]):
        raise ValueError("prediction rejected by active profile")
    x, y, yaw = (last(arrays, "rollout." + key) for key in ("x", "y", "yaw"))
    if x.shape != y.shape or x.shape != yaw.shape or x.ndim != 2:
        raise ValueError("rollout tensors have incompatible shapes")
    count, steps = x.shape
    if count != int(follow["batch_size"]) or steps != int(follow["time_steps"]):
        raise ValueError("rollout tensors differ from profile")
    if settings["batch"] != count or settings["steps"] != steps or \
            abs(dt - float(follow["model_dt"])) > 1e-7:
        raise ValueError("captured optimizer settings differ from profile")
    for name in ("initial.vx", "initial.vy", "initial.wz", "before_filter.vx",
                 "before_filter.vy", "before_filter.wz", "after_filter.vx",
                 "after_filter.vy", "after_filter.wz"):
        if last(arrays, name).shape != (steps,):
            raise ValueError(f"control sequence missing or malformed: {name}")
    for name in ("cvx", "cvy", "cwz", "vx", "vy", "wz"):
        if last(arrays, "sampled." + name).shape != (count, steps):
            raise ValueError(f"sampled controls missing or malformed: {name}")
    replay = integrate_omni(*(last(arrays, "sampled." + key)
                              for key in ("vx", "vy", "wz")), meta["pose"], dt)
    replay_error = max(float(np.max(np.abs(actual - generated)))
                       for actual, generated in zip((x, y, yaw), replay))
    if replay_error > 1e-5:
        raise ValueError(f"frozen Omni replay differs from captured rollouts: {replay_error}")
    terms, total = critic_deltas(arrays)
    if any(term.shape != (count,) for term in terms.values()):
        raise ValueError("critic score count differs from rollout count")
    pred_keys = [name for name in terms if name.endswith("PredictionV1Critic")]
    if len(pred_keys) != 1:
        raise ValueError("expected one PredictionV1Critic score term")
    pred_key = pred_keys[0]
    captured_pred_score = terms[pred_key]
    if not np.allclose(total, last(arrays, "scored.costs"), rtol=1e-5, atol=1e-3):
        raise ValueError("critic cumulative scores do not match final MPPI costs")
    tracks = [tr for tr in prediction["tracks"] if tr["state"] == 2]
    if not tracks:
        raise ValueError("accepted prediction has no confirmed tracks")
    boxes = [[predicted_box(tr["xy"], tr["vxy"], tr["size_xy"],
                            (float(params["object_width"]), float(params["object_height"])),
                            age, (j + 1) * dt, float(params["reference_acceleration"]))
              for tr in tracks] for j in range(horizon_steps)]
    truth = rows_from_transport(truth_path)
    truth_times = [row["t"] for row in truth]
    if consumed_t + steps * dt > truth_times[-1] or consumed_t + dt < truth_times[0]:
        raise ValueError("truth stream cannot cover the full MPPI horizon")
    actual_box = obstacle_polygon()
    padded_shape = [tuple(p) for p in meta["padded_footprint"]]
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    source_body = local["footprint"]
    body_shape = [tuple(p) for p in yaml.safe_load(source_body) if len(p) == 2]
    if len(body_shape) < 3:
        raise ValueError("missing unpadded reference body footprint")
    # Require this relation from the unchanged 0.03 m profile. The polygon
    # padding algorithm is Nav2-specific; do not silently reconstruct it here.
    if abs(float(local["footprint_padding"]) - .03) > 1e-9:
        raise ValueError("footprint padding differs from frozen fixture")
    first_spread = max(float(np.ptp(values[:, 0])) for values in
                       (x, y, yaw, *(last(arrays, "sampled." + key)
                                      for key in ("vx", "vy", "wz"))))
    first_pose = (float(x[0, 0]), float(y[0, 0]), float(yaw[0, 0]))
    first_truth_pose = interpolated_pose(truth, truth_times, consumed_t + dt)
    first_truth_body_gap = polygon_distance(placed(body_shape, first_pose),
                                            placed(actual_box, first_truth_pose))
    first_pred_gap = min(polygon_distance(placed(padded_shape, first_pose), box.polygon())
                         for box in boxes[0])
    optimized = integrate_omni(*(last(arrays, "after_filter." + key)
                                  for key in ("vx", "vy", "wz")), meta["pose"], dt)
    optimized_truth_gaps = [polygon_distance(
        placed(body_shape, tuple(float(v[j]) for v in optimized)),
        placed(actual_box, interpolated_pose(truth, truth_times,
                                             consumed_t + (j + 1) * dt)))
        for j in range(steps)]
    output = event(meta, "output")
    selected_index = int(settings["offset"])
    filtered_output = [float(last(arrays, "after_filter." + key)[selected_index])
                       for key in ("vx", "vy", "wz")]
    if not np.allclose(output, filtered_output, rtol=0, atol=1e-6):
        raise ValueError("returned MPPI command differs from filtered sequence")
    weighted = last(arrays, "weighted.costs")
    if weighted.shape != (count,):
        raise ValueError("weighted score count differs from rollout count")
    static_mask = last(arrays, "cost_critic.collisions")
    if static_mask.shape != (count,):
        raise ValueError("CostCritic collision mask differs from rollout count")
    weights = last(arrays, "weighted.probability")
    if weights.shape != (count,) or not np.all(np.isfinite(weights)) or np.any(weights < 0) or \
            abs(float(weights.sum()) - 1.) > 1e-4:
        raise ValueError("invalid MPPI aggregation probabilities")
    uncovered_times = []
    for j, step_boxes in enumerate(boxes):
        actual_pose = interpolated_pose(truth, truth_times, consumed_t + (j + 1) * dt)
        actual_vertices = placed(actual_box, actual_pose)
        if not any(all(box.min_x - 1e-8 <= px <= box.max_x + 1e-8 and
                       box.min_y - 1e-8 <= py <= box.max_y + 1e-8
                       for px, py in actual_vertices) for box in step_boxes):
            uncovered_times.append((j + 1) * dt)
    records = []
    for i in range(count):
        true_body_gap = math.inf
        true_padded_gap = math.inf
        pred_gap = math.inf
        first_conflict = None
        first_margin_breach = None
        near_count = 0
        collision = False
        for j in range(steps):
            pose = (float(x[i, j]), float(y[i, j]), float(yaw[i, j]))
            truth_pose = interpolated_pose(truth, truth_times, consumed_t + (j + 1) * dt)
            actual = placed(actual_box, truth_pose)
            true_body_gap = min(true_body_gap, polygon_distance(placed(body_shape, pose), actual))
            true_padded_gap = min(true_padded_gap, polygon_distance(placed(padded_shape, pose), actual))
            if j >= horizon_steps:
                continue
            polygon = placed(padded_shape, pose)
            for box in boxes[j]:
                gap = polygon_distance(polygon, box.polygon())
                pred_gap = min(pred_gap, gap)
                if gap <= 1e-9:
                    collision = True
                    if first_conflict is None:
                        first_conflict = (j + 1) * dt
                elif gap < .02:
                    near_count += 1
                    if first_margin_breach is None:
                        first_margin_breach = (j + 1) * dt
            # V1 stops scoring this rollout after first collision.
            if collision:
                break
        # Truth metric must use *all* MPPI steps even when prediction collides.
        if collision:
            for j in range(j + 1, steps):
                pose = (float(x[i, j]), float(y[i, j]), float(yaw[i, j]))
                truth_pose = interpolated_pose(truth, truth_times, consumed_t + (j + 1) * dt)
                actual = placed(actual_box, truth_pose)
                true_body_gap = min(true_body_gap, polygon_distance(placed(body_shape, pose), actual))
                true_padded_gap = min(true_padded_gap, polygon_distance(placed(padded_shape, pose), actual))
        repulsive = 1_000_000. if collision else 300. * near_count
        calculated_score = (3.81 / 254.) * repulsive / horizon_steps
        rec = {"rollout": i, "truth_body_min_clearance_m": true_body_gap,
               "truth_padded_min_clearance_m": true_padded_gap,
               "predicted_padded_min_clearance_m": pred_gap,
               "first_predicted_conflict_s": first_conflict,
               "first_predicted_margin_breach_s": first_margin_breach,
               "truth_dynamic_safe": true_body_gap >= .05 and true_padded_gap > 0,
               "predicted_dynamic_safe": pred_gap >= .02,
               "costcritic_collision": bool(static_mask[i]),
               "prediction_score_captured": float(captured_pred_score[i]),
               "prediction_score_recomputed": calculated_score,
               "total_score": float(total[i]), "weighted_score": float(weighted[i]),
               "mppi_probability": float(weights[i])}
        rec.update({"critic_" + name: float(term[i]) for name, term in terms.items()})
        records.append(rec)
    rank_mode = params.get("collision_rank_mode", "legacy")
    if rank_mode == "uniform_center_overlap":
        # Import here to avoid a module cycle: the independent geometry probe
        # reuses this trace reader. Only hard-envelope collisions get the
        # additional continuous rank; noncolliding rollouts retain V1 scores.
        from occupancy_rank_probe import expected_overlap
        overlap = expected_overlap(meta, arrays, params)
        for rec, fraction in zip(records, overlap):
            if rec["first_predicted_conflict_s"] is not None:
                rec["prediction_score_recomputed"] += (
                    (3.81 / 254.) * 1_000_000. * float(fraction) / horizon_steps)
            rec["uniform_center_overlap_fraction"] = float(fraction)
    elif rank_mode != "legacy":
        raise ValueError(f"unsupported rank mode: {rank_mode}")
    score_errors = [abs(r["prediction_score_captured"] - r["prediction_score_recomputed"])
                    for r in records]
    if max(score_errors) > .05:
        raise ValueError(f"independent V1 score differs from captured term: max {max(score_errors)}")
    safe = [r for r in records if r["truth_dynamic_safe"] and not r["costcritic_collision"]]
    order = sorted(records, key=lambda r: r["total_score"])
    summary = {
        "schema": "rm_dynamic_prediction_frozen_cycle_analysis/v1",
        "source": {"cycle_json": str(cycle_path), "cycle_sha256": sha(cycle_path),
                   "binary_sha256": sha(Path(cycle_path).with_suffix(".bin")),
                   "profile_sha256": sha(profile_path), "gazebo_truth_sha256": sha(truth_path)},
        "cycle_id": meta["cycle_id"], "consumer_sim_s": consumed_t,
        "prediction_source_age_s": age, "batch_size": count, "steps": steps,
        "prediction_steps": horizon_steps, "safe_truth_and_costmap_rollouts": len(safe),
        "predicted_margin_safe_rollouts": sum(r["predicted_dynamic_safe"] for r in records),
        "predicted_collision_rollouts": sum(r["first_predicted_conflict_s"] is not None for r in records),
        "prediction_truth_uncovered_times_s": uncovered_times,
        "best_truth_body_clearance_m": max(r["truth_body_min_clearance_m"] for r in records),
        "safe_probability_mass": float(sum(weights[r["rollout"]] for r in safe)),
        "prediction_score_span": float(np.ptp(captured_pred_score)),
        "sampled_dynamics_replay_max_abs_error_m_or_rad": replay_error,
        "first_step_sampled_state_max_spread_m_or_rad_or_mps": first_spread,
        "first_step_predicted_padded_gap_m": first_pred_gap,
        "first_step_truth_body_gap_m": first_truth_body_gap,
        "all_batches_prediction_collision_invariant_at_first_step":
            first_spread < 1e-6 and first_pred_gap <= 1e-9,
        "optimized_filtered_sequence_truth_body_min_gap_m": min(optimized_truth_gaps),
        "optimized_filtered_sequence_truth_body_min_gap_step":
            optimized_truth_gaps.index(min(optimized_truth_gaps)) + 1,
        "best_safe_total_rank": next((rank for rank, r in enumerate(order, 1) if r in safe), None),
        "minimum_total_score_rollout": order[0]["rollout"],
        "maximum_probability_rollout": int(np.argmax(weights)),
        "returned_control": output,
        "full_compute_cycle_ms": (meta["finish_steady_ns"] - meta["request_steady_ns"]) / 1e6,
        "observer_copy_ms": meta["observer_copy_ns"] / 1e6,
        "prediction_score_max_abs_error": max(score_errors),
        "limits": "Sampled planar geometry; future Gazebo motion is an offline oracle, not online input. Instrumentation alters timing. The optimized filtered sequence is open-loop intent, not the actual later closed-loop path."
    }
    return summary, records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cycle_json", type=Path)
    parser.add_argument("profile_yaml", type=Path)
    parser.add_argument("gazebo_poses_jsonl", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise SystemExit("output directory already exists; refusing to overwrite evidence")
    summary, records = analyze(args.cycle_json, args.profile_yaml, args.gazebo_poses_jsonl)
    args.output_dir.mkdir(parents=True)
    with (args.output_dir / "rollouts.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    with (args.output_dir / "summary.json").open("x") as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
