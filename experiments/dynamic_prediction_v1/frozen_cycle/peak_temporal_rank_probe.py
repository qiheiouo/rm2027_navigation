#!/usr/bin/env python3
"""Compare V1 mean and peak temporal overlap on five frozen inputs.

The conservative hard envelope, near term, seven native standard critics,
temperature, controls and output filter remain fixed. Future Gazebo truth
labels rankings and completed outputs only after score aggregation.
"""
import argparse
import csv
import json
from pathlib import Path
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from costmap_mask_fixture import export, shortcut_threshold
from envelope import AxisBox
from filtered_batch_candidate_rank import raw_controls
from filtered_candidate_rank_audit import safe_auc
from filtered_graded_batch_probe import filtered_poses
from filtered_graded_cross_cycle import filtered_prediction_score
from native_critic_sensitivity import AXES, aggregate
from output_first_step_audit import actual, pose_array
import rank_probe
import replay_ranking


CASES = (("early_147", 147, "collision_trial", None, "score147"),
         ("intrusion_162", 162, "collision_trial", None, "score162"),
         ("goal_263", 263, "goal_trial", None, "score263"),
         ("seed_2_batch300", 162, "collision_trial", 2, "seed2_score"),
         ("seed_3_batch300", 162, "collision_trial", 3, "seed3_score"))
CONTROL_HEADER = struct.Struct("<3I")


def step_overlap(meta, poses, params):
    """Same five-point center integral as V1, retaining its per-step values."""
    prediction = analyze.event(meta, "prediction.input")
    tracks = [track for track in prediction["tracks"] if track["state"] == 2]
    if len(tracks) != 1:
        raise ValueError("one confirmed track required")
    track = tracks[0]
    settings = analyze.event(meta, "settings")
    steps = int(np.floor(params["horizon"] / settings["dt"] + 1e-9))
    if steps != 9 or poses.shape != (300, 30, 3):
        raise ValueError("not the frozen nine-step 300 rollout input")
    extent = np.array((params["object_width"], params["object_height"]))
    nodes, weights = np.polynomial.legendre.leggauss(5)
    weights /= 2.
    area = rank_probe.area(meta["padded_footprint"])
    overlap = np.empty((300, steps), dtype=np.float64)
    for step in range(steps):
        duration = prediction["source_age_s"] + (step + 1) * settings["dt"]
        center = np.asarray(track["xy"]) + np.asarray(track["vxy"]) * duration
        support = (np.asarray(track["size_xy"]) / 2 + extent / 2 +
                   .5 * params["reference_acceleration"] * duration ** 2)
        boxes = []
        for nx, wx in zip(nodes, weights):
            for ny, wy in zip(nodes, weights):
                xy = center + np.array((nx, ny)) * support
                boxes.append((AxisBox(xy[0] - extent[0] / 2,
                                      xy[1] - extent[1] / 2,
                                      xy[0] + extent[0] / 2,
                                      xy[1] + extent[1] / 2), wx * wy))
        for index in range(300):
            polygon = analyze.placed(meta["padded_footprint"],
                                     tuple(map(float, poses[index, step])))
            overlap[index, step] = sum(
                weight * rank_probe.overlap_area(polygon, box)
                for box, weight in boxes) / area
    return overlap


def safe_labels(path):
    with path.open(newline="") as source:
        rows = list(csv.DictReader(source))
    if len(rows) < 300:
        raise ValueError(f"label rows truncated: {path}")
    return np.asarray([row["joint_safe_3s"] == "1" for row in rows[:300]])


def reference_controls(name, evidence):
    if name in ("early_147", "goal_263"):
        source = evidence / "cross_cycle_first_step_audit_20260928" / name / "controls.bin"
        index, count = 1, 3
    else:
        source = evidence / "output_first_step_audit_20260928" / "output_controls.bin"
        index, count = (0 if name == "intrusion_162" else
                        (9 if name == "seed_2_batch300" else 13)), 17
    data = source.read_bytes()
    if CONTROL_HEADER.unpack_from(data) != (0x43545231, count, 30):
        raise ValueError(f"reference controls malformed: {source}")
    return (np.frombuffer(data, dtype="<f4", offset=CONTROL_HEADER.size)
            .reshape(count, 30, 3)[index], source)


def run(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    args.output_dir.mkdir(parents=True)
    raw = json.loads(args.raw_inputs.read_text())
    evidence = Path("docs/dynamic_navigation/evidence")
    unit = np.float32((3.81 / 254.) * 1_000_000. / 9)
    rows = []
    for name, cycle_id, trial_key, seed, score_key in CASES:
        trial, score_path = getattr(args, trial_key), getattr(args, score_key)
        cycle = trial / f"mppi_cycles/cycle_{cycle_id}.json"
        profile_path = trial / "profile.yaml"
        truth_path = trial / "gazebo_poses.jsonl"
        meta, arrays = analyze.read_cycle(cycle)
        settings = analyze.event(meta, "settings")
        if (meta["cycle_id"], settings["batch"], settings["steps"]) != (
                cycle_id, 300, 30):
            raise ValueError(f"frozen settings differ: {name}")
        profile = yaml.safe_load(profile_path.read_text())
        params = profile["controller_server"]["ros__parameters"][
            "FollowPath"]["PredictionV1Critic"]
        if seed is None:
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
            raw_controls_path = None
        else:
            source = next(case for case in raw["cases"] if case["name"] == name)
            raw_controls_path = Path(source["controls"])
            controls = raw_controls(raw_controls_path, 300)
        previous = replay_ranking.history_from_trial(
            trial / "mppi_cycles", cycle_id, arrays)
        history = np.stack([previous[axis] for axis in AXES], axis=-1)
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1)
        poses = filtered_poses(controls, meta, settings, history)
        original, hits, mean = filtered_prediction_score(meta, poses, params)
        by_step = step_overlap(meta, poses, params)
        mean_error = float(np.max(np.abs(by_step.mean(axis=1) - mean)))
        if mean_error > 1e-8:
            raise ValueError(f"V1 mean overlap differs: {name}, {mean_error}")
        if np.any((original >= 100.) & (original < 1000.)):
            raise ValueError(f"unexpected frozen V1 score scale: {name}")
        hard = original > 100.
        peak = by_step.max(axis=1)
        changed = original + np.float32(unit * (peak - mean) * hard)
        standard = np.fromfile(score_path, dtype="<f4")
        if standard.shape != (300,) or not np.isfinite(standard).all():
            raise ValueError(f"native standard score differs: {name}")
        baseline = aggregate(standard + original, controls, initial,
                             settings, history)
        candidate = aggregate(standard + changed, controls, initial,
                              settings, history)
        reference, reference_path = reference_controls(name, evidence)
        baseline_error = float(np.max(np.abs(
            baseline["filtered_sequence"] - reference)))
        if baseline_error > 3e-5:
            raise ValueError(f"published baseline differs: {name}, {baseline_error}")
        if seed is None:
            label_path = (evidence / "filtered_candidate_rank_20260928" /
                          f"{name}_detail.csv")
        else:
            label_path = (evidence / "filtered_batch_candidate_rank_20260928" /
                          f"seed_{seed}_batch2000_labels.csv")
        safe = safe_labels(label_path)
        source = args.output_dir / name
        source.mkdir()
        output_controls = np.asarray((baseline["filtered_sequence"],
                                      candidate["filtered_sequence"]), dtype="<f4")
        poses_output = pose_array(output_controls, meta, settings, True)
        poses_output.tofile(source / "output_poses.bin")
        (source / "output_controls.bin").write_bytes(
            CONTROL_HEADER.pack(0x43545231, 2, 30) + output_controls.tobytes())
        body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
            "ros__parameters"]["footprint"])
        boxes = actual(meta, settings, truth_path)
        gaps = np.asarray([[analyze.polygon_distance(
            analyze.placed(body, tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in poses_output])
        padded_gaps = np.asarray([[analyze.polygon_distance(
            analyze.placed(meta["padded_footprint"], tuple(map(float, pose))), box)
            for pose, box in zip(sequence, boxes)]
            for sequence in poses_output])
        static_fixture = source / "static.bin"
        export(static_fixture, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(poses_output[:, :, axis] for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        result = {"name": name, "cycle_id": cycle_id, "seed": seed,
                  "input_sha256": {
                      "cycle": digest(cycle),
                      "cycle_bin": digest(cycle.with_suffix(".bin")),
                      "profile": digest(profile_path),
                      "truth": digest(truth_path),
                      "standard": digest(score_path),
                      "raw_controls": digest(raw_controls_path) if seed is not None else None,
                      "reference_controls": digest(reference_path),
                      "labels": digest(label_path)},
                  "file_sha256": {path.name: digest(path) for path in (
                      source / "output_poses.bin", source / "output_controls.bin",
                      static_fixture)},
                  "hard_collision_rollouts": int(hard.sum()),
                  "hard_hits_by_step": hits,
                  "overlap_mean_replay_max_abs_error": mean_error,
                  "baseline_control_max_abs_error": baseline_error,
                  "joint_safe_filtered_candidates": int(safe.sum()),
                  "outputs": []}
        if cycle_id == 263:
            goal = json.loads((trial / "runtime_audit.json").read_text())[
                "navigation_result"]["goal"]
        for label, v1, ranked, index in (("mean", original, baseline, 0),
                                          ("peak", changed, candidate, 1)):
            weight = ranked["probability"]
            item = {"name": label,
                    "v1_safe_auc": safe_auc(v1, safe),
                    "total_safe_auc": safe_auc(ranked["weighted"], safe),
                    "top_candidate_joint_safe": bool(safe[np.argmax(weight)]),
                    "safe_probability_mass": float(weight[safe].sum()),
                    "effective_sample_size": float(1. / np.sum(weight ** 2)),
                    "returned_control": ranked["returned_control"],
                    "body_min_gap_m": float(gaps[index].min()),
                    "body_min_gap_step": int(gaps[index].argmin() + 1),
                    "padded_min_gap_m": float(padded_gaps[index].min()),
                    "body_min_gap_first_4_m": float(gaps[index, :4].min()),
                    "body_min_gap_first_9_m": float(gaps[index, :9].min())}
            if cycle_id == 263:
                item["endpoint_goal_distance_m"] = float(np.hypot(
                    poses_output[index, -1, 0] - goal[0],
                    poses_output[index, -1, 1] - goal[1]))
            result["outputs"].append(item)
        rows.append(result)
    report = {"schema": "rm_dynamic_prediction/peak_temporal_rank_probe/v1",
              "scope": "Five preselected frozen 300-rollout inputs. Preserve hard V1 envelope and near score, candidate filtering, all seven native standard critics, MPPI regularizer/temperature, controls, final filter and original safety thresholds. Replace mean continuous physical-center overlap across nine steps with maximum of the same nine values. Gazebo future truth only labels rankings and completed outputs. Python native-first integrator previously checked against Nav2 C++; static mask pending.",
              "raw_inputs_sha256": digest(args.raw_inputs),
              "cases": rows}
    (args.output_dir / "prepared.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


def finalize(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    report = json.loads(args.prepared.read_text())
    for row in report["cases"]:
        name = row["name"]
        source = args.prepared.parent / name
        for filename, expected in row["file_sha256"].items():
            if digest(source / filename) != expected:
                raise ValueError(f"frozen output changed: {name}, {filename}")
        mask = args.mask_dir / f"{name}_mask.txt"
        values = [int(value) for value in mask.read_text().split()]
        if len(values) != 2 or any(value not in (0, 1) for value in values):
            raise ValueError(f"native static mask malformed: {name}")
        row["static_mask_sha256"] = digest(mask)
        for output, collision in zip(row["outputs"], values):
            output["costcritic_collision"] = bool(collision)
            output["joint_gate_met"] = bool(
                output["body_min_gap_m"] >= .05 and
                output["padded_min_gap_m"] > 0 and not collision)
    report["scope"] = report["scope"].replace(
        "static mask pending.", "original raw-costmap native static mask checked.")
    report["prepared_sha256"] = digest(args.prepared)
    report["native_mask_binary_sha256"] = digest(args.mask_binary)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(report["cases"])}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("collision-trial", "goal-trial", "raw-inputs", "score147",
                 "score162", "score263", "seed2-score", "seed3-score",
                 "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    done = sub.add_parser("finalize")
    for name in ("prepared", "mask-dir", "mask-binary", "output"):
        done.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": run, "finalize": finalize}[args.command](args)
