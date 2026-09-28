#!/usr/bin/env python3
"""Offline truth-oracle audit of safe MPPI samples and their filtered mixtures.

Truth selects samples only in this diagnostic. It never enters a runtime critic.
The native costmap checker must evaluate the exported filtered sequences before
the `finalize` command reports joint safety.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry


def filtered_poses(sequence, meta, settings):
    return analyze.integrate_omni(
        *(sequence[:, axis] for axis in range(3)), meta["pose"], settings["dt"])


def one_hot(controls, index, settings, history):
    sequence = controls[index].copy()
    limits = ((settings["vx_min"], settings["vx_max"]),
              (-settings["vy_max"], settings["vy_max"]),
              (-settings["wz_max"], settings["wz_max"]))
    for axis, (lower, upper) in enumerate(limits):
        sequence[:, axis] = np.clip(sequence[:, axis], lower, upper)
    import replay_ranking
    return np.stack([replay_ranking.smooth_axis(
        sequence[:, axis], history[:, axis]) for axis in range(3)], axis=-1)


def prepare(cycle, profile_path, truth_path, history_path, score_root,
            mask_dir, output_dir):
    if output_dir.exists():
        raise FileExistsError(output_dir)
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"],
            settings["iterations"]) != (162, 300, 30, 1):
        raise ValueError("oracle fixture expects frozen cycle 162")
    history_payload = json.loads(history_path.read_text())
    if history_payload["cycle_id"] != meta["cycle_id"]:
        raise ValueError("control history does not match cycle")
    history = np.asarray(history_payload["previous_outputs"], dtype=np.float32)
    if history.shape != (4, 3):
        raise ValueError("invalid control history")
    initial = np.stack([analyze.last(arrays, "initial." + name)
                        for name in AXES], axis=-1)
    body = yaml.safe_load(yaml.safe_load(profile_path.read_text())[
        "local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
    padded = meta["padded_footprint"]
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
                             analyze.interpolated_pose(
                                 truth, times, consumed + (step + 1) *
                                 settings["dt"]))
              for step in range(settings["steps"])]
    raw = analyze.last(arrays, "locked.raw_map")
    threshold = shortcut_threshold(meta, profile_path)
    rows, poses, groups = [], [], []

    def add_row(label, sequence, extra):
        geometry = open_loop_geometry(sequence, meta, settings, actual,
                                      body, padded)
        row = {"row": len(rows), "label": label, **extra, "geometry": geometry}
        rows.append(row)
        poses.append(filtered_poses(sequence, meta, settings))
        return row["row"]

    def process_sample_set(label, controls, raw_trajectory, static_mask,
                           score_files, batches, probability_override=None):
        body_gaps, padded_gaps = geometry_labels(raw_trajectory, body,
                                                  padded, actual)
        raw_safe = (body_gaps >= .05) & (padded_gaps > 0) & ~static_mask
        individual_rows = {}
        for index in np.flatnonzero(raw_safe):
            individual_rows[int(index)] = add_row(
                "one_hot_filtered", one_hot(controls, index, settings, history),
                {"sample_set": label, "sample_index": int(index),
                 "raw_body_min_gap_m": float(body_gaps[index]),
                 "raw_padded_min_gap_m": float(padded_gaps[index])})
        for batch, score_path in zip(batches, score_files):
            scores = np.fromfile(score_path, dtype="<f4") if label != "captured" \
                else analyze.last(arrays, "scored.costs")
            if scores.shape != (batch,):
                raise ValueError(f"score count mismatch: {score_path}")
            selected = np.flatnonzero(raw_safe[:batch])
            if not len(selected):
                raise ValueError(f"no raw joint-safe samples: {label}/{batch}")
            baseline = aggregate(scores, controls[:batch], initial, settings,
                                 history, probability_override)
            probability = baseline["probability"]
            variants = {"actual_softmax": baseline}
            for mode in ("conditional_softmax", "uniform_safe"):
                weights = np.zeros(batch, dtype=np.float32)
                if mode == "conditional_softmax":
                    weights[selected] = probability[selected] / np.sum(
                        probability[selected], dtype=np.float32)
                else:
                    weights[selected] = np.float32(1 / len(selected))
                variants[mode] = aggregate(scores, controls[:batch], initial,
                                           settings, history, weights)
            aggregate_rows = {mode: add_row(
                mode, result["filtered_sequence"],
                {"sample_set": label, "batch": batch,
                 "returned_control_mps_radps": result["returned_control"]})
                for mode, result in variants.items()}
            groups.append({
                "sample_set": label, "batch": batch,
                "score_sha256": digest(score_path) if label != "captured" else None,
                "raw_joint_safe_count": int(len(selected)),
                "raw_best_body_gap_m": float(np.max(body_gaps[selected])),
                "raw_safe_probability_mass": float(np.sum(
                    probability[selected], dtype=np.float32)),
                "one_hot_rows": [individual_rows[int(index)] for index in selected],
                "aggregate_rows": aggregate_rows,
            })

    captured_controls = np.stack([analyze.last(arrays, "sampled.c" + name)
                                  for name in AXES], axis=-1)
    captured_trajectory = tuple(analyze.last(arrays, "rollout." + axis)
                                for axis in ("x", "y", "yaw"))
    captured_mask = np.loadtxt(mask_dir / "captured_mask.txt", dtype=bool)
    if captured_mask.shape != (300,):
        raise ValueError("captured native mask changed")
    process_sample_set("captured", captured_controls, captured_trajectory,
                       captured_mask, [None], [300],
                       analyze.last(arrays, "weighted.probability"))
    actual_row = rows[groups[0]["aggregate_rows"]["actual_softmax"]]
    recorded = np.asarray(analyze.event(meta, "output"))
    replay_control_error = max(abs(np.asarray(
        actual_row["returned_control_mps_radps"]) - recorded))
    replay_gap_error = abs(actual_row["geometry"]["body_min_gap_m"] -
                           .009435515466096334)
    if replay_control_error > 2e-5 or replay_gap_error > 1e-6:
        raise ValueError("captured aggregation did not replay: "
                         f"control={replay_control_error}, gap={replay_gap_error}")
    for seed in range(4):
        control_axes, trajectory = sample_omni(meta, arrays, 2000, seed)
        controls = np.stack([control_axes[name] for name in AXES], axis=-1)
        static_mask = np.loadtxt(mask_dir / f"seed_{seed}_mask.txt", dtype=bool)
        if static_mask.shape != (2000,):
            raise ValueError(f"native mask changed: seed {seed}")
        score_files = [score_root / f"seed{seed}_batch{batch}" /
                       "native_scores.bin" for batch in BATCHES]
        process_sample_set(f"seed_{seed}", controls, trajectory, static_mask,
                           score_files, BATCHES)
    output_dir.mkdir(parents=True)
    stacked_poses = tuple(np.stack([item[axis] for item in poses])
                          for axis in range(3))
    export(output_dir / "filtered_sequences.bin", meta, raw,
           stacked_poses, threshold)
    manifest = {
        "schema": "rm_dynamic_prediction_oracle_aggregation/v1",
        "scope": "Offline truth-oracle diagnostic only. Truth selects raw joint-safe samples; filtered control is integrated as a new open-loop sequence. No runtime critic or deployable policy is implied. New-batch native scores remain exploratory because captured cycle 162 has 22 PathAlign residuals.",
        "input_sha256": {
            "cycle_json": digest(cycle),
            "cycle_bin": digest(cycle.with_suffix(".bin")),
            "profile": digest(profile_path), "truth": digest(truth_path),
            "history": digest(history_path),
            "captured_static_mask": digest(mask_dir / "captured_mask.txt"),
            **{f"seed_{seed}_static_mask": digest(
                mask_dir / f"seed_{seed}_mask.txt") for seed in range(4)},
        },
        "fixture_sha256": digest(output_dir / "filtered_sequences.bin"),
        "possibly_inscribed_cost_threshold": threshold,
        "rows": rows, "groups": groups,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return len(rows), len(groups)


def finalize(output_dir, native_mask, native_checker, summary_path):
    if summary_path.exists():
        raise FileExistsError(summary_path)
    manifest_path = output_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if digest(output_dir / "filtered_sequences.bin") != manifest["fixture_sha256"]:
        raise ValueError("fixture binary hash differs from manifest")
    mask = np.loadtxt(native_mask, dtype=bool)
    rows = manifest["rows"]
    if mask.shape != (len(rows),):
        raise ValueError("native filtered mask count differs")

    def observed(index):
        row = rows[index]
        if row["row"] != index:
            raise ValueError("manifest row order differs")
        return {**row["geometry"], "costcritic_collision": bool(mask[index]),
                "joint_clearance_gate_met": bool(
                    row["geometry"]["dynamic_clearance_gate_met"] and
                    not mask[index]),
                **({"returned_control_mps_radps":
                    row["returned_control_mps_radps"]}
                   if "returned_control_mps_radps" in row else {})}

    groups = []
    for group in manifest["groups"]:
        one_hot = [(index, observed(index)) for index in group["one_hot_rows"]]
        safe = [(index, state) for index, state in one_hot
                if state["joint_clearance_gate_met"]]
        best = max(safe, key=lambda entry: entry[1]["body_min_gap_m"]) \
            if safe else None
        groups.append({
            "sample_set": group["sample_set"], "batch": group["batch"],
            "score_sha256": group["score_sha256"],
            "raw_joint_safe_count": group["raw_joint_safe_count"],
            "raw_best_body_gap_m": group["raw_best_body_gap_m"],
            "raw_safe_probability_mass": group["raw_safe_probability_mass"],
            "one_hot_filtered_dynamic_safe_count": sum(
                state["dynamic_clearance_gate_met"] for _, state in one_hot),
            "one_hot_filtered_joint_safe_count": len(safe),
            "best_one_hot_joint_safe": {"sample_index": rows[best[0]][
                "sample_index"], **best[1]} if best else None,
            "aggregates": {name: observed(index) for name, index in
                           group["aggregate_rows"].items()},
        })
    output = {
        "schema": "rm_dynamic_prediction_oracle_aggregation_result/v1",
        "scope": manifest["scope"],
        "input_sha256": manifest["input_sha256"],
        "fixture_sha256": manifest["fixture_sha256"],
        "manifest_sha256": digest(manifest_path),
        "native_mask_sha256": digest(native_mask),
        "native_checker_sha256": digest(native_checker),
        "filtered_costcritic_collision_count": int(np.sum(mask)),
        "groups": groups,
    }
    summary_path.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    return len(groups)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prepare_args = sub.add_parser("prepare")
    for flag in ("cycle", "profile", "truth", "history", "score-root",
                 "mask-dir", "output-dir"):
        prepare_args.add_argument("--" + flag, type=Path, required=True)
    final_args = sub.add_parser("finalize")
    for flag in ("output-dir", "native-mask", "native-checker", "summary"):
        final_args.add_argument("--" + flag, type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        rows, groups = prepare(args.cycle, args.profile, args.truth,
                               args.history, args.score_root, args.mask_dir,
                               args.output_dir)
        print(json.dumps({"rows": rows, "groups": groups}))
    else:
        groups = finalize(args.output_dir, args.native_mask,
                          args.native_checker, args.summary)
        print(json.dumps({"groups": groups, "summary": str(args.summary)}))
