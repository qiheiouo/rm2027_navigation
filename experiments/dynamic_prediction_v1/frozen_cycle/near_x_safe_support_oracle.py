#!/usr/bin/env python3
"""Offline perfect-safe-support ceiling on two fixed near-X MPPI cycles."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_sensitivity import aggregate
from near_x_batch_sensitivity import AXES, frozen as north_frozen
from near_x_scale4_score import controls_for
from near_x_south_batch import frozen as south_frozen, physical as south_physical
from phase2_filtered_audit import candidates
import replay_ranking


IMAGE_ID = "sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6"
CHECKER_SHA = "37671b08cd8248f336703d79da4b5dd73fd7677999d24706f4955b7b6f6c059a"
CASES = {
    "north": {"trial": "dynamic_prediction_near_x_crossing_20260930",
              "cycle_sha": "5b9608928eb729a91256404770ca20a4143f4661ec4ec1938293a772e4283491",
              "summary_sha": "fac24214f22ea0962a0736df153ae3c19a746edfe9a687ff56a8e15985a18a94",
              "summary": "scale4_score_result_73/summary.json",
              "scores": "scale4_native_scores_73",
              "noise": "noise_scale_inputs_73", "masks": "noise_scale_masks_73",
              "score_sha_key": "native_scores_sha256",
              "mask_sha_key": "filtered_static_mask_sha256"},
    "south": {"trial": "dynamic_prediction_near_x_south_crossing_20260930",
              "cycle_sha": "cf62847bc075612b793594cbe6c18c9b33db11a69f051aa68e02a551d83f68c1",
              "summary_sha": "fb40812c9dee25edd6601cd6453e1f174bcbb1ed6dd0f39ac693dc1e30db5bac",
              "summary": "guarded_score_result_65/summary.json",
              "scores": "guarded_scores_65_verified",
              "noise": "noise_inputs_65", "masks": "noise_masks_65",
              "score_sha_key": "guarded_scores_sha256",
              "mask_sha_key": "static_mask_sha256"}}


def north_physical(meta, settings, truth_path):
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    if times[-1] - consumed < 3.0:
        raise ValueError("north physical future shorter than 3 s")
    return [analyze.placed(analyze.obstacle_polygon(),
            analyze.interpolated_pose(truth, times,
                                      consumed + (j + 1) * settings["dt"]))
            for j in range(settings["steps"])]


def case_input(root, side):
    config = CASES[side]
    trial = root / "build" / config["trial"] / "candidate_navfn_1"
    if side == "north":
        paths, meta, arrays, settings = north_frozen(trial)
        cycle, profile_path, truth_path = (paths[key] for key in
                                           ("cycle", "profile", "truth"))
        previous = replay_ranking.history_from_trial(
            trial / "mppi_cycles", 73, arrays)
        history = np.stack([previous[key] for key in AXES], axis=-1)
        physical = north_physical(meta, settings, truth_path)
        hard = float(json.loads((trial / config["summary"]).read_text())[
            "rows"][0]["v1_hard_score"])
    else:
        cycle, meta, arrays, settings, profile_path, truth_path = south_frozen(trial)
        history = candidates(trial, meta, arrays, settings)[3]
        physical = south_physical(meta, settings, truth_path)
        v1 = analyze.critic_deltas(arrays)[0]["FollowPath.PredictionV1Critic"]
        if np.ptp(v1) > 1e-3:
            raise ValueError("south original V1 hard scores differ")
        hard = float(np.float32(np.mean(v1)))
    if digest(cycle) != config["cycle_sha"]:
        raise ValueError(side + " frozen cycle hash changed")
    summary_path = trial / config["summary"]
    if digest(summary_path) != config["summary_sha"]:
        raise ValueError(side + " prior scoring summary changed")
    summary = json.loads(summary_path.read_text())
    if len(summary["rows"]) != 4:
        raise ValueError("expected four fixed seed rows")
    profile = yaml.safe_load(profile_path.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    goal = json.loads((trial / "observation/summary.json").read_text())["goal"]
    return (trial, config, meta, arrays, settings, profile_path, physical,
            history.astype(np.float32), hard, summary, body, goal)


def output_geometry(result, meta, settings, body, physical, goal):
    sequence = result["filtered_sequence"]
    velocity = np.concatenate((np.asarray(meta["speed"], dtype=np.float32)[None, :],
                               sequence[:-1]), axis=0)
    pose = analyze.integrate_omni(*(velocity[:, axis] for axis in range(3)),
                                  meta["pose"], settings["dt"])
    body_gap, padded_gap = geometry_labels(
        tuple(axis[None, :] for axis in pose), body,
        meta["padded_footprint"], physical)
    initial_goal = float(np.hypot(meta["pose"][0] - goal[0],
                                  meta["pose"][1] - goal[1]))
    progress = initial_goal - float(np.hypot(pose[0][-1] - goal[0],
                                             pose[1][-1] - goal[1]))
    return pose, float(body_gap[0]), float(padded_gap[0]), progress


def prepare(root, checker, image_id, output):
    if output.exists():
        raise FileExistsError(output)
    if image_id != IMAGE_ID or digest(checker) != CHECKER_SHA:
        raise ValueError("runtime image or native static checker differs")
    output.mkdir(parents=True)
    all_rows = []
    maps = {}
    for side in CASES:
        (trial, config, meta, arrays, settings, profile_path, physical,
         history, hard, prior, body, goal) = case_input(root, side)
        initial = np.stack([analyze.last(arrays, "initial." + axis)
                            for axis in AXES], axis=-1).astype(np.float32)
        scaled = dict(settings)
        for axis in AXES:
            scaled[axis + "_std"] *= 4
        noise_record = json.loads((trial / config["noise"] / "inputs.json").read_text())
        case_poses = []
        for seed in range(4):
            prior_row = prior["rows"][seed]
            if prior_row["seed"] != seed or prior_row["batch"] != 2000:
                raise ValueError("prior row order changed")
            controls = controls_for(meta, arrays, seed)
            score_path = trial / config["scores"] / f"seed_{seed}_scores.bin"
            mask_path = trial / config["masks"] / f"seed_{seed}_scale_4_mask.txt"
            gaps_path = trial / config["noise"] / f"seed_{seed}_scale_4_gaps.npz"
            if digest(score_path) != prior_row[config["score_sha_key"]] or \
                    digest(mask_path) != prior_row[config["mask_sha_key"]] or \
                    digest(gaps_path) != noise_record["files_sha256"][gaps_path.name]:
                raise ValueError(side + " previous score, mask or gap fixture changed")
            native = np.fromfile(score_path, dtype="<f4")
            mask = np.loadtxt(mask_path, dtype=bool)
            with np.load(gaps_path) as gaps:
                body_gap, padded_gap = gaps["body"], gaps["padded"]
            if native.shape != (2000,) or mask.shape != native.shape or \
                    body_gap.shape != native.shape or padded_gap.shape != native.shape:
                raise ValueError("fixed candidate batch dimensions differ")
            safe = (body_gap >= .05) & (padded_gap > 0) & ~mask
            if int(safe.sum()) != prior_row["joint_safe_count"] or not safe.any():
                raise ValueError("prior safe count differs or empty")
            baseline = aggregate(native + np.float32(hard), controls,
                                 initial, scaled, history)
            baseline_error = float(np.max(np.abs(
                np.asarray(baseline["returned_control"]) -
                np.asarray(prior_row["returned_control"]))))
            if baseline_error > 1e-5:
                raise ValueError(side + " baseline control failed exact replay")
            oracle_scores = native + np.float32(hard) + \
                np.where(safe, np.float32(0), np.float32(1e6))
            support = aggregate(oracle_scores, controls, initial, scaled, history)
            if np.any(support["probability"][~safe] != 0) or \
                    abs(float(support["probability"][safe].sum()) - 1) > 1e-5:
                raise ValueError("unsafe candidate got oracle support")
            best = int(np.flatnonzero(safe)[np.argmin(baseline["weighted"][safe])])
            one_hot = np.zeros(2000, dtype=np.float32)
            one_hot[best] = 1
            selected = aggregate(native + np.float32(hard), controls,
                                 initial, scaled, history,
                                 probability_override=one_hot)
            variants = (("safe_support", support), ("best_safe", selected))
            for variant, result in variants:
                pose, out_body, out_padded, progress = output_geometry(
                    result, meta, settings, body, physical, goal)
                if variant == "best_safe" and (abs(out_body - body_gap[best]) > 1e-4 or
                                               abs(out_padded - padded_gap[best]) > 1e-4):
                    raise ValueError("one-hot output differs from selected safe label")
                case_poses.append(pose)
                all_rows.append({"side": side, "seed": seed, "variant": variant,
                                 "safe_count": int(safe.sum()),
                                 "best_safe_index": best,
                                 "score_sha256": digest(score_path),
                                 "mask_sha256": digest(mask_path),
                                 "gaps_sha256": digest(gaps_path),
                                 "baseline_control_max_abs_error": baseline_error,
                                 "oracle_safe_probability_mass": float(
                                     result["probability"][safe].sum()),
                                 "effective_sample_size": float(1. / np.sum(
                                     result["probability"] ** 2)),
                                 "returned_control": result["returned_control"],
                                 "body_min_gap_m": out_body,
                                 "padded_min_gap_m": out_padded,
                                 "dynamic_gate_pass": bool(out_body >= .05 and
                                                           out_padded > 0),
                                 "goal_progress_m": progress})
        map_path = output / f"{side}_aggregate_map.bin"
        export(map_path, meta, analyze.last(arrays, "locked.raw_map"),
               tuple(np.stack([pose[axis] for pose in case_poses])
                     for axis in range(3)),
               shortcut_threshold(meta, profile_path))
        maps[side] = digest(map_path)
    report = {"schema": "rm_dynamic_prediction_near_x_safe_support_oracle/v1",
              "scope": "Future truth and native raw-map labels restrict support; same existing C++ scores, MPPI regularizer and filter. Offline oracle only; two frozen states, four seeds each.",
              "image_id": image_id, "native_checker_sha256": digest(checker),
              "map_sha256": maps, "rows": all_rows}
    (output / "preliminary.json").write_text(json.dumps(
        report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"rows": len(all_rows), "output": str(output)}))


def finalize(output):
    report = json.loads((output / "preliminary.json").read_text())
    if (output / "summary.json").exists() or len(report["rows"]) != 16:
        raise ValueError("oracle output count differs or summary exists")
    for side in CASES:
        if digest(output / f"{side}_aggregate_map.bin") != report["map_sha256"][side]:
            raise ValueError("oracle aggregate map changed")
        mask_path = output / f"{side}_static_mask.txt"
        masks = np.loadtxt(mask_path, dtype=bool)
        if masks.shape != (8,):
            raise ValueError("native oracle static mask dimensions differ")
        for row, hit in zip([r for r in report["rows"] if r["side"] == side], masks):
            row["native_static_collision"] = bool(hit)
            row["joint_gate_pass"] = bool(row["dynamic_gate_pass"] and not hit)
        report.setdefault("static_mask_sha256", {})[side] = digest(mask_path)
    (output / "summary.json").write_text(json.dumps(report, indent=2,
                                              sort_keys=True) + "\n")
    print(json.dumps({"joint_gate_by_variant": {
        variant: [row["joint_gate_pass"] for row in report["rows"]
                  if row["variant"] == variant]
        for variant in ("safe_support", "best_safe")}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--root", type=Path, required=True)
    prep.add_argument("--native-checker", type=Path, required=True)
    prep.add_argument("--image-id", required=True)
    prep.add_argument("--output", type=Path, required=True)
    done = sub.add_parser("finalize")
    done.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.root, args.native_checker, args.image_id, args.output)
    else:
        finalize(args.output)
