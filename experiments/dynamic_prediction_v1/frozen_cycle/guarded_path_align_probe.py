#!/usr/bin/env python3
"""Replace only undefined PathAlign end lookups in an isolated frozen replay.

The installed six other critics are scored natively with PathAlign omitted.
This Python reference applies the installed PathAlign arithmetic with one
defined boundary: lower_bound(end) selects the last integrated path point.
It is a research replay, not a runtime Nav2 patch or deployment candidate.
"""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import BATCHES, digest, geometry_labels, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry
from path_align_bound_audit import path_exposure


def input_cases(captured_dir, score_root):
    yield "captured", 300, captured_dir
    for seed in range(4):
        for batch in BATCHES:
            yield f"seed_{seed}", batch, score_root / f"seed{seed}_batch{batch}"


def prepare(captured_dir, score_root, output_dir):
    if output_dir.exists():
        raise FileExistsError(output_dir)
    output_dir.mkdir(parents=True)
    cases = []
    for label, batch, source in input_cases(captured_dir, score_root):
        meta_path = source / "meta.json"
        payload = json.loads(meta_path.read_text())
        critics = payload["parameters"]["FollowPath.critics"]
        if critics.count("PathAlignCritic") != 1 or payload["batch"] != batch:
            raise ValueError(f"PathAlign is not present once: {meta_path}")
        payload["parameters"]["FollowPath.critics"] = [
            critic for critic in critics if critic != "PathAlignCritic"]
        name = f"{label}_batch{batch}"
        case_dir = output_dir / name
        case_dir.mkdir()
        (case_dir / "meta_without_align.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n")
        cases.append({"name": name, "label": label, "batch": batch,
                      "source_dir": str(source.resolve()),
                      "map_file": payload["map_data_file"],
                      "meta_sha256": digest(meta_path),
                      "without_align_meta_sha256": digest(
                          case_dir / "meta_without_align.json"),
                      "map_sha256": digest(source / payload["map_data_file"]),
                      "controls_sha256": digest(source / "controls.bin")})
    (output_dir / "inputs.json").write_text(json.dumps({
        "schema": "rm_dynamic_prediction_guarded_path_align_inputs/v1",
        "cases": cases}, indent=2, sort_keys=True) + "\n")
    return cases


def run_native(output_dir, binary, host_root, mount_root):
    """Run the installed six-critic scorer serially inside the frozen image."""
    cases = json.loads((output_dir / "inputs.json").read_text())["cases"]
    for case in cases:
        source = mount_root / Path(case["source_dir"]).relative_to(host_root)
        target = output_dir / case["name"]
        output = target / "native_without_align.bin"
        if output.exists():
            raise FileExistsError(output)
        result = subprocess.run([
            str(binary), str(target / "meta_without_align.json"),
            str(source / case["map_file"]), str(source / "controls.bin"),
            str(output)], capture_output=True, text=True, check=False)
        (target / "native_log.txt").write_text(
            result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f"native scoring failed for {case['name']}: "
                               f"{result.stderr[-1000:]}")
    return cases


def guarded_align_scores(path, trajectories, weight, stride, raw_map, map_meta):
    """Mirror active PathAlign for this frozen map, guarding only the end."""
    path = np.asarray(path, dtype=np.float32)[:, :2]
    trajectories = np.asarray(trajectories, dtype=np.float32)
    furthest, limit, _, first_exposed = path_exposure(
        path, trajectories, stride)
    origin_x, origin_y = map_meta["origin"]
    resolution = map_meta["resolution"]
    for x, y in path[:furthest]:
        mx = int((float(x) - origin_x) / resolution)
        my = int((float(y) - origin_y) / resolution)
        if (mx < 0 or my < 0 or mx >= map_meta["width"] or
                my >= map_meta["height"] or
                int(raw_map[my, mx]) in (253, 254, 255)):
            raise ValueError("frozen path has invalid costmap points")
    integrated = np.zeros(furthest, dtype=np.float32)
    for index in range(1, furthest):
        delta = path[index] - path[index - 1]
        integrated[index] = np.float32(integrated[index - 1] + np.sqrt(
            np.float32(delta[0] * delta[0] + delta[1] * delta[1])))
    if integrated[-1] != limit:
        raise ValueError("integrated path disagrees with exposure audit")
    count, steps, _ = trajectories.shape
    scores = np.zeros(count, dtype=np.float32)
    for row in range(count):
        traveled = np.float32(0)
        total_distance = np.float32(0)
        point = 0
        samples = 0
        for step in range(stride, steps, stride):
            delta = trajectories[row, step] - trajectories[row, step - stride]
            traveled = np.float32(traveled + np.sqrt(np.float32(
                delta[0] * delta[0] + delta[1] * delta[1])))
            position = int(np.searchsorted(
                integrated[point:], traveled, side="left") + point)
            if position == point:
                point = 0  # Matches installed findClosestPathPt first branch.
            elif position == furthest:
                point = furthest - 1  # Defined replacement for *end.
            elif traveled - integrated[position - 1] < \
                    integrated[position] - traveled:
                point = position - 1
            else:
                point = position
            delta = path[point] - trajectories[row, step]
            total_distance = np.float32(total_distance + np.sqrt(np.float32(
                delta[0] * delta[0] + delta[1] * delta[1])))
            samples += 1
        scores[row] = np.float32(np.float32(total_distance /
                                      np.float32(samples)) * np.float32(weight))
    return scores, first_exposed, furthest


def score_and_aggregate(cycle, profile_path, truth_path, history_path,
                        captured_dir, score_root, mask_dir, output_dir):
    manifest = json.loads((output_dir / "inputs.json").read_text())
    meta, arrays = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    profile = yaml.safe_load(profile_path.read_text())
    align = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PathAlignCritic"]
    if (align["cost_power"] != 1 or align["trajectory_point_step"] != 4 or
            align["offset_from_furthest"] != 20 or
            align["use_path_orientations"] or settings["iterations"] != 1):
        raise ValueError("frozen PathAlign configuration changed")
    history_payload = json.loads(history_path.read_text())
    if history_payload["cycle_id"] != meta["cycle_id"]:
        raise ValueError("history belongs to another cycle")
    history = np.asarray(history_payload["previous_outputs"], dtype=np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    raw = analyze.last(arrays, "locked.raw_map")
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    body = yaml.safe_load(local["footprint"])
    padded = meta["padded_footprint"]
    truth = analyze.rows_from_transport(truth_path)
    times = [row["t"] for row in truth]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
                             analyze.interpolated_pose(truth, times,
                                 consumed + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    aggregate_poses = []
    rows = []
    for case in manifest["cases"]:
        label, batch, name = case["label"], case["batch"], case["name"]
        source = Path(case["source_dir"])
        case_dir = output_dir / name
        for path, key in ((source / "meta.json", "meta_sha256"),
                          (case_dir / "meta_without_align.json",
                           "without_align_meta_sha256"),
                          (source / case["map_file"], "map_sha256"),
                          (source / "controls.bin", "controls_sha256")):
            if digest(path) != case[key]:
                raise ValueError(f"fixture input changed: {path}")
        other_path = case_dir / "native_without_align.bin"
        other = np.fromfile(other_path, dtype="<f4")
        if other.shape != (batch,):
            raise ValueError(f"missing native six-critic scores: {other_path}")
        if label == "captured":
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
            full_trajectory = tuple(analyze.last(arrays, "rollout." + axis)
                                    for axis in ("x", "y", "yaw"))
            trajectory = np.stack(full_trajectory[:2], axis=-1)
            static_mask = np.loadtxt(mask_dir / "captured_mask.txt", dtype=bool)
            full_path = score_root / "captured_native_scores.bin"
            original_pre_v1 = analyze.last(
                arrays, "critic.FollowPath.PathAngleCritic")
        else:
            seed = int(label.removeprefix("seed_"))
            axis_controls, sampled = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([axis_controls[axis][:batch]
                                 for axis in AXES], axis=-1)
            full_trajectory = tuple(axis[:batch] for axis in sampled)
            trajectory = np.stack(full_trajectory[:2], axis=-1)
            static_mask = np.loadtxt(
                mask_dir / f"seed_{seed}_mask.txt", dtype=bool)[:batch]
            full_path = source / "native_scores.bin"
            original_pre_v1 = None
        full = np.fromfile(full_path, dtype="<f4")
        if full.shape != (batch,):
            raise ValueError(f"native seven-critic score count differs: {full_path}")
        if static_mask.shape != (batch,):
            raise ValueError(f"native static mask count differs: {name}")
        body_gaps, padded_gaps = geometry_labels(
            full_trajectory, body, padded, actual)
        joint_safe = (body_gaps >= .05) & (padded_gaps > 0) & ~static_mask
        aligned, first_exposed, furthest = guarded_align_scores(
            meta["path"], trajectory, align["cost_weight"],
            align["trajectory_point_step"], raw, meta["map"])
        corrected = other + aligned
        exposed = first_exposed >= 0
        if not np.any(~exposed):
            raise ValueError("no in-bounds rows for native parity")
        nonexposed_error = float(np.max(np.abs(
            corrected[~exposed] - full[~exposed])))
        if nonexposed_error > 1e-3:
            raise ValueError(f"six-critic and guarded score disagree in bounds: {name}: {nonexposed_error}")
        (case_dir / "guarded_native_scores.bin").write_bytes(
            corrected.astype("<f4").tobytes())
        baseline = aggregate(full, controls, initial, settings, history)
        result = aggregate(corrected, controls, initial, settings, history)
        old_rank = np.argsort(baseline["weighted"])
        new_rank = np.argsort(result["weighted"])
        baseline_geometry = open_loop_geometry(
            baseline["filtered_sequence"], meta, settings, actual, body, padded)
        corrected_geometry = open_loop_geometry(
            result["filtered_sequence"], meta, settings, actual, body, padded)
        pose = analyze.integrate_omni(
            *(result["filtered_sequence"][:, axis] for axis in range(3)),
            meta["pose"], settings["dt"])
        aggregate_poses.append(pose)
        row = {
            "name": name, "label": label, "batch": batch,
            "furthest_reached_path_point": furthest,
            "end_iterator_exposure_count": int(np.sum(exposed)),
            "in_bounds_native_score_max_abs_error": nonexposed_error,
            "native_score_changed_gt_1e_3_count": int(np.sum(
                np.abs(corrected - full) > 1e-3)),
            "native_score_change_max_abs": float(np.max(
                np.abs(corrected - full))),
            "native_full_score_sha256": digest(full_path),
            "native_six_score_sha256": digest(other_path),
            "guarded_score_sha256": digest(
                case_dir / "guarded_native_scores.bin"),
            "old_min_score_index": int(np.argmin(baseline["weighted"])),
            "guarded_min_score_index": int(np.argmin(result["weighted"])),
            "raw_joint_safe_count": int(np.sum(joint_safe)),
            "old_raw_joint_safe_probability_mass": float(np.sum(
                baseline["probability"][joint_safe], dtype=np.float32)),
            "guarded_raw_joint_safe_probability_mass": float(np.sum(
                result["probability"][joint_safe], dtype=np.float32)),
            "old_top_10_raw_joint_safe_count": int(np.sum(
                joint_safe[old_rank[:10]])),
            "guarded_top_10_raw_joint_safe_count": int(np.sum(
                joint_safe[new_rank[:10]])),
            "old_returned_control": baseline["returned_control"],
            "guarded_returned_control": result["returned_control"],
            "old_filtered_geometry": baseline_geometry,
            "guarded_filtered_geometry": corrected_geometry,
        }
        if original_pre_v1 is not None:
            row["captured_in_bounds_pre_v1_max_abs_error"] = float(np.max(
                np.abs(corrected[~exposed] - original_pre_v1[~exposed])))
        rows.append(row)
    fixture = output_dir / "guarded_filtered_aggregates.bin"
    export(fixture, meta, raw,
           tuple(np.stack([pose[axis] for pose in aggregate_poses])
                 for axis in range(3)), shortcut_threshold(meta, profile_path))
    result = {
        "schema": "rm_dynamic_prediction_guarded_path_align_scores/v1",
        "scope": "Isolated frozen replay: installed six other standard critics, plus a Python PathAlign arithmetic mirror that clamps lower_bound(end) to the last integrated path point. V1 is an all-sample constant first-step collision in this cycle and omitted from ranking. This is not a runtime fix. New batch open-loop geometry uses future truth for labels only.",
        "input_sha256": {"cycle_json": digest(cycle),
                         "cycle_bin": digest(cycle.with_suffix(".bin")),
                         "profile": digest(profile_path),
                         "truth": digest(truth_path),
                         "history": digest(history_path),
                         "native_inputs_manifest": digest(output_dir / "inputs.json")},
        "fixture_sha256": digest(fixture), "rows": rows,
    }
    (output_dir / "pre_static_summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    return rows


def finalize(output_dir, native_mask, checker, summary):
    if summary.exists():
        raise FileExistsError(summary)
    result = json.loads((output_dir / "pre_static_summary.json").read_text())
    if digest(output_dir / "guarded_filtered_aggregates.bin") != result[
            "fixture_sha256"]:
        raise ValueError("aggregate fixture changed")
    mask = np.loadtxt(native_mask, dtype=bool)
    if mask.shape != (len(result["rows"]),):
        raise ValueError("native mask length differs")
    for row, collision in zip(result["rows"], mask):
        geometry = row["guarded_filtered_geometry"]
        geometry["costcritic_collision"] = bool(collision)
        geometry["joint_clearance_gate_met"] = bool(
            geometry["dynamic_clearance_gate_met"] and not collision)
    result["native_mask_sha256"] = digest(native_mask)
    result["native_checker_sha256"] = digest(checker)
    result["guarded_aggregate_costcritic_collision_count"] = int(np.sum(mask))
    summary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    return len(result["rows"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    prep.add_argument("--captured-dir", type=Path, required=True)
    prep.add_argument("--score-root", type=Path, required=True)
    prep.add_argument("--output-dir", type=Path, required=True)
    native = sub.add_parser("native")
    for flag in ("output-dir", "binary", "host-root", "mount-root"):
        native.add_argument("--" + flag, type=Path, required=True)
    score = sub.add_parser("score")
    for flag in ("cycle", "profile", "truth", "history", "captured-dir",
                 "score-root", "mask-dir", "output-dir"):
        score.add_argument("--" + flag, type=Path, required=True)
    finish = sub.add_parser("finalize")
    for flag in ("output-dir", "native-mask", "checker", "summary"):
        finish.add_argument("--" + flag, type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        rows = prepare(args.captured_dir, args.score_root, args.output_dir)
    elif args.command == "native":
        rows = run_native(args.output_dir, args.binary, args.host_root,
                          args.mount_root)
    elif args.command == "score":
        rows = score_and_aggregate(args.cycle, args.profile, args.truth,
                                   args.history, args.captured_dir,
                                   args.score_root, args.mask_dir,
                                   args.output_dir)
    else:
        rows = finalize(args.output_dir, args.native_mask, args.checker,
                        args.summary)
    print(json.dumps({"rows": len(rows) if isinstance(rows, list) else rows}))
