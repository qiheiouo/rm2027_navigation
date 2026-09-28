#!/usr/bin/env python3
"""Rescore individually filtered MPPI candidates with all seven native critics.

The existing graded V1 risk is evaluated on the same filtered candidate pose.
Aggregation still uses the original sampled controls and final output filter;
this is an isolated ranking diagnostic, not a runtime controller change.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, sample_omni
from costmap_mask_fixture import export, shortcut_threshold
from filtered_graded_batch_probe import filtered_poses
from native_critic_sensitivity import AXES, aggregate, open_loop_geometry
import replay_ranking


BATCHES = (300, 600, 1000, 2000)


def filtered_controls(controls, settings, history):
    limits = ((settings["vx_min"], settings["vx_max"]),
              (-settings["vy_max"], settings["vy_max"]),
              (-settings["wz_max"], settings["wz_max"]))
    return np.stack([np.stack([replay_ranking.smooth_axis(
        np.clip(control[:, axis], *limits[axis]), history[:, axis])
        for axis in range(3)], axis=-1) for control in controls])


def prepare(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"]) != (162, 300, 30):
        raise ValueError("not the frozen cycle 162")
    history_record = json.loads(args.history.read_text())
    if history_record["cycle_id"] != 162:
        raise ValueError("history belongs to another cycle")
    history = np.asarray(history_record["previous_outputs"], dtype=np.float32)
    native = json.loads(args.native_inputs.read_text())
    native_cases = {row["name"]: row for row in native["cases"]}
    raw_map = analyze.last(arrays, "locked.raw_map")
    threshold = shortcut_threshold(meta, args.profile)
    args.output_dir.mkdir(parents=True)
    rows = []
    for seed in (-1, 0, 1, 2, 3):
        if seed == -1:
            cases = (("captured_batch300", 300),)
            controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                 for axis in AXES], axis=-1)
        else:
            cases = tuple((f"seed_{seed}_batch{batch}", batch)
                          for batch in BATCHES)
            sampled, _ = sample_omni(meta, arrays, 2000, seed)
            controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
        filtered = filtered_controls(controls, settings, history)
        poses = filtered_poses(controls, meta, settings, history)
        for name, batch in cases:
            source = native_cases[name]
            source_meta = Path(source["meta"])
            if digest(source_meta) != source["sha256"]["replay_meta"]:
                raise ValueError(f"native input meta changed: {name}")
            target = args.output_dir / name
            target.mkdir()
            map_path = target / "map_and_poses.bin"
            control_path = target / "controls.bin"
            export(map_path, meta, raw_map,
                   tuple(poses[:batch, :, axis] for axis in range(3)),
                   threshold)
            control_path.write_bytes(
                struct.pack("<3I", 0x43545231, batch, 30) +
                filtered[:batch].astype("<f4").tobytes())
            rows.append({"name": name, "batch": batch, "seed": seed,
                         "source_meta": str(source_meta),
                         "source_meta_sha256": digest(source_meta),
                         "map": str(map_path.resolve()),
                         "map_sha256": digest(map_path),
                         "controls": str(control_path.resolve()),
                         "controls_sha256": digest(control_path)})
    record = {"schema": "rm_dynamic_prediction/filtered_all_critic_inputs/v1",
              "cycle_sha256": digest(args.cycle),
              "profile_sha256": digest(args.profile),
              "history_sha256": digest(args.history),
              "native_inputs_sha256": digest(args.native_inputs),
              "cases": rows}
    (args.output_dir / "inputs.json").write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "output": str(args.output_dir)}))


def native(args):
    inputs = json.loads(args.inputs.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=True)

    def mounted(host):
        return args.mount_root / Path(host).relative_to(args.host_root)

    for case in inputs["cases"]:
        for path_key, hash_key in (("source_meta", "source_meta_sha256"),
                                   ("map", "map_sha256"),
                                   ("controls", "controls_sha256")):
            if digest(mounted(case[path_key])) != case[hash_key]:
                raise ValueError(f"fixture changed: {case['name']}, {path_key}")
        score = args.output_dir / f"{case['name']}_scores.bin"
        done = subprocess.run(
            [str(args.binary), str(mounted(case["source_meta"])),
             str(mounted(case["map"])), str(mounted(case["controls"])),
             str(score)], capture_output=True, text=True, check=False)
        (args.output_dir / f"{case['name']}_log.txt").write_text(
            done.stdout + done.stderr)
        if done.returncode:
            raise RuntimeError(f"{case['name']}: {done.stderr[-1000:]}")
    print(json.dumps({"cases": len(inputs["cases"])}))


def evaluate(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    profile = yaml.safe_load(args.profile.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    padded = meta["padded_footprint"]
    history = np.asarray(json.loads(args.history.read_text())[
        "previous_outputs"], dtype=np.float32)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    truth = analyze.rows_from_transport(args.truth)
    times = [row["t"] for row in truth]
    consumed = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
        analyze.interpolated_pose(truth, times,
            consumed + (step + 1) * settings["dt"]))
        for step in range(settings["steps"])]
    inputs = json.loads(args.inputs.read_text())
    aligned = json.loads(args.aligned_summary.read_text())
    aligned_rows = {row["name"]: row for row in aligned["rows"]}
    unit = np.float32((3.81 / 254.) * 1_000_000. / 9)
    rows = []
    aggregate_poses = []
    cached_seed = None
    controls = None
    for case in inputs["cases"]:
        name, batch, seed = case["name"], case["batch"], case["seed"]
        if seed != cached_seed:
            if seed == -1:
                controls = np.stack([analyze.last(arrays, "sampled.c" + axis)
                                     for axis in AXES], axis=-1)
            else:
                sampled, _ = sample_omni(meta, arrays, 2000, seed)
                controls = np.stack([sampled[axis] for axis in AXES], axis=-1)
            cached_seed = seed
        score_path = args.native_scores / f"{name}_scores.bin"
        scores = np.fromfile(score_path, dtype="<f4")
        if scores.shape != (batch,):
            raise ValueError(f"native score shape differs: {name}")
        risk_file = args.aligned_risks / aligned_rows[name]["filtered_risk_file"]
        if digest(risk_file) != aligned_rows[name]["filtered_risk_sha256"]:
            raise ValueError(f"aligned risk changed: {name}")
        risk = np.fromfile(risk_file, dtype="<f8")[:batch]
        score = scores + np.float32(unit * risk)
        result = aggregate(score, controls[:batch], initial, settings, history)
        geometry = open_loop_geometry(result["filtered_sequence"],
                                      meta, settings, actual, body, padded)
        aggregate_poses.append(analyze.integrate_omni(
            *(result["filtered_sequence"][:, axis] for axis in range(3)),
            meta["pose"], settings["dt"]))
        rows.append({"name": name, "batch": batch,
                     "filtered_native_score_sha256": digest(score_path),
                     "aligned_risk_sha256": digest(risk_file),
                     "native_standard_score_min": float(scores.min()),
                     "native_standard_score_max": float(scores.max()),
                     "mixed_score_geometry": aligned_rows[name][
                         "aligned_geometry"],
                     "all_critic_geometry": geometry,
                     "all_critic_returned_control": result[
                         "returned_control"],
                     "all_critic_effective_sample_size": float(
                         1. / np.sum(result["probability"] ** 2))})
    export(args.static_fixture, meta, analyze.last(arrays, "locked.raw_map"),
           tuple(np.stack([pose[axis] for pose in aggregate_poses])
                 for axis in range(3)), shortcut_threshold(meta, args.profile))
    report = {"schema": "rm_dynamic_prediction/filtered_all_critic_probe/v1",
              "scope": "Frozen cycle 162: each candidate control clipped and Savitzky-Golay filtered, re-integrated with native current-speed first step, then all seven guarded native standard critics and existing continuous V1 overlap score that filtered trajectory. Original hard V1 term is common to all samples and omitted from ranking. Aggregation retains original raw controls plus final Nav2 filter, so this remains an offline ranking diagnostic, not a fully changed runtime algorithm. Truth only evaluates outputs.",
              "input_sha256": {"cycle": digest(args.cycle),
                               "profile": digest(args.profile),
                               "truth": digest(args.truth),
                               "history": digest(args.history),
                               "inputs": digest(args.inputs),
                               "aligned_summary": digest(args.aligned_summary),
                               "native_binary": digest(args.native_binary)},
              "static_fixture_sha256": digest(args.static_fixture),
              "rows": rows,
              "dynamic_gate_pass_count": sum(
                  row["all_critic_geometry"]["dynamic_clearance_gate_met"]
                  for row in rows)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "dynamic_gate_pass_count":
                      report["dynamic_gate_pass_count"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("cycle", "profile", "history", "native-inputs", "output-dir"):
        prep.add_argument("--" + name, type=Path, required=True)
    run_native = sub.add_parser("native")
    for name in ("inputs", "binary", "host-root", "mount-root", "output-dir"):
        run_native.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("cycle", "profile", "history", "truth", "inputs",
                 "native-scores", "native-binary", "aligned-summary",
                 "aligned-risks", "static-fixture", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    {"prepare": prepare, "native": native, "evaluate": evaluate}[args.command](args)
