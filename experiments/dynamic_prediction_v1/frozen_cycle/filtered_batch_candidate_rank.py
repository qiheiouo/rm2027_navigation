#!/usr/bin/env python3
"""Audit filtered-candidate safety and ranking for frozen batch prefixes.

Uses prior native standard critic scores and V1 continuous-overlap arrays.
Native CostCritic masks are generated separately on each stored raw-costmap
fixture. This is a same-cycle sensitivity study, not a closed-loop result.
"""
import argparse
import csv
import json
from pathlib import Path
import struct

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest, geometry_labels
from filtered_candidate_rank_audit import ranked_metrics
from native_critic_sensitivity import AXES, aggregate
import replay_ranking


MAGIC = 0x43504D31
HEADER = struct.Struct("<7I3d")
CONTROL_HEADER = struct.Struct("<3I")


def fixture_poses(path, batch):
    data = path.read_bytes()
    magic, width, height, count, steps, vertices, threshold, *_ = \
        HEADER.unpack_from(data)
    if magic != MAGIC or count != batch or steps != 30 or vertices < 3 or \
            width * height > 1000000 or not 1 <= threshold <= 253:
        raise ValueError(f"bad candidate fixture: {path}")
    offset = HEADER.size + vertices * 2 * 8 + width * height
    if len(data) != offset + batch * steps * 3 * 4:
        raise ValueError(f"candidate fixture truncated: {path}")
    return np.frombuffer(data, dtype="<f4", count=batch * steps * 3,
                         offset=offset).reshape(batch, steps, 3)


def raw_controls(path, batch):
    data = path.read_bytes()
    magic, count, steps = CONTROL_HEADER.unpack_from(data)
    if magic != 0x43545231 or count != batch or steps != 30 or \
            len(data) != CONTROL_HEADER.size + batch * steps * 3 * 4:
        raise ValueError(f"bad raw controls: {path}")
    return np.frombuffer(data, dtype="<f4",
                         offset=CONTROL_HEADER.size).reshape(batch, steps, 3)


def native_mask(path, batch):
    values = [int(value) for value in path.read_text().split()]
    if len(values) != batch or any(value not in (0, 1) for value in values):
        raise ValueError(f"native static mask invalid: {path}")
    return np.asarray(values, dtype=bool)


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    inputs = json.loads(args.filtered_inputs.read_text())
    raw_inputs = json.loads(args.raw_inputs.read_text())
    aligned = json.loads(args.aligned_summary.read_text())
    all_critic = json.loads(args.all_critic_summary.read_text())
    by_raw = {row["name"]: row for row in raw_inputs["cases"]}
    by_risk = {row["name"]: row for row in aligned["rows"]}
    by_score = {row["name"]: row for row in all_critic["rows"]}
    meta, arrays = analyze.read_cycle(args.cycle)
    settings = analyze.event(meta, "settings")
    if (meta["cycle_id"], settings["batch"], settings["steps"]) != (162, 300, 30):
        raise ValueError("not frozen cycle 162")
    profile = yaml.safe_load(args.profile.read_text())
    body = yaml.safe_load(profile["local_costmap"]["local_costmap"][
        "ros__parameters"]["footprint"])
    truth = analyze.rows_from_transport(args.truth)
    times = [row["t"] for row in truth]
    stamp = analyze.event(meta, "prediction.input")["consumer_sim_s"]
    actual = [analyze.placed(analyze.obstacle_polygon(),
              analyze.interpolated_pose(truth, times,
                                        stamp + (step + 1) * settings["dt"]))
              for step in range(settings["steps"])]
    previous = replay_ranking.history_from_trial(
        args.cycle.parent, 162, arrays)
    history = np.stack([previous[axis] for axis in AXES], axis=-1)
    initial = np.stack([analyze.last(arrays, "initial." + axis)
                        for axis in AXES], axis=-1)
    max_rows = {row["seed"]: row for row in inputs["cases"]
                if row["batch"] == 2000 or row["seed"] == -1}
    cached = {}
    labels = []
    for seed, source in max_rows.items():
        batch = source["batch"]
        fixture = Path(source["map"])
        if digest(fixture) != source["map_sha256"]:
            raise ValueError(f"filtered fixture changed: {source['name']}")
        poses = fixture_poses(fixture, batch)
        body_gap, padded_gap = geometry_labels(
            tuple(poses[:, :, axis] for axis in range(3)), body,
            meta["padded_footprint"], actual)
        mask_file = args.mask_dir / f"{source['name']}_mask.txt"
        mask = native_mask(mask_file, batch)
        cached[seed] = (poses, body_gap, padded_gap, mask)
        detail = args.output.parent / f"{source['name']}_labels.csv"
        with detail.open("x", newline="") as output:
            writer = csv.writer(output, lineterminator="\n")
            writer.writerow(("rollout", "body_gap_3s_m", "padded_gap_3s_m",
                             "static_collision", "joint_safe_3s"))
            for index in range(batch):
                writer.writerow((index, body_gap[index], padded_gap[index],
                                 int(mask[index]), int(body_gap[index] >= .05
                                      and padded_gap[index] > 0 and
                                      not mask[index])))
        labels.append({"name": source["name"], "detail_file": detail.name,
                       "detail_sha256": digest(detail),
                       "fixture_sha256": source["map_sha256"],
                       "static_mask_sha256": digest(mask_file)})
    rows = []
    unit = np.float32((3.81 / 254.) * 1_000_000. / 9)
    for case in inputs["cases"]:
        name, batch, seed = case["name"], case["batch"], case["seed"]
        if name not in by_raw or name not in by_risk or name not in by_score:
            raise ValueError(f"batch case missing from prior evidence: {name}")
        fixture = Path(case["map"])
        if digest(fixture) != case["map_sha256"]:
            raise ValueError(f"filtered fixture changed: {name}")
        poses = fixture_poses(fixture, batch)
        max_poses, max_body, max_padded, max_mask = cached[seed]
        if not np.array_equal(poses, max_poses[:batch]):
            raise ValueError(f"batch candidates are not nested: {name}")
        mask_file = args.mask_dir / f"{name}_mask.txt"
        mask = native_mask(mask_file, batch)
        if not np.array_equal(mask, max_mask[:batch]):
            raise ValueError(f"native static masks are not nested: {name}")
        safe = (max_body[:batch] >= .05) & (max_padded[:batch] > 0) & ~mask
        risk_row, score_row, raw_row = (by_risk[name], by_score[name],
                                        by_raw[name])
        risk_path = args.risk_dir / risk_row["filtered_risk_file"]
        score_path = args.score_dir / f"{name}_scores.bin"
        controls_path = Path(raw_row["controls"])
        for path, expected in ((risk_path, risk_row["filtered_risk_sha256"]),
                               (score_path, score_row["filtered_native_score_sha256"]),
                               (controls_path, raw_row["sha256"]["controls"])):
            if digest(path) != expected:
                raise ValueError(f"frozen score/control changed: {path}")
        risk = np.fromfile(risk_path, dtype="<f8")[:batch]
        standard = np.fromfile(score_path, dtype="<f4")
        controls = raw_controls(controls_path, batch)
        if risk.shape != standard.shape or standard.shape != (batch,):
            raise ValueError(f"score dimensions differ: {name}")
        prediction = np.asarray(unit * risk, dtype=np.float32)
        result = aggregate(standard + prediction, controls, initial,
                           settings, history)
        published = np.asarray(score_row["all_critic_returned_control"])
        output_error = float(np.max(np.abs(
            np.asarray(result["returned_control"]) - published)))
        if output_error > 1e-5:
            raise ValueError(f"published aggregate not reproduced: {name}")
        rows.append({"name": name, "seed": seed, "batch": batch,
                     "dynamic_safe_count": int(np.count_nonzero(
                         (max_body[:batch] >= .05) &
                         (max_padded[:batch] > 0))),
                     "static_collision_count": int(mask.sum()),
                     "joint_safe_count": int(safe.sum()),
                     "best_joint_safe_body_gap_m": (float(max_body[:batch][safe].max())
                                                    if safe.any() else None),
                     "joint_safe_probability_mass": float(
                         result["probability"][safe].sum()),
                     "prediction_ranking": ranked_metrics(prediction, safe),
                     "native_standard_ranking": ranked_metrics(standard, safe),
                     "total_weighted_ranking": ranked_metrics(
                         result["weighted"], safe),
                     "effective_sample_size": float(1. / np.sum(
                         result["probability"] ** 2)),
                     "published_output_max_abs_error": output_error,
                     "output_joint_gate_met": bool(score_row[
                         "all_critic_geometry"]["joint_clearance_gate_met"]),
                     "input_sha256": {"fixture": case["map_sha256"],
                                      "native_mask": digest(mask_file),
                                      "risk": risk_row["filtered_risk_sha256"],
                                      "score": score_row[
                                          "filtered_native_score_sha256"],
                                      "raw_controls": raw_row["sha256"][
                                          "controls"]}})
    report = {"schema": "rm_dynamic_prediction/filtered_batch_candidate_rank/v1",
              "scope": "Frozen cycle 162 only. Captured batch300 plus four deterministic nested seeds at 300/600/1000/2000. Each individual filtered candidate has 30-step true physical-box geometry and native raw-costmap CostCritic mask. Seven native standard critic scores plus prior uncalibrated V1 filtered overlap term; common hard term omitted from ranking. Original raw-control MPPI regularizer, softmax and final filter reproduced against published outputs. Truth only labels; results across batch prefixes and seeds are correlated.",
              "input_sha256": {"cycle": digest(args.cycle),
                               "profile": digest(args.profile),
                               "truth": digest(args.truth),
                               "filtered_inputs": digest(args.filtered_inputs),
                               "raw_inputs": digest(args.raw_inputs),
                               "aligned_summary": digest(args.aligned_summary),
                               "all_critic_summary": digest(args.all_critic_summary),
                               "native_mask_binary": digest(args.native_binary)},
              "label_files": labels, "rows": rows}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cycle", "profile", "truth", "filtered-inputs", "raw-inputs",
                 "aligned-summary", "all-critic-summary", "risk-dir",
                 "score-dir", "mask-dir", "native-binary", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
