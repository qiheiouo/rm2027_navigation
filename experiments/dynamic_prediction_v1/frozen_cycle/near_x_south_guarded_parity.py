#!/usr/bin/env python3
"""Validate guarded south 4x C++ scores against six critics plus Python align."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import analyze
from batch_sampling_probe import digest
from guarded_path_align_probe import guarded_align_scores
from near_x_scale4_score import controls_for, raw_poses
from near_x_south_batch import SEEDS, frozen


TOLERANCE = 1e-3


def prepare(trial, score_inputs, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, meta, arrays, settings, profile_path, _ = frozen(trial)
    source = json.loads((score_inputs / "inputs.json").read_text())
    if source["cycle_sha256"] != digest(cycle):
        raise ValueError("guarded scoring source differs")
    profile = yaml.safe_load(profile_path.read_text())
    align = profile["controller_server"]["ros__parameters"]["FollowPath"][
        "PathAlignCritic"]
    if (align["cost_power"], align["trajectory_point_step"],
            align["offset_from_furthest"], align["use_path_orientations"]) != \
            (1, 4, 20, False):
        raise ValueError("guarded PathAlign profile changed")
    raw = analyze.last(arrays, "locked.raw_map")
    output.mkdir(parents=True)
    rows = []
    for seed in SEEDS:
        case = score_inputs / f"seed_{seed}"
        native = json.loads((case / "meta.json").read_text())
        critics = native["parameters"]["FollowPath.critics"]
        if critics.count("PathAlignCritic") != 1:
            raise ValueError("PathAlign not unique")
        native["parameters"]["FollowPath.critics"] = [
            critic for critic in critics if critic != "PathAlignCritic"]
        meta_file = output / f"seed_{seed}_without_align_meta.json"
        meta_file.write_text(json.dumps(native, indent=2, sort_keys=True) + "\n")
        trajectory = raw_poses(controls_for(meta, arrays, seed), meta, settings)
        guarded, exposed, furthest = guarded_align_scores(
            meta["path"], np.stack(trajectory[:2], axis=-1),
            align["cost_weight"], align["trajectory_point_step"], raw,
            meta["map"])
        align_file = output / f"seed_{seed}_python_align.bin"
        guarded.astype("<f4").tofile(align_file)
        rows.append({"seed": seed, "furthest_path_index": furthest,
                     "end_lookup_exposure_count": int(np.sum(exposed >= 0)),
                     "meta_sha256": digest(meta_file),
                     "python_align_sha256": digest(align_file),
                     "source_meta_sha256": digest(case / "meta.json"),
                     "source_map_sha256": digest(case / "raw_map.bin"),
                     "source_controls_sha256": digest(case / "controls.bin")})
    report = {"schema": "rm_dynamic_prediction_south_guarded_parity_inputs/v1",
              "cycle_sha256": digest(cycle),
              "score_inputs_sha256": digest(score_inputs / "inputs.json"),
              "tolerance": TOLERANCE, "rows": rows}
    (output / "inputs.json").write_text(json.dumps(report, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"cases": len(rows), "output": str(output)}))


def evaluate(trial, score_inputs, parity, guarded_scores, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, _, _, _, _, _ = frozen(trial)
    record = json.loads((parity / "inputs.json").read_text())
    if record["cycle_sha256"] != digest(cycle) or \
            record["score_inputs_sha256"] != digest(score_inputs / "inputs.json"):
        raise ValueError("guarded parity inputs changed")
    rows = []
    for item in record["rows"]:
        seed = item["seed"]
        case = score_inputs / f"seed_{seed}"
        for path, key in ((parity / f"seed_{seed}_without_align_meta.json",
                           "meta_sha256"),
                          (parity / f"seed_{seed}_python_align.bin",
                           "python_align_sha256"),
                          (case / "meta.json", "source_meta_sha256"),
                          (case / "raw_map.bin", "source_map_sha256"),
                          (case / "controls.bin", "source_controls_sha256")):
            if digest(path) != item[key]:
                raise ValueError("guarded parity source changed")
        other_path = parity / f"seed_{seed}_six_scores.bin"
        guarded_path = guarded_scores / f"seed_{seed}_scores.bin"
        other = np.fromfile(other_path, dtype="<f4")
        align = np.fromfile(parity / f"seed_{seed}_python_align.bin", dtype="<f4")
        guarded = np.fromfile(guarded_path, dtype="<f4")
        if other.shape != align.shape or other.shape != guarded.shape or \
                other.shape != (2000,) or not all(np.isfinite(v).all()
                                                   for v in (other, align, guarded)):
            raise ValueError("guarded parity score shape or finiteness differs")
        difference = np.abs(other + align - guarded)
        rows.append({"seed": seed,
                     "end_lookup_exposure_count": item["end_lookup_exposure_count"],
                     "six_scores_sha256": digest(other_path),
                     "guarded_cpp_sha256": digest(guarded_path),
                     "max_abs_error": float(difference.max()),
                     "error_gt_1e_3_count": int(np.sum(difference > TOLERANCE))})
    result = {"schema": "rm_dynamic_prediction_south_guarded_parity/v1",
              "scope": "Four fixed 2000-row scale-four inputs; six native critics plus independent Python guarded PathAlign versus isolated guarded seven-critic C++. No runtime controller change.",
              "inputs_sha256": digest(parity / "inputs.json"),
              "tolerance_abs_score": TOLERANCE,
              "all_pass": all(row["error_gt_1e_3_count"] == 0 for row in rows),
              "rows": rows}
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"all_pass": result["all_pass"],
                      "max_abs_error": max(row["max_abs_error"] for row in rows)}))
    if not result["all_pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    prep = sub.add_parser("prepare")
    for name in ("trial", "score-inputs", "output"):
        prep.add_argument("--" + name, type=Path, required=True)
    check = sub.add_parser("evaluate")
    for name in ("trial", "score-inputs", "parity", "guarded-scores", "output"):
        check.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.score_inputs, args.output)
    else:
        evaluate(args.trial, args.score_inputs, args.parity,
                 args.guarded_scores, args.output)
