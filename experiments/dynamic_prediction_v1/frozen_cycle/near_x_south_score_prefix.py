#!/usr/bin/env python3
"""Locate the two south-start native total score replay differences."""
import argparse
import json
from pathlib import Path

import numpy as np

import analyze
from batch_sampling_probe import digest
from near_x_south_batch import frozen


CRITICS = ("ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
           "PathAlignCritic", "PathFollowCritic", "PathAngleCritic")
NATIVE_SHA = "1046705dfdbe4e7a4996c23c0999d1988d06c775f058425fe46df34a18dee118"


def prepare(trial, native_binary, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, _, _, _, _, _ = frozen(trial)
    fixture = trial / "native_raw_cycle_65"
    meta_file = fixture / "meta.json"
    meta = json.loads(meta_file.read_text())
    if tuple(meta["parameters"]["FollowPath.critics"]) != CRITICS or \
            meta["batch"] != 300 or digest(native_binary) != NATIVE_SHA:
        raise ValueError("native scorer or frozen critic order changed")
    output.mkdir(parents=True)
    hashes = {}
    for index in range(1, 8):
        modified = json.loads(json.dumps(meta))
        modified["parameters"]["FollowPath.critics"] = CRITICS[:index]
        path = output / f"prefix_{index}_meta.json"
        path.write_text(json.dumps(modified, indent=2, sort_keys=True) + "\n")
        hashes[path.name] = digest(path)
    record = {"schema": "rm_dynamic_prediction_south_native_prefix_inputs/v1",
              "cycle_sha256": digest(cycle),
              "native_binary_sha256": digest(native_binary),
              "fixture_sha256": {p.name: digest(p) for p in (
                  meta_file, fixture / "captured.bin", fixture / "controls.bin",
                  fixture / "raw_scores.bin")},
              "prefix_meta_sha256": hashes, "critic_order": CRITICS}
    (output / "inputs.json").write_text(json.dumps(record, indent=2,
                                                   sort_keys=True) + "\n")
    print(json.dumps({"prefixes": len(CRITICS), "output": str(output)}))


def evaluate(trial, native_binary, fixture, output):
    if output.exists():
        raise FileExistsError(output)
    cycle, _, arrays, _, _, _ = frozen(trial)
    record = json.loads((fixture / "inputs.json").read_text())
    if record["cycle_sha256"] != digest(cycle) or \
            record["native_binary_sha256"] != digest(native_binary) or \
            tuple(record["critic_order"]) != CRITICS:
        raise ValueError("native prefix input changed")
    for name, sha in record["prefix_meta_sha256"].items():
        if digest(fixture / name) != sha:
            raise ValueError("native prefix meta changed")
    source = trial / "native_raw_cycle_65"
    for name, sha in record["fixture_sha256"].items():
        if digest(source / name) != sha:
            raise ValueError("captured native scorer fixture changed")
    rows = []
    previous_native = np.zeros(300, dtype=np.float64)
    previous_captured = np.zeros(300, dtype=np.float64)
    first = {}
    for index, critic in enumerate(CRITICS, 1):
        path = fixture / f"prefix_{index}_scores.bin"
        native = np.fromfile(path, dtype="<f4").astype(np.float64)
        captured = analyze.last(arrays, "critic.FollowPath." + critic).astype(np.float64)
        if native.shape != (300,) or captured.shape != native.shape:
            raise ValueError("native prefix score size differs")
        cumulative = native - captured
        term = (native - previous_native) - (captured - previous_captured)
        changed = np.flatnonzero(np.abs(cumulative) > 1e-3)
        term_changed = np.flatnonzero(np.abs(term) > 1e-3)
        for rollout in term_changed:
            first.setdefault(str(int(rollout)), critic)
        rows.append({"critic": critic, "prefix": index,
                     "native_scores_sha256": digest(path),
                     "cumulative_mismatch_count_gt_1e_3": len(changed),
                     "cumulative_mismatch_rollouts": changed.astype(int).tolist(),
                     "cumulative_max_abs_error": float(np.max(np.abs(cumulative))),
                     "increment_mismatch_rollouts": term_changed.astype(int).tolist(),
                     "increment_max_abs_error": float(np.max(np.abs(term)))})
        previous_native, previous_captured = native, captured
    original = np.fromfile(source / "raw_scores.bin", dtype="<f4")
    final = np.fromfile(fixture / "prefix_7_scores.bin", dtype="<f4")
    if not np.array_equal(final, original):
        raise ValueError("seven-prefix score does not reproduce original native total")
    report = {"schema": "rm_dynamic_prediction_south_native_prefix/v1",
              "scope": "Sequential original seven critic prefixes on captured controls and raw costmap; diagnose scorer parity only, not a safety or ranking adjustment.",
              "inputs_sha256": digest(fixture / "inputs.json"),
              "first_changed_critic_by_rollout": first, "rows": rows}
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"first_changed_critic_by_rollout": first,
                      "final_mismatch_count": rows[-1][
                          "cumulative_mismatch_count_gt_1e_3"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("prepare", "evaluate"):
        part = sub.add_parser(name)
        part.add_argument("--trial", type=Path, required=True)
        part.add_argument("--native-binary", type=Path, required=True)
        part.add_argument("--fixture", type=Path, required=True)
        if name == "evaluate":
            part.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.trial, args.native_binary, args.fixture)
    else:
        evaluate(args.trial, args.native_binary, args.fixture, args.output)
