#!/usr/bin/env python3
"""Sequentially replay frozen MPPI critic batches and compare guarded scores.

Run inside the ROS image after sourcing the isolated Nav2 and scorer overlays.
Use --compare-only to audit scores that have already been produced.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess


TOLERANCE = 1e-3


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def floats(path):
    data = path.read_bytes()
    if len(data) % 4:
        raise ValueError(f"invalid float32 score length: {path}")
    values = [value[0] for value in struct.iter_unpack("<f", data)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"non-finite score: {path}")
    return values


def cases(fixtures, references):
    yield "captured_batch300", 300, (
        fixtures.parent / "native_critic_capture_v3_20260926" / "meta.json",
        fixtures.parent / "native_critic_capture_v3_20260926" / "captured.bin",
        fixtures.parent / "native_critic_capture_v3_20260926" / "controls.bin",
        references / "captured_batch300" / "guarded_native_scores.bin",
    )
    for seed in range(4):
        for batch in (300, 600, 1000, 2000):
            item = fixtures / f"seed{seed}_batch{batch}"
            label = f"seed_{seed}_batch{batch}"
            yield label, batch, (
                item / "meta.json", item / f"seed_{seed}.bin",
                item / "controls.bin",
                references / label / "guarded_native_scores.bin",
            )


def validate(args):
    args.output.mkdir(parents=True, exist_ok=True)
    records = []
    for label, batch, (meta, map_file, controls, reference) in cases(
        args.fixtures, args.references
    ):
        result = args.output / f"{label}_cpp_scores.bin"
        log = args.output / f"{label}_scorer.log"
        if not args.compare_only:
            if args.scorer is None:
                raise ValueError("--scorer is required for replay")
            completed = subprocess.run(
                [str(args.scorer), str(meta), str(map_file), str(controls), str(result)],
                capture_output=True, text=True, check=False,
            )
            log.write_text(completed.stdout + completed.stderr)
            if completed.returncode:
                raise RuntimeError(f"{label}: scorer failed: {completed.stderr}")
        actual_values = floats(result)
        reference_values = floats(reference)
        if len(actual_values) != batch or len(reference_values) != batch:
            raise ValueError(f"{label}: expected {batch} scores")
        differences = [abs(a - b) for a, b in zip(actual_values, reference_values)]
        if max(differences) > TOLERANCE:
            raise AssertionError(f"{label}: max score difference {max(differences)}")
        log_text = log.read_text()
        match = re.search(r"score_eval_ms=([0-9.]+)", log_text)
        records.append({
            "case": label,
            "batch": batch,
            "max_abs_error": max(differences),
            "count_error_gt_0_001": sum(value > TOLERANCE for value in differences),
            "score_eval_ms": float(match.group(1)) if match else None,
            "input_sha256": {str(path): digest(path) for path in (meta, map_file, controls)},
            "reference_sha256": digest(reference),
            "cpp_scores_sha256": digest(result),
        })
    summary = {
        "schema": "rm_dynamic_prediction/guarded_nav2_cpp_validation/v1",
        "tolerance_abs_score": TOLERANCE,
        "environment": {
            "image_id": args.image_id,
            "source_manifest": json.loads(args.source_manifest.read_text()),
            "source_manifest_sha256": digest(args.source_manifest),
            "nav2_critics_library_sha256": digest(args.plugin),
            "scorer_sha256": digest(args.scorer),
            "nav2_build_log_sha256": digest(args.nav2_build_log),
            "scorer_build_log_sha256": digest(args.scorer_build_log),
            "build_parallel_workers": 1,
            "cmake_build_parallel_level": 1,
        },
        "cases": records,
        "total_scores": sum(row["batch"] for row in records),
        "max_abs_error": max(row["max_abs_error"] for row in records),
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    if args.copy_scores:
        args.copy_scores.mkdir(parents=True, exist_ok=True)
        for row in records:
            name = row["case"] + "_cpp_scores.bin"
            shutil.copyfile(args.output / name, args.copy_scores / name)
    print(json.dumps({"cases": len(records), "total_scores": summary["total_scores"],
                      "max_abs_error": summary["max_abs_error"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixtures", required=True, type=Path)
    parser.add_argument("--references", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--scorer", required=True, type=Path)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--source-manifest", required=True, type=Path)
    parser.add_argument("--plugin", required=True, type=Path)
    parser.add_argument("--nav2-build-log", required=True, type=Path)
    parser.add_argument("--scorer-build-log", required=True, type=Path)
    parser.add_argument("--compare-only", action="store_true")
    parser.add_argument("--copy-scores", type=Path)
    validate(parser.parse_args())
