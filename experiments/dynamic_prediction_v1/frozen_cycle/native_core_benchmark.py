#!/usr/bin/env python3
"""Measure repeated frozen MPPI core replays in one sequential Docker job.

This benchmarks the injected-control core only. It excludes noise generation,
ROS controller callbacks, fixture loading, and output serialization. Each
repetition starts a fresh process; no measurements are pooled across batches.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import time


COMPONENTS = ("integrate_ms", "score_ms", "prediction_ms",
              "aggregate_ms", "filter_ms")
BATCHES = (300, 600, 1000, 2000)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentile(values, quantile):
    values = sorted(values)
    position = (len(values) - 1) * quantile
    lower = int(position)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def stats(values):
    return {"p50": statistics.median(values), "p95": percentile(values, .95),
            "max": max(values), "min": min(values)}


def mapped(value, host_root, mount_root):
    return mount_root / Path(value).relative_to(host_root)


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.repeats < 2 or args.warmups < 1:
        raise ValueError("at least two measurements and one warmup required")
    inputs = json.loads(args.inputs.read_text())
    selected = {row["batch"]: row for row in inputs["cases"]
                if row["seed"] == 0}
    if set(selected) != set(BATCHES):
        raise ValueError("seed-zero batch fixtures differ")
    if digest(args.binary) != args.expected_binary_sha256:
        raise ValueError("native binary differs from accepted replay")
    args.scratch.mkdir(parents=True, exist_ok=True)
    rows = []
    for batch in BATCHES:
        case = selected[batch]
        paths = {key: mapped(case[key], args.host_root, args.mount_root)
                 for key in ("meta", "map", "controls")}
        for key, path in paths.items():
            expected = case["sha256"]["replay_meta" if key == "meta" else key]
            if digest(path) != expected:
                raise ValueError(f"fixture changed: {path}")
        prefix = args.scratch / f"batch{batch}"
        measured = []
        expected_control = mapped(
            args.reference / f"seed_0_batch{batch}_after_filter.bin",
            args.host_root, args.mount_root)
        for index in range(args.warmups + args.repeats):
            log = args.scratch / f"batch{batch}_last.log"
            with log.open("wb") as stream:
                start = time.perf_counter()
                child = subprocess.Popen(
                    [str(args.binary), str(paths["meta"]), str(paths["map"]),
                     str(paths["controls"]), str(prefix)],
                    stdout=stream, stderr=subprocess.STDOUT)
                _, status, usage = os.wait4(child.pid, 0)
                wall_ms = (time.perf_counter() - start) * 1000.
                child.returncode = os.waitstatus_to_exitcode(status)
            if child.returncode:
                raise RuntimeError(f"batch {batch}, repetition {index}: {log.read_text()[-1000:]}")
            control = Path(str(prefix) + "_after_filter.bin")
            if digest(control) != digest(expected_control):
                raise AssertionError(f"batch {batch}: replay output changed")
            if index < args.warmups:
                continue
            timing = json.loads(Path(str(prefix) + "_timing.json").read_text())
            if timing["batch"] != batch:
                raise ValueError("timing batch differs")
            core_ms = sum(timing[key] for key in COMPONENTS)
            measured.append({"core_ms": core_ms, "wall_ms": wall_ms,
                             "peak_rss_kib": usage.ru_maxrss,
                             **{key: timing[key] for key in COMPONENTS}})
        rows.append({
            "batch": batch, "case": case["name"],
            "fixture_sha256": case["sha256"],
            "reference_after_filter_sha256": digest(expected_control),
            "core_ms": stats([item["core_ms"] for item in measured]),
            "prediction_ms": stats([item["prediction_ms"] for item in measured]),
            "process_wall_ms": stats([item["wall_ms"] for item in measured]),
            "peak_rss_kib": stats([item["peak_rss_kib"] for item in measured]),
            "deadline_exceeded_core_count": sum(item["core_ms"] > 100
                                                for item in measured),
            "measurements": measured,
        })
        print(json.dumps({"batch": batch, "core_ms": rows[-1]["core_ms"],
                          "peak_rss_kib": rows[-1]["peak_rss_kib"]}), flush=True)
    report = {
        "schema": "rm_dynamic_prediction/native_core_benchmark/v1",
        "scope": "Fresh-process sequential replay of frozen seed-zero controls; timer covers native dynamics, seven guarded standard critics, frozen V1 geometry mirror, MPPI aggregation and output filter. It excludes noise generation, fixture loading, ROS controller callbacks and output serialization. Process wall time and process peak RSS include initialization and serialization.",
        "binary_sha256": digest(args.binary), "inputs_sha256": digest(args.inputs),
        "repetitions_per_batch": args.repeats, "warmups_per_batch": args.warmups,
        "omp_num_threads": os.environ.get("OMP_NUM_THREADS"), "rows": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("inputs", "binary", "host-root", "mount-root", "reference",
                 "scratch", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--expected-binary-sha256", required=True)
    parser.add_argument("--warmups", type=int, default=2)
    parser.add_argument("--repeats", type=int, default=20)
    run(parser.parse_args())
