#!/usr/bin/env python3
"""Rescore existing individually filtered candidates with guarded PathAlign.

Requires the prebuilt single-threaded native seven-critic scorer. The source
fixtures are read-only; this script records hashes and does not use truth.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def floats(path):
    data = path.read_bytes()
    if len(data) % 4:
        raise ValueError(f"malformed float32 score file: {path}")
    return [value for (value,) in struct.iter_unpack("<f", data)]


def mounted(path, host_root, mount_root):
    return mount_root / Path(path).relative_to(host_root)


def run(args):
    if args.output_dir.exists():
        raise FileExistsError(args.output_dir)
    manifest = json.loads(args.inputs.read_text())
    if (manifest["schema"] !=
            "rm_dynamic_prediction/filtered_all_critic_inputs/v1" or
            len(manifest["cases"]) != 17):
        raise ValueError("expected 17 frozen filtered-candidate fixtures")
    args.output_dir.mkdir(parents=True)
    records = []
    for case in manifest["cases"]:
        name, batch = case["name"], case["batch"]
        if name != ("captured_batch300" if case["seed"] == -1 else
                    f"seed_{case['seed']}_batch{batch}"):
            raise ValueError(f"unexpected case identity: {name}")
        paths = {key: mounted(case[key], args.host_root, args.mount_root)
                 for key in ("source_meta", "map", "controls")}
        for key, path in paths.items():
            if digest(path) != case[key + "_sha256"]:
                raise ValueError(f"source fixture changed: {name}/{key}")
        score = args.output_dir / f"{name}_scores.bin"
        log = args.output_dir / f"{name}_scorer.log"
        done = subprocess.run([
            str(args.binary), str(paths["source_meta"]), str(paths["map"]),
            str(paths["controls"]), str(score)], capture_output=True,
            text=True, check=False)
        log.write_text(done.stdout + done.stderr)
        if done.returncode:
            raise RuntimeError(f"guarded native score failed {name}: "
                               f"{done.stderr[-1000:]}")
        new = floats(score)
        old_path = args.old_scores / f"{name}_scores.bin"
        old = floats(old_path)
        if len(new) != batch or len(old) != batch:
            raise ValueError(f"score count changed: {name}")
        differences = [abs(a - b) for a, b in zip(new, old)]
        records.append({"name": name, "batch": batch,
                        "source_fixture_sha256": {key: case[key + "_sha256"]
                                                  for key in paths},
                        "guarded_score_sha256": digest(score),
                        "old_score_sha256": digest(old_path),
                        "score_count": len(new),
                        "old_vs_guarded_max_abs": max(differences),
                        "old_vs_guarded_over_0_001": sum(
                            value > .001 for value in differences),
                        "log_sha256": digest(log)})
    report = {"schema": "rm_dynamic_prediction/guarded_filtered_score_replay/v1",
              "scope": "All 17 frozen filtered-candidate fixtures rescored sequentially using a prebuilt Nav2 native seven-critic scorer with the documented PathAlign end guard. No future truth or runtime controller change. Old score comparison is diagnostic; undefined old PathAlign values are not a stable baseline.",
              "inputs_sha256": digest(args.inputs),
              "scorer_sha256": digest(args.binary),
              "cases": records}
    (args.output_dir / "summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"cases": len(records),
                      "output": str(args.output_dir)}, sort_keys=True))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--old-scores", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--host-root", type=Path, required=True)
    parser.add_argument("--mount-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    run(parser.parse_args())
