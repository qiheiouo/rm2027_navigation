#!/usr/bin/env python3
"""Prepare an isolated Nav2 1.1.20 MPPI source with a PathAlign end guard.

This copies the already archived diagnostic package; it never edits the
project's normal Nav2 installation or the preserved source archive.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


SOURCE_HEADER_SHA256 = "cd48e40c5e698eaf6568ef6f50671e1b90190c8210464d2b7dff54fdc5f2dbe7"
HEADER = Path("include/nav2_mppi_controller/tools/utils.hpp")
OLD = """  if (iter == vec.begin() + init) {
    return 0;
  }
  if (dist - *(iter - 1) < *iter - dist) {
"""
NEW = """  if (iter == vec.begin() + init) {
    return 0;
  }
  // A trajectory may extend beyond the integrated local path. In that case
  // lower_bound returns end, which must not be dereferenced below.
  if (iter == vec.end()) {
    return vec.size() - 1;
  }
  if (dist - *(iter - 1) < *iter - dist) {
"""


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(source, destination):
    if destination.exists():
        raise FileExistsError(destination)
    if digest(source / HEADER) != SOURCE_HEADER_SHA256:
        raise ValueError("archived Nav2 utility header differs from frozen image")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, destination)
    path = destination / HEADER
    current = path.read_text()
    if current.count(OLD) != 1:
        raise ValueError("PathAlign helper no longer has the expected source")
    path.write_text(current.replace(OLD, NEW))
    record = {
        "schema": "rm_dynamic_prediction_guarded_nav2_source/v1",
        "source": str(source.resolve()),
        "source_header_sha256": SOURCE_HEADER_SHA256,
        "patched_header_sha256": digest(path),
        "source_license_sha256": digest(source / "LICENSE.md"),
        "change": "After the existing begin+init branch, return the final valid integrated path index when lower_bound returns end.",
    }
    (destination.parent / "source_manifest.json").write_text(
        json.dumps(record, indent=2, sort_keys=True) + "\n")
    return record


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.source, args.destination)))
