#!/usr/bin/env python3
"""Archive the fixed two-cycle safe-support oracle and its native static masks."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil


PRIOR_MANIFESTS = {
    "north": ("near_x_crossing_20260930",
              "7422df027e34c0fc8708e01652581a592cc2865293b38aac7a63b37678fe1ca6"),
    "south": ("near_x_south_crossing_20260930",
              "1b3739cf0ba8909961a7b37e3a9e68c295729cf3e518703dd90a31a141e631c5")}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def verify_prior(root, side):
    directory, digest = PRIOR_MANIFESTS[side]
    folder = root / "docs/dynamic_navigation/evidence" / directory
    data = (folder / "manifest.json").read_bytes()
    if sha(data) != digest:
        raise ValueError(side + " prior evidence manifest changed")
    manifest = json.loads(data)
    for name, record in manifest["files"].items():
        artifact = (folder / name).read_bytes()
        if sha(artifact) != record["artifact_sha256"]:
            raise ValueError(side + " prior artifact changed: " + name)
        source = gzip.decompress(artifact) if record["gzip"] else artifact
        if sha(source) != record["source_sha256"]:
            raise ValueError(side + " prior source changed: " + name)
    return folder / "manifest.json"


def export(root, work, output):
    if output.exists():
        raise FileExistsError(output)
    summary = json.loads((work / "summary.json").read_text())
    if summary["schema"] != "rm_dynamic_prediction_near_x_safe_support_oracle/v1" or \
            len(summary["rows"]) != 16:
        raise ValueError("oracle summary changed")
    for side in PRIOR_MANIFESTS:
        if sha((work / f"{side}_aggregate_map.bin").read_bytes()) != \
                summary["map_sha256"][side] or \
                sha((work / f"{side}_static_mask.txt").read_bytes()) != \
                summary["static_mask_sha256"][side]:
            raise ValueError(side + " output changed")
    previous = {side: verify_prior(root, side) for side in PRIOR_MANIFESTS}
    output.mkdir(parents=True)
    sources = {
        "preliminary.json": work / "preliminary.json",
        "summary.json": work / "summary.json",
        "north_aggregate_map.bin": work / "north_aggregate_map.bin",
        "south_aggregate_map.bin": work / "south_aggregate_map.bin",
        "north_static_mask.txt": work / "north_static_mask.txt",
        "south_static_mask.txt": work / "south_static_mask.txt",
        "native_checker": root / "build/phase2_native_mask_head_20260929/install/costmap_mask_probe_cpp/lib/costmap_mask_probe_cpp/costmap_mask_probe",
        "north_prior_manifest.json": previous["north"],
        "south_prior_manifest.json": previous["south"]}
    files = {}
    for name, source in sources.items():
        content = source.read_bytes()
        compressed = name.endswith(".bin") or name == "native_checker"
        target = output / (name + ".gz" if compressed else name)
        if compressed:
            with target.open("xb") as result:
                with gzip.GzipFile(filename="", fileobj=result, mode="wb",
                                   mtime=0) as stream:
                    stream.write(content)
        else:
            shutil.copyfile(source, target)
        files[target.name] = {"source": str(source),
                              "source_sha256": sha(content),
                              "artifact_sha256": sha(target.read_bytes()),
                              "gzip": compressed}
    report = {
        "schema": "rm_dynamic_prediction_near_x_safe_support_export/v1",
        "scope": "Existing north and south frozen cycles, 16 offline oracle outputs, original static map checked with pinned native Nav2 probe; physical future is validation-only.",
        "preregistration_commit": "c956f06",
        "evaluation_commit": "fcc61b5",
        "prior_manifest_sha256": {side: digest for side, (_, digest)
                                  in PRIOR_MANIFESTS.items()},
        "image_id": summary["image_id"],
        "rows": len(summary["rows"]),
        "files": files}
    (output / "manifest.json").write_text(json.dumps(report, indent=2,
                                               sort_keys=True) + "\n")
    print(json.dumps({"files": len(files), "rows": report["rows"]}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--work", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.root, args.work, args.output)
