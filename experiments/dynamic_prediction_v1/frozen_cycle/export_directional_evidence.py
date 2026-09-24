#!/usr/bin/env python3
"""Preserve just the inputs needed to replay the directional oracle probe."""
import argparse
import json
from pathlib import Path
import tarfile
from tempfile import TemporaryDirectory

from analyze import sha
from directional_support_probe import probe


def export(trial, output):
    trial, output = Path(trial), Path(output)
    summary_file = output / "summary.json"
    summary = json.loads(summary_file.read_text())
    if summary["source_sha256"]["probe_script"] != sha(
            Path(__file__).with_name("directional_support_probe.py")):
        raise ValueError("probe script changed after producing summary")
    archive = output / "replay_inputs.tar.gz"
    manifest_file = output / "manifest.json"
    if archive.exists() or manifest_file.exists():
        raise FileExistsError("evidence export already exists")
    series_name = trial.parent.name
    archive_root = Path(series_name)
    sources = [
        (trial.parent / "plan.json", archive_root / "plan.json"),
        (trial / "profile.yaml", archive_root / trial.name / "profile.yaml"),
        (trial / "runtime_audit.json", archive_root / trial.name / "runtime_audit.json"),
        (trial / "gazebo_poses.jsonl", archive_root / trial.name / "gazebo_poses.jsonl"),
    ]
    cycles = sorted((trial / "mppi_cycles").glob("cycle_*.json"),
                    key=lambda path: int(path.stem.split("_")[1]))
    audit = json.loads((trial / "runtime_audit.json").read_text())
    accepted = sum(group["cycles"] for group in summary["residual_by_side"].values())
    if len(cycles) != audit["writer_status"]["written"] or \
            accepted != audit["prediction_status_counts"]["accepted"]:
        raise ValueError("cycle count changed after directional analysis")
    sources.extend((path, archive_root / trial.name / "mppi_cycles" / path.name)
                   for path in cycles)
    selected_id = summary["first_near_goal_cycle"]["cycle_id"]
    for cycle_id in range(selected_id - 4, selected_id + 1):
        path = trial / "mppi_cycles" / f"cycle_{cycle_id}.bin"
        sources.append((path, archive_root / trial.name / "mppi_cycles" / path.name))
    with tarfile.open(archive, "x:gz", compresslevel=9) as bundle:
        for source, name in sources:
            if not source.is_file():
                raise FileNotFoundError(source)
            bundle.add(source, arcname=str(name), recursive=False)
    with tarfile.open(archive, "r:gz") as bundle:
        names = bundle.getnames()
    if names != [str(name) for _, name in sources]:
        raise ValueError("archive members differ from planned inputs")
    with TemporaryDirectory() as directory:
        with tarfile.open(archive, "r:gz") as bundle:
            bundle.extractall(directory, filter="data")
        reproduced = probe(Path(directory) / archive_root / trial.name)
        original = dict(summary)
        original.pop("trial")
        reproduced.pop("trial")
        if reproduced != original:
            raise ValueError("replay from exported inputs changed probe results")
    root = Path(__file__).resolve().parents[3]
    code = ("experiments/dynamic_prediction_v1/frozen_cycle/analyze.py",
            "experiments/dynamic_prediction_v1/frozen_cycle/replay_ranking.py",
            "experiments/dynamic_prediction_v1/frozen_cycle/directional_support_probe.py",
            "experiments/dynamic_prediction_v1/frozen_cycle/export_directional_evidence.py")
    manifest = {
        "schema": "rm_dynamic_prediction_directional_oracle_export/v1",
        "scope": "All 902 cycle metadata files and full Gazebo truth; binary tensors only for selected cycle 263 and the four preceding filter-history cycles. Enough to rerun the directional support probe, not the entire closed-loop trace.",
        "archive_root": str(archive_root),
        "archive_member_count": len(sources),
        "all_cycle_metadata_count": len(cycles),
        "selected_and_predecessor_binary_count": 5,
        "replayed_from_export": True,
        "summary_sha256": sha(summary_file),
        "replay_inputs_sha256": sha(archive),
        "code_sha256": {name: sha(root / name) for name in code},
    }
    manifest_file.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    print(json.dumps(export(args.trial, args.output), indent=2))
