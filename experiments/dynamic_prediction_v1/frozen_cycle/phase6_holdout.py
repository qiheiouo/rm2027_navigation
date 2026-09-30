#!/usr/bin/env python3
"""One-shot phase-6 Navfn+V1 holdout with the existing pinned runtime."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

import phase2_holdout as base

ROOT = Path(__file__).resolve().parents[3]
WORK = ROOT / "build"
HERE = Path(__file__).resolve().parent


def prepare(archive, series):
    if series.exists():
        raise FileExistsError("phase-6 series already exists")
    if not series.is_relative_to(WORK) or series == WORK:
        raise ValueError("series must be a new build/ directory")
    if base.git("branch", "--show-current") != base.BRANCH or \
            base.git("status", "--porcelain") or not base.git(
                "merge-base", "--is-ancestor", base.SOURCE_COMMIT, "HEAD") == "":
        raise ValueError("wrong branch, dirty source, or compiled HEAD unavailable")
    profile = archive / base.ARCHIVE_PROFILE
    if base.sha(profile) != base.PROFILE_SHA:
        raise ValueError("original Navfn profile changed")
    matched = base.historical_manifest_check(archive)
    sources = base.source_check()
    plan = {"schema": "rm_dynamic_prediction_phase6_holdout/v1",
            "scope": "One fresh simulation-only instrumented Navfn+V1 phase-6 capture, no runtime algorithm or safety changes.",
            "branch": base.BRANCH, "run_commit": base.git("rev-parse", "HEAD"),
            "compiled_source_commit": base.SOURCE_COMMIT,
            "image_tag": base.IMAGE_TAG, "image_id": base.image_id(),
            "historical_candidate_manifest_entries_verified": matched,
            "instrumented_source_files_verified": sources,
            "historical_profile_sha256": base.PROFILE_SHA,
            "phase_s": 6, "batch_size": 300,
            "selection_rule": (
                "First sampled physical octagon-to-box clearance <0.05 m; otherwise sampled minimum. "
                "Select latest complete accepted MPPI cycle at least 0.25 s earlier. Keep one attempt regardless of result."),
            "no_repeat_for_favorable_phase": True,
            "accepted_for_deployment": False,
            "file_sha256": {**{name: base.sha(path) for name, path in base.FILES.items()},
                            "phase6_holdout.py": base.sha(HERE / "phase6_holdout.py")}}
    trial = series / "candidate_navfn_1"
    trial.mkdir(parents=True)
    shutil.copy2(profile, trial / "profile.yaml")
    (series / "plan.json").write_text(json.dumps(plan, indent=2,
                                               sort_keys=True) + "\n")
    print(json.dumps({"series": str(series), "run_commit": plan["run_commit"],
                      "image_id": plan["image_id"], "phase_s": 6}))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / "candidate_navfn_1"
    if (trial / "docker_exit.txt").exists() or (trial / "container_id").exists():
        raise FileExistsError("phase-6 attempt already started")
    if base.git("rev-parse", "HEAD") != plan["run_commit"] or \
            base.git("status", "--porcelain"):
        raise ValueError("source changed after preregistration")
    if base.image_id() != plan["image_id"] or \
            base.sha(trial / "profile.yaml") != plan["historical_profile_sha256"]:
        raise ValueError("image or profile changed")
    base.source_check()
    for name, path in {**base.FILES, "phase6_holdout.py": HERE / "phase6_holdout.py"}.items():
        if base.sha(path) != plan["file_sha256"][name]:
            raise ValueError(f"frozen input changed: {name}")
    target = "/work/" + str(trial.relative_to(WORK))
    (WORK / "tmp").mkdir(exist_ok=True)
    args = ["docker", "run", "--rm", "--init", "--network", "none",
            "--cpus", "2", "--memory", "6g", "--memory-swap", "8g",
            "--pids-limit", "1024", "--security-opt", "no-new-privileges",
            "--cap-drop", "ALL", "--user", f"{os.getuid()}:{os.getgid()}",
            "--entrypoint", "bash", "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
            "--cidfile", str(trial / "container_id")]
    for value in ("HOME=/work/tmp", "ROS_DOMAIN_ID=178", "ROS_LOCALHOST_ONLY=1",
                  "PYTHONDONTWRITEBYTECODE=1", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
                  "TMPDIR=/work/tmp", "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                  "TDT_PHASE_SECONDS=6", "IGN_PARTITION=dynamic_prediction_phase6_holdout_20260930"):
        args.extend(("-e", value))
    args.extend((base.IMAGE_TAG,
                 "/ws/experiments/dynamic_prediction_v1/frozen_cycle/run_phase2_holdout.sh",
                 target, target + "/profile.yaml"))
    with (trial / "docker_stdout.log").open("x") as stream:
        status = subprocess.call(args, stdout=stream, stderr=subprocess.STDOUT)
    (trial / "docker_exit.txt").write_text(str(status) + "\n")
    print(json.dumps({"docker_exit": status, "trial": str(trial)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run"))
    parser.add_argument("series", type=Path)
    parser.add_argument("--archive", type=Path,
                        default=Path("/home/wpie/tdt_p2b"))
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.archive.resolve(), args.series.resolve())
    else:
        run(args.series.resolve())
