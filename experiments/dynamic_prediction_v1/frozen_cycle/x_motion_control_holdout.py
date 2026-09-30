#!/usr/bin/env python3
"""One-shot phase-6 Navfn+V1 control capture with X-axis moving box."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import phase2_holdout as base

HERE = Path(__file__).resolve().parent
ROOT = base.ROOT
WORK = base.WORK
sys.path.insert(0, str(ROOT / "experiments/dynamic_prediction_v1/paired_view"))
from x_axis_pair import derive_model  # noqa: E402

MODEL = ROOT / "src/rm_simulation/models/moving_obstacle.sdf"
LAUNCH = ROOT / "docs/tdt_migration/evidence/dynamic_reference_20260922/dynamic.launch.py"
RUN = HERE / "run_phase2_holdout.sh"
SELF = HERE / "x_motion_control_holdout.py"
MODEL_ANCHOR = "/ws/src/rm_simulation/models/moving_obstacle.sdf"
LAUNCH_ANCHOR = "/ws/docs/tdt_migration/evidence/dynamic_reference_20260922/dynamic.launch.py"


def mounted(path):
    relative = path.relative_to(WORK)
    if not re.fullmatch(r"[A-Za-z0-9_./-]+", str(relative)):
        raise ValueError("unsupported experiment path")
    return "/work/" + str(relative)


def derive(series):
    model = derive_model(MODEL.read_text())
    original_launch = LAUNCH.read_text()
    if original_launch.count(MODEL_ANCHOR) != 1:
        raise ValueError("source launch model anchor changed")
    launch = original_launch.replace(MODEL_ANCHOR,
                                     mounted(series / "models/x_axis_obstacle.sdf"))
    original_run = RUN.read_text()
    if original_run.count(LAUNCH_ANCHOR) != 1:
        raise ValueError("source runner launch anchor changed")
    runner = original_run.replace(LAUNCH_ANCHOR,
                                  mounted(series / "dynamic_x.launch.py"))
    if launch.replace(mounted(series / "models/x_axis_obstacle.sdf"),
                      MODEL_ANCHOR) != original_launch or \
            runner.replace(mounted(series / "dynamic_x.launch.py"),
                           LAUNCH_ANCHOR) != original_run:
        raise ValueError("derived files differ beyond model/launch path")
    return model, launch, runner


def prepare(archive, series):
    if series.exists() or not series.is_relative_to(WORK) or series == WORK:
        raise ValueError("use one new series under build/")
    if base.git("branch", "--show-current") != base.BRANCH or \
            base.git("status", "--porcelain") or \
            subprocess.call(["git", "merge-base", "--is-ancestor",
                             base.SOURCE_COMMIT, "HEAD"], cwd=ROOT):
        raise ValueError("wrong branch, dirty HEAD, or compiled source not ancestor")
    if base.sha(archive / base.ARCHIVE_PROFILE) != base.PROFILE_SHA:
        raise ValueError("frozen Navfn profile changed")
    matched = base.historical_manifest_check(archive)
    source_count = base.source_check()
    image = base.image_id()
    if image != "sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6":
        raise ValueError("pinned Docker image changed")
    model, launch, runner = derive(series)
    (series / "models").mkdir(parents=True)
    (series / "candidate_navfn_1").mkdir()
    (series / "models/x_axis_obstacle.sdf").write_text(model)
    (series / "dynamic_x.launch.py").write_text(launch)
    (series / "run_x.sh").write_text(runner)
    shutil.copy2(archive / base.ARCHIVE_PROFILE,
                 series / "candidate_navfn_1/profile.yaml")
    source_files = {**base.FILES, "moving_model": MODEL,
                    "dynamic.launch.py": LAUNCH, "x_motion_control_holdout.py": SELF}
    plan = {
        "schema": "rm_dynamic_prediction_x_motion_control_holdout/v1",
        "scope": "One new simulation-only X-slider control capture at phase 6; Navfn+V1 profile and running nodes unchanged.",
        "branch": base.BRANCH, "run_commit": base.git("rev-parse", "HEAD"),
        "compiled_source_commit": base.SOURCE_COMMIT,
        "image_tag": base.IMAGE_TAG, "image_id": image,
        "historical_candidate_manifest_entries_verified": matched,
        "instrumented_source_files_verified": source_count,
        "profile_sha256": base.PROFILE_SHA,
        "moving_axis": "x", "phase_s": 6, "batch_size": 300,
        "selection_rule": "First sampled physical body-to-box gap <0.05 m, else sampled minimum; latest complete accepted cycle >=0.25 s before event.",
        "no_repeat_for_favorable_result": True,
        "accepted_for_deployment": False,
        "file_sha256": {name: base.sha(path) for name, path in source_files.items()},
        "derived_sha256": {"model": base.sha(series / "models/x_axis_obstacle.sdf"),
                           "launch": base.sha(series / "dynamic_x.launch.py"),
                           "runner": base.sha(series / "run_x.sh")}}
    (series / "plan.json").write_text(json.dumps(plan, indent=2,
                                                sort_keys=True) + "\n")
    print(json.dumps({"series": str(series), "run_commit": plan["run_commit"],
                      "image_id": image, "derived_sha256": plan["derived_sha256"]}))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / "candidate_navfn_1"
    if (trial / "container_id").exists() or (trial / "docker_exit.txt").exists():
        raise FileExistsError("trial already attempted")
    if base.git("rev-parse", "HEAD") != plan["run_commit"] or \
            base.git("status", "--porcelain") or base.image_id() != plan["image_id"]:
        raise ValueError("HEAD, image or working tree changed after freeze")
    base.source_check()
    source_files = {**base.FILES, "moving_model": MODEL,
                    "dynamic.launch.py": LAUNCH, "x_motion_control_holdout.py": SELF}
    for name, path in source_files.items():
        if base.sha(path) != plan["file_sha256"][name]:
            raise ValueError(f"source or binary changed: {name}")
    for name, path in (("model", series / "models/x_axis_obstacle.sdf"),
                       ("launch", series / "dynamic_x.launch.py"),
                       ("runner", series / "run_x.sh")):
        if base.sha(path) != plan["derived_sha256"][name]:
            raise ValueError(f"derived fixture changed: {name}")
    if base.sha(trial / "profile.yaml") != plan["profile_sha256"]:
        raise ValueError("Navfn profile changed")
    (WORK / "tmp").mkdir(exist_ok=True)
    args = ["docker", "run", "--rm", "--init", "--network", "none",
            "--cpus", "2", "--memory", "6g", "--memory-swap", "8g",
            "--pids-limit", "1024", "--security-opt", "no-new-privileges",
            "--cap-drop", "ALL", "--user", f"{os.getuid()}:{os.getgid()}",
            "--entrypoint", "bash", "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
            "--cidfile", str(trial / "container_id")]
    for value in ("HOME=/work/tmp", "ROS_DOMAIN_ID=174", "ROS_LOCALHOST_ONLY=1",
                  "PYTHONDONTWRITEBYTECODE=1", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
                  "TMPDIR=/work/tmp", "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                  "TDT_PHASE_SECONDS=6", "IGN_PARTITION=dynamic_prediction_x_motion_control_20260930"):
        args.extend(("-e", value))
    args.extend((base.IMAGE_TAG, mounted(series / "run_x.sh"),
                 mounted(trial), mounted(trial / "profile.yaml")))
    with (trial / "docker_stdout.log").open("x") as stream:
        status = subprocess.call(args, stdout=stream, stderr=subprocess.STDOUT)
    (trial / "docker_exit.txt").write_text(str(status) + "\n")
    print(json.dumps({"docker_exit": status, "trial": str(trial)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run"))
    parser.add_argument("series", type=Path)
    parser.add_argument("--archive", type=Path, default=Path("/home/wpie/tdt_p2b"))
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.archive.resolve(), args.series.resolve())
    else:
        run(args.series.resolve())
