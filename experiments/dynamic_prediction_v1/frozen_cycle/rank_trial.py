#!/usr/bin/env python3
"""Pre-register one phase-4 runtime sanity trial of uniform-center V1 ranking."""
import argparse
import json
import os
from pathlib import Path
import subprocess

import yaml

from one_trial import (ARCHIVE_PROFILE, IMAGE_TAG, PROFILE_SHA, ROOT, WORK,
                       image_id, manifest_check, sha)


HERE = Path(__file__).resolve().parent
RANK_KEY = "        horizon: 1.0\n"


def modified_profile(source):
    text = source.read_text()
    if text.count(RANK_KEY) != 1:
        raise ValueError("expected one V1 horizon anchor")
    updated = text.replace(RANK_KEY, RANK_KEY +
                           "        collision_rank_mode: uniform_center_overlap\n")
    original_tree, updated_tree = yaml.safe_load(text), yaml.safe_load(updated)
    follow = updated_tree["controller_server"]["ros__parameters"]["FollowPath"]
    if follow["PredictionV1Critic"].pop("collision_rank_mode") != "uniform_center_overlap" or \
            updated_tree != original_tree:
        raise ValueError("profile change exceeded the single rank-mode parameter")
    return updated


def binary_paths():
    return {
        "nav2_mppi_controller": WORK / "dynamic_prediction_trace_v3/install/nav2_mppi_controller/lib/libmppi_controller.so",
        "nav2_mppi_critics": WORK / "dynamic_prediction_trace_v3/install/nav2_mppi_controller/lib/libmppi_critics.so",
        "uniform_rank_plugin": WORK / "dynamic_prediction_rank_trace_v1/install/rm_dynamic_prediction_critic/lib/librm_dynamic_prediction_critic.so",
    }


def source_paths():
    return {
        "run_rank_trial.sh": HERE / "run_rank_trial.sh",
        "prediction_critic.cpp": ROOT / "experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp",
        "geometry.hpp": ROOT / "experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/include/rm_dynamic_prediction_critic/geometry.hpp",
        "docker/Dockerfile.humble": ROOT / "docker/Dockerfile.humble",
        "trace_source_manifest": WORK / "dynamic_prediction_trace_sources_v3/trace_sources.json",
        "rank_trace_source_manifest": WORK / "dynamic_prediction_trace_sources_rank_v1/trace_sources.json",
    }


def prepare(archive, series):
    branch = subprocess.check_output(["git", "branch", "--show-current"],
                                     cwd=ROOT, text=True).strip()
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"],
                                     cwd=ROOT, text=True).strip()
    if branch != "experiment/dynamic-prediction-v1":
        raise ValueError(f"wrong branch: {branch}")
    dirty = subprocess.check_output(["git", "status", "--porcelain"],
                                    cwd=ROOT, text=True)
    if dirty:
        raise ValueError("commit source and trial tooling before pre-registration")
    if manifest_check(archive) != 52 or sha(archive / ARCHIVE_PROFILE) != PROFILE_SHA:
        raise ValueError("historical input changed")
    source = archive / ARCHIVE_PROFILE
    profile = modified_profile(source)
    if series.exists():
        raise FileExistsError("series exists; refusing to overwrite")
    trial = series / "candidate_navfn_1"
    trial.mkdir(parents=True)
    (trial / "profile.yaml").write_text(profile)
    paths = {**source_paths(), **binary_paths()}
    plan = {
        "schema": "rm_dynamic_prediction_uniform_rank_trial/v1",
        "scope": "One phase-4 runtime integration trial. Different phase from frozen phase-0 evidence; not a paired or multi-phase safety claim.",
        "source_branch": branch, "source_commit": commit,
        "image_tag": IMAGE_TAG, "image_id": image_id(),
        "historical_profile_sha256": PROFILE_SHA,
        "derived_profile_sha256": sha(trial / "profile.yaml"),
        "only_profile_change": "FollowPath.PredictionV1Critic.collision_rank_mode=uniform_center_overlap",
        "phase_s": 4, "batch_size": 300,
        "no_repeat_for_favorable_phase": True,
        "accepted_for_deployment": False,
        "file_sha256": {name: sha(path) for name, path in paths.items()},
    }
    (series / "plan.json").write_text(json.dumps(plan, indent=2) + "\n")
    print(json.dumps({"series": str(series), "commit": commit,
                      "profile_sha256": plan["derived_profile_sha256"],
                      "image_id": plan["image_id"]}, indent=2))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / "candidate_navfn_1"
    if (trial / "docker_exit.txt").exists() or (trial / "container_id").exists():
        raise FileExistsError("trial already started; refusing to repeat")
    if image_id() != plan["image_id"] or \
            sha(trial / "profile.yaml") != plan["derived_profile_sha256"]:
        raise ValueError("image or profile changed after pre-registration")
    for name, path in {**source_paths(), **binary_paths()}.items():
        if sha(path) != plan["file_sha256"][name]:
            raise ValueError(f"frozen input changed: {name}")
    target = "/work/" + str(trial.relative_to(WORK))
    (WORK / "tmp").mkdir(exist_ok=True)
    args = ["docker", "run", "--rm", "--init", "--network", "none", "--user",
            f"{os.getuid()}:{os.getgid()}", "--entrypoint", "bash",
            "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
            "--cidfile", str(trial / "container_id")]
    for value in ("ROS_DOMAIN_ID=174", "ROS_LOCALHOST_ONLY=1", "PYTHONDONTWRITEBYTECODE=1",
                  "TMPDIR=/work/tmp", "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                  "TDT_PHASE_SECONDS=4", "IGN_PARTITION=dynamic_prediction_rank_phase4_02"):
        args.extend(("-e", value))
    args.extend((IMAGE_TAG, "/ws/experiments/dynamic_prediction_v1/frozen_cycle/run_rank_trial.sh",
                 target, target + "/profile.yaml"))
    with (trial / "docker_stdout.log").open("x") as out:
        status = subprocess.call(args, stdout=out, stderr=subprocess.STDOUT)
    (trial / "docker_exit.txt").write_text(str(status) + "\n")
    print(json.dumps({"docker_exit": status, "trial": str(trial)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run"))
    parser.add_argument("series", type=Path)
    parser.add_argument("--archive", type=Path, default=Path("/home/wpie/tdt_p2b"))
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.archive, args.series)
    else:
        run(args.series)
