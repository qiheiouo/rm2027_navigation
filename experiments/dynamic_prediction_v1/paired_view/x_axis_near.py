#!/usr/bin/env python3
"""One-shot safe stationary near-view X-slider observation, ending at 45 s."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import one_pair
from x_axis_pair import derive_model

ROOT = Path(__file__).resolve().parents[3]
WORK = ROOT / "build"
HERE = Path(__file__).resolve().parent
ROBOT_XY = (5.25, 1.10)
EXTRA = {"near_runner": HERE / "x_axis_near.py"}


def inputs():
    return {**one_pair.INPUTS, **EXTRA}


def check():
    one_pair.current_source_check()
    if one_pair.git("status", "--porcelain"):
        raise ValueError("dirty worktree")


def prepare(series):
    if series.exists():
        raise FileExistsError(series)
    if not series.is_relative_to(WORK) or series == WORK:
        raise ValueError("series must be new and under build/")
    check()
    world = one_pair.render_world(one_pair.INPUTS["source_world"].read_text(), ROBOT_XY)
    model = derive_model(one_pair.INPUTS["moving_model"].read_text())
    plan = {"schema": "rm_dynamic_prediction_x_axis_near/v1",
            "scope": "One north near-view stationary simulation, no Nav2 or robot commands",
            "branch": one_pair.BRANCH, "run_commit": one_pair.git("rev-parse", "HEAD"),
            "compiled_source_commit": one_pair.COMPILED_SOURCE_COMMIT,
            "image_tag": one_pair.IMAGE_TAG, "image_id": one_pair.IMAGE_ID,
            "robot_pose_xy": list(ROBOT_XY), "stop_sim_s": 45.0,
            "source_window_s": [16.0, 43.9], "moving_amplitude_m": .9,
            "moving_period_s": 8., "no_repeat_for_favorable_observation": True,
            "accepted_for_deployment": False,
            "file_sha256": {name: one_pair.sha(path) for name, path in inputs().items()},
            "world_sha256": hashlib.sha256(world.encode()).hexdigest(),
            "model_sha256": hashlib.sha256(model.encode()).hexdigest()}
    (series / "worlds").mkdir(parents=True)
    (series / "models").mkdir()
    (series / "north").mkdir()
    (series / "worlds/north.sdf").write_text(world)
    (series / "models/x_axis_obstacle.sdf").write_text(model)
    (series / "plan.json").write_text(json.dumps(plan, indent=2,
                                                sort_keys=True) + "\n")
    print(json.dumps({"series": str(series), "run_commit": plan["run_commit"]}))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / "north"
    if (trial / "container_id").exists() or (trial / "docker_exit.txt").exists():
        raise FileExistsError("view already attempted; refusing repeat")
    check()
    if one_pair.git("rev-parse", "HEAD") != plan["run_commit"]:
        raise ValueError("HEAD changed after plan freeze")
    for name, path in inputs().items():
        if one_pair.sha(path) != plan["file_sha256"][name]:
            raise ValueError(f"frozen source changed: {name}")
    world, model = series / "worlds/north.sdf", series / "models/x_axis_obstacle.sdf"
    if one_pair.sha(world) != plan["world_sha256"] or \
            one_pair.sha(model) != plan["model_sha256"]:
        raise ValueError("derived fixture changed")
    (WORK / "tmp").mkdir(exist_ok=True)
    def mount(path):
        return "/work/" + str(path.relative_to(WORK))
    args = ["docker", "run", "--rm", "--init", "--network", "none",
            "--cpus", "2", "--memory", "6g", "--memory-swap", "8g",
            "--pids-limit", "1024", "--security-opt", "no-new-privileges",
            "--cap-drop", "ALL", "--user", f"{os.getuid()}:{os.getgid()}",
            "--entrypoint", "bash", "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
            "--cidfile", str(trial / "container_id")]
    for value in ("HOME=/work/tmp", "ROS_DOMAIN_ID=176", "ROS_LOCALHOST_ONLY=1",
                  "PYTHONDONTWRITEBYTECODE=1", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
                  "TMPDIR=/work/tmp", "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                  "IGN_PARTITION=dynamic_prediction_x_axis_near_20260930"):
        args.extend(("-e", value))
    args.extend((one_pair.IMAGE_TAG,
                 "/ws/experiments/dynamic_prediction_v1/paired_view/run_observation.sh",
                 mount(trial), mount(world), mount(model), "45.0"))
    with (trial / "docker_stdout.log").open("x") as log:
        status = subprocess.call(args, stdout=log, stderr=subprocess.STDOUT)
    (trial / "docker_exit.txt").write_text(str(status) + "\n")
    print(json.dumps({"docker_exit": status, "trial": str(trial)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run"))
    parser.add_argument("series", type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.series.resolve())
    else:
        run(args.series.resolve())
