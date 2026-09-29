#!/usr/bin/env python3
"""Pre-register two static viewpoints, then run each once in isolated Gazebo."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
WORK = ROOT / "build"
IMAGE_TAG = "rm2027_navigation:dynamic-prediction-20260924"
IMAGE_ID = "sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6"
COMPILED_SOURCE_COMMIT = "148a3a59e0f7936a5c06bd65a8ec2309306c0aef"
BRANCH = "codex/dynamic-differential-risk"
Y = {"south": -1.65, "north": 1.65}
INPUTS = {
    "source_world": ROOT / "src/rm_simulation/worlds/phase1_omni.sdf",
    "moving_model": ROOT / "src/rm_simulation/models/moving_obstacle.sdf",
    "wall_model": ROOT / "src/rm_simulation/models/course_wall.sdf",
    "tracker_config": ROOT / "src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml",
    "observation.launch.py": HERE / "observation.launch.py",
    "record_until.py": HERE / "record_until.py",
    "run_observation.sh": HERE / "run_observation.sh",
    "tracker_binary": WORK / "dynamic_prediction_runtime_head_20260929/install/rm_dynamic_obstacle_tracking/lib/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node",
    "simulation_binary": WORK / "dynamic_prediction_runtime_head_20260929/install/rm_simulation/lib/rm_simulation/moving_obstacle_controller",
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def image_id():
    return subprocess.check_output(
        ["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE_TAG],
        text=True).strip()


def current_source_check():
    if git("branch", "--show-current") != BRANCH or git("status", "--porcelain"):
        raise ValueError("wrong branch or dirty tracked worktree")
    if subprocess.call(["git", "diff", "--quiet", COMPILED_SOURCE_COMMIT, "HEAD", "--",
                        "src/", "experiments/tdt_planner/",
                        "experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/"],
                       cwd=ROOT):
        raise ValueError("runtime sources changed after pinned build")
    previous_plan = json.loads((WORK / "dynamic_prediction_phase2_holdout_20260929_setupfix/plan.json").read_text())
    for name, key in (("tracker_binary", "shadow_tracker"),
                      ("simulation_binary", "simulation_moving_obstacle_controller")):
        if sha(INPUTS[name]) != previous_plan["file_sha256"][key]:
            raise ValueError(f"rebuilt binary changed: {name}")
    if image_id() != IMAGE_ID:
        raise ValueError("Docker image changed")


def render_world(source, side):
    old = ('<model name="rm_sentry_2027" xmlns:ignition="http://ignitionrobotics.org/schema">\n'
           '      <pose>0 0 0 0 0 0</pose>')
    new = old.replace('<pose>0 0 0 0 0 0</pose>',
                      f'<pose>4.9 {Y[side]:.2f} 0 0 0 0</pose>')
    if source.count(old) != 1:
        raise ValueError("expected one original robot model and pose")
    result = source.replace(old, new)
    if result.replace(new, old) != source:
        raise ValueError("world changed beyond robot pose")
    root = ET.fromstring(result)
    robot = root.find(".//model[@name='rm_sentry_2027']")
    if robot is None or [float(x) for x in robot.find("pose").text.split()][:2] != [4.9, Y[side]]:
        raise ValueError("derived robot pose invalid")
    return result


def prepare(series):
    if series.exists():
        raise FileExistsError("pair already exists")
    if not series.is_relative_to(WORK) or series == WORK:
        raise ValueError("pair must be a new directory under this worktree build/")
    current_source_check()
    source = INPUTS["source_world"].read_text()
    derived = {side: render_world(source, side) for side in Y}
    plan = {"schema": "rm_dynamic_prediction_paired_view/v1",
            "scope": "Two stationary simulation-only viewpoints, no Nav2, no robot commands; each is attempted once regardless of observation outcome",
            "branch": BRANCH, "run_commit": git("rev-parse", "HEAD"),
            "compiled_source_commit": COMPILED_SOURCE_COMMIT,
            "image_tag": IMAGE_TAG, "image_id": IMAGE_ID,
            "robot_pose_xy": {side: [4.9, y] for side, y in Y.items()},
            "moving_box_amplitude_m": .9, "moving_box_period_s": 8.0,
            "stop_sim_s": 44.0, "no_repeat_for_favorable_observation": True,
            "accepted_for_deployment": False,
            "file_sha256": {name: sha(path) for name, path in INPUTS.items()},
            "derived_world_sha256": {side: hashlib.sha256(text.encode()).hexdigest()
                                     for side, text in derived.items()}}
    (series / "worlds").mkdir(parents=True)
    for side, text in derived.items():
        (series / "worlds" / f"{side}.sdf").write_text(text)
        (series / side).mkdir()
    (series / "plan.json").write_text(json.dumps(plan, indent=2) + "\n")
    print(json.dumps({"series": str(series), "run_commit": plan["run_commit"],
                      "world_hashes": plan["derived_world_sha256"]}, indent=2))


def run(series, side):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / side
    if (trial / "container_id").exists() or (trial / "docker_exit.txt").exists():
        raise FileExistsError("viewpoint already started; refusing repeat")
    current_source_check()
    if git("rev-parse", "HEAD") != plan["run_commit"]:
        raise ValueError("source commit changed after pre-registration")
    for name, path in INPUTS.items():
        if sha(path) != plan["file_sha256"][name]:
            raise ValueError(f"frozen input changed: {name}")
    world = series / "worlds" / f"{side}.sdf"
    if sha(world) != plan["derived_world_sha256"][side] or \
            world.read_text() != render_world(INPUTS["source_world"].read_text(), side):
        raise ValueError("derived world changed")
    (WORK / "tmp").mkdir(exist_ok=True)
    mount_trial = "/work/" + str(trial.relative_to(WORK))
    mount_world = "/work/" + str(world.relative_to(WORK))
    args = ["docker", "run", "--rm", "--init", "--network", "none",
            "--cpus", "2", "--memory", "6g", "--memory-swap", "8g",
            "--pids-limit", "1024", "--security-opt", "no-new-privileges",
            "--cap-drop", "ALL", "--user", f"{os.getuid()}:{os.getgid()}",
            "--entrypoint", "bash", "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
            "--cidfile", str(trial / "container_id")]
    for value in ("HOME=/work/tmp", "ROS_DOMAIN_ID=174", "ROS_LOCALHOST_ONLY=1",
                  "PYTHONDONTWRITEBYTECODE=1", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
                  "TMPDIR=/work/tmp", "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                  f"IGN_PARTITION=dynamic_prediction_paired_view_20260929_{side}"):
        args.extend(("-e", value))
    args.extend((IMAGE_TAG, "/ws/experiments/dynamic_prediction_v1/paired_view/run_observation.sh",
                 mount_trial, mount_world))
    with (trial / "docker_stdout.log").open("x") as log:
        status = subprocess.call(args, stdout=log, stderr=subprocess.STDOUT)
    (trial / "docker_exit.txt").write_text(str(status) + "\n")
    print(json.dumps({"side": side, "docker_exit": status, "trial": str(trial)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run"))
    parser.add_argument("series", type=Path)
    parser.add_argument("side", nargs="?", choices=tuple(Y))
    args = parser.parse_args()
    if args.command == "prepare":
        if args.side is not None:
            parser.error("prepare does not accept a side")
        prepare(args.series.resolve())
    else:
        if args.side is None:
            parser.error("run requires one side")
        run(args.series.resolve(), args.side)
