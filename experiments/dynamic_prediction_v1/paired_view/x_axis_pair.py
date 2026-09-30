#!/usr/bin/env python3
"""Prepare and run each fixed static viewpoint once with an X-axis slider."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import one_pair

ROOT = Path(__file__).resolve().parents[3]
WORK = ROOT / "build"
HERE = Path(__file__).resolve().parent
VIEWS = {"south": (4.9, -1.8), "north": (4.9, 1.8)}
EXTRA = {"axis_runner": HERE / "x_axis_pair.py"}


def derive_model(source):
    old, new = "<xyz>0 1 0</xyz>", "<xyz>1 0 0</xyz>"
    if source.count(old) != 1:
        raise ValueError("slider joint axis source changed")
    result = source.replace(old, new)
    if result.replace(new, old) != source:
        raise ValueError("derived slider model changed beyond joint axis")
    root = ET.fromstring(result)
    axis = root.find(".//joint[@name='slider_joint']/axis/xyz")
    if axis is None or axis.text != "1 0 0":
        raise ValueError("derived model axis invalid")
    return result


def source_check():
    one_pair.current_source_check()
    for name, path in EXTRA.items():
        if not path.is_file():
            raise ValueError(f"missing source: {name}")


def prepare(series):
    if series.exists():
        raise FileExistsError(series)
    if not series.is_relative_to(WORK) or series == WORK:
        raise ValueError("new series must be inside build/")
    source_check()
    world_source = one_pair.INPUTS["source_world"].read_text()
    model = derive_model(one_pair.INPUTS["moving_model"].read_text())
    worlds = {side: one_pair.render_world(world_source, xy) for side, xy in VIEWS.items()}
    inputs = {**one_pair.INPUTS, **EXTRA}
    plan = {"schema": "rm_dynamic_prediction_x_axis_pair/v1",
            "scope": "One attempt per south/north static simulation-only viewpoint. No Nav2, goals, robot commands, network, or hardware.",
            "branch": one_pair.BRANCH, "run_commit": one_pair.git("rev-parse", "HEAD"),
            "compiled_source_commit": one_pair.COMPILED_SOURCE_COMMIT,
            "image_tag": one_pair.IMAGE_TAG, "image_id": one_pair.IMAGE_ID,
            "robot_pose_xy": {side: list(xy) for side, xy in VIEWS.items()},
            "moving_box_amplitude_m": .9, "moving_box_period_s": 8.0,
            "stop_sim_s": 44.0, "no_repeat_for_favorable_observation": True,
            "accepted_for_deployment": False,
            "file_sha256": {name: one_pair.sha(path) for name, path in inputs.items()},
            "derived_model_sha256": hashlib.sha256(model.encode()).hexdigest(),
            "derived_world_sha256": {side: hashlib.sha256(text.encode()).hexdigest()
                                     for side, text in worlds.items()}}
    (series / "worlds").mkdir(parents=True)
    (series / "models").mkdir()
    (series / "models/x_axis_obstacle.sdf").write_text(model)
    for side, text in worlds.items():
        (series / "worlds" / f"{side}.sdf").write_text(text)
        (series / side).mkdir()
    (series / "plan.json").write_text(json.dumps(plan, indent=2,
                                                sort_keys=True) + "\n")
    print(json.dumps({"series": str(series), "run_commit": plan["run_commit"],
                      "derived_model_sha256": plan["derived_model_sha256"]}))


def run(series, side):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / side
    if (trial / "container_id").exists() or (trial / "docker_exit.txt").exists():
        raise FileExistsError("view already attempted; refusing repeat")
    source_check()
    if one_pair.git("rev-parse", "HEAD") != plan["run_commit"]:
        raise ValueError("HEAD changed after plan freeze")
    if plan["image_id"] != one_pair.IMAGE_ID:
        raise ValueError("image plan changed")
    for name, path in {**one_pair.INPUTS, **EXTRA}.items():
        if one_pair.sha(path) != plan["file_sha256"][name]:
            raise ValueError(f"source changed after plan: {name}")
    world = series / "worlds" / f"{side}.sdf"
    model = series / "models/x_axis_obstacle.sdf"
    if one_pair.sha(world) != plan["derived_world_sha256"][side] or \
            one_pair.sha(model) != plan["derived_model_sha256"] or \
            model.read_text() != derive_model(one_pair.INPUTS["moving_model"].read_text()):
        raise ValueError("derived fixture changed")
    (WORK / "tmp").mkdir(exist_ok=True)
    mount_trial = "/work/" + str(trial.relative_to(WORK))
    mount_world = "/work/" + str(world.relative_to(WORK))
    mount_model = "/work/" + str(model.relative_to(WORK))
    args = ["docker", "run", "--rm", "--init", "--network", "none",
            "--cpus", "2", "--memory", "6g", "--memory-swap", "8g",
            "--pids-limit", "1024", "--security-opt", "no-new-privileges",
            "--cap-drop", "ALL", "--user", f"{os.getuid()}:{os.getgid()}",
            "--entrypoint", "bash", "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
            "--cidfile", str(trial / "container_id")]
    for value in ("HOME=/work/tmp", "ROS_DOMAIN_ID=175", "ROS_LOCALHOST_ONLY=1",
                  "PYTHONDONTWRITEBYTECODE=1", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
                  "TMPDIR=/work/tmp", "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                  f"IGN_PARTITION=dynamic_prediction_x_axis_20260930_{side}"):
        args.extend(("-e", value))
    args.extend((one_pair.IMAGE_TAG,
                 "/ws/experiments/dynamic_prediction_v1/paired_view/run_observation.sh",
                 mount_trial, mount_world, mount_model))
    with (trial / "docker_stdout.log").open("x") as log:
        status = subprocess.call(args, stdout=log, stderr=subprocess.STDOUT)
    (trial / "docker_exit.txt").write_text(str(status) + "\n")
    print(json.dumps({"side": side, "docker_exit": status, "trial": str(trial)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run"))
    parser.add_argument("series", type=Path)
    parser.add_argument("side", nargs="?", choices=tuple(VIEWS))
    args = parser.parse_args()
    if args.command == "prepare":
        if args.side is not None:
            parser.error("prepare does not accept side")
        prepare(args.series.resolve())
    else:
        if args.side is None:
            parser.error("run requires side")
        run(args.series.resolve(), args.side)
