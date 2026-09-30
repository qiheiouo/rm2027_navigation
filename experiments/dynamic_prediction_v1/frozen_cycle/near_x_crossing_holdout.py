#!/usr/bin/env python3
"""One-shot near-field X-slider crossing in an isolated simulation fixture."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import phase2_holdout as base
import x_motion_control_holdout as previous

ROOT, WORK = base.ROOT, base.WORK
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "experiments/dynamic_prediction_v1/paired_view"))
from one_pair import render_world  # noqa: E402
from x_axis_pair import derive_model  # noqa: E402

START = (5.25, 1.10)
GOAL = (5.25, -1.10)
PHASE = 6
TAIL_S = 3.2
BASE_LAUNCH = ROOT / "src/rm_simulation/launch/phase1_5_gazebo.launch.py"
COMPARISON = ROOT / "experiments/tdt_planner/rm_tdt_planner/launch/simulation_comparison.launch.py"
REFERENCE = ROOT / "docs/tdt_migration/evidence/snapshot_revalidation_20260922/reference.launch.py"
SOURCE = {**base.FILES,
          "source_world": ROOT / "src/rm_simulation/worlds/phase1_omni.sdf",
          "source_model": previous.MODEL,
          "base_launch": BASE_LAUNCH,
          "comparison_launch": COMPARISON,
          "reference_launch": REFERENCE,
          "dynamic_launch": previous.LAUNCH,
          "record_physical_tail.py": HERE / "record_physical_tail.py",
          "near_x_crossing_holdout.py": HERE / "near_x_crossing_holdout.py"}


def swap_once(original, old, new):
    if original.count(old) != 1:
        raise ValueError(f"expected one derivation anchor: {old[:80]}")
    result = original.replace(old, new)
    if result.replace(new, old) != original:
        raise ValueError("derivation changed more than one intended anchor")
    return result


def derive(series):
    mount = previous.mounted
    world = render_world(SOURCE["source_world"].read_text(), START)
    model = derive_model(SOURCE["source_model"].read_text())
    base_text = BASE_LAUNCH.read_text()
    world_block = ('    world = PathJoinSubstitution([\n'
                   '        FindPackageShare("rm_simulation"),\n'
                   '        "worlds",\n'
                   '        "phase1_omni.sdf",\n'
                   '    ])')
    base_launch = swap_once(base_text, world_block,
                            f'    world = "{mount(series / "worlds/crossing.sdf")}"')
    comparison_block = ('PathJoinSubstitution([\n'
                        '                share, "launch", "phase1_5_gazebo.launch.py"])')
    comparison = swap_once(COMPARISON.read_text(), comparison_block,
                           f'"{mount(series / "base_gazebo.launch.py")}"')
    reference = swap_once(REFERENCE.read_text(),
                          '/ws/experiments/tdt_planner/rm_tdt_planner/launch/simulation_comparison.launch.py',
                          mount(series / "comparison.launch.py"))
    dynamic = swap_once(previous.LAUNCH.read_text(),
                        '/ws/docs/tdt_migration/evidence/snapshot_revalidation_20260922/reference.launch.py',
                        mount(series / "reference.launch.py"))
    dynamic = swap_once(dynamic, previous.MODEL_ANCHOR,
                        mount(series / "models/x_axis_obstacle.sdf"))
    run = swap_once(previous.RUN.read_text(), previous.LAUNCH_ANCHOR,
                    mount(series / "dynamic_near_x.launch.py"))
    run = swap_once(run, '--goal-x 5.6 --launch-log',
                    f'--goal-x {GOAL[0]:.2f} --goal-y {GOAL[1]:.2f} --launch-log')
    tail = ('set +e\n'
            f'python3 /ws/experiments/dynamic_prediction_v1/frozen_cycle/record_physical_tail.py '
            f'"$out/gazebo_poses.jsonl" {TAIL_S:.1f} > "$out/physical_tail.log" 2>&1\n'
            'tail_status=$?\nset -e\n'
            'printf \'%s\\n\' "$tail_status" > "$out/physical_tail_exit.txt"\n'
            'if [ "$tail_status" -ne 0 ]; then exit "$tail_status"; fi\n')
    run = swap_once(run, 'exit "$observer_status"\n', tail + 'exit "$observer_status"\n')
    return {"worlds/crossing.sdf": world,
            "models/x_axis_obstacle.sdf": model,
            "base_gazebo.launch.py": base_launch,
            "comparison.launch.py": comparison,
            "reference.launch.py": reference,
            "dynamic_near_x.launch.py": dynamic,
            "run_near_x.sh": run}


def prepare(archive, series):
    if series.exists() or not series.is_relative_to(WORK) or series == WORK:
        raise ValueError("use one new series under build/")
    if base.git("branch", "--show-current") != base.BRANCH or \
            base.git("status", "--porcelain") or \
            subprocess.call(["git", "merge-base", "--is-ancestor",
                             base.SOURCE_COMMIT, "HEAD"], cwd=ROOT):
        raise ValueError("wrong branch, dirty HEAD, or compiled source not ancestor")
    if base.sha(archive / base.ARCHIVE_PROFILE) != base.PROFILE_SHA:
        raise ValueError("Navfn profile changed")
    historical = base.historical_manifest_check(archive)
    instrumented = base.source_check()
    image = base.image_id()
    if image != "sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6":
        raise ValueError("pinned Docker image changed")
    generated = derive(series)
    for name, content in generated.items():
        path = series / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    trial = series / "candidate_navfn_1"
    trial.mkdir()
    shutil.copy2(archive / base.ARCHIVE_PROFILE, trial / "profile.yaml")
    plan = {
        "schema": "rm_dynamic_prediction_near_x_crossing_holdout/v1",
        "scope": "Single simulation-only X-slider near crossing with full physical future tail; runtime nodes and safety configuration unchanged.",
        "branch": base.BRANCH, "run_commit": base.git("rev-parse", "HEAD"),
        "compiled_source_commit": base.SOURCE_COMMIT,
        "image_tag": base.IMAGE_TAG, "image_id": image,
        "historical_manifest_entries_verified": historical,
        "instrumented_source_files_verified": instrumented,
        "profile_sha256": base.PROFILE_SHA,
        "robot_start_xy": START, "goal_xy": GOAL,
        "moving_axis": "x", "phase_s": PHASE,
        "physical_tail_sim_s": TAIL_S, "batch_size": 300,
        "selection_rule": "First sampled physical body-to-box gap <0.05 m, else sampled minimum; latest complete accepted cycle >=0.25 s before event; require 3.0 s future physical labels or report gate failure without selecting another cycle.",
        "no_repeat_for_favorable_result": True,
        "accepted_for_deployment": False,
        "source_sha256": {name: base.sha(path) for name, path in SOURCE.items()},
        "derived_sha256": {name: base.sha(series / name) for name in generated}}
    (series / "plan.json").write_text(json.dumps(plan, indent=2,
                                                sort_keys=True) + "\n")
    print(json.dumps({"series": str(series), "run_commit": plan["run_commit"],
                      "image_id": image, "derived_files": len(generated)}))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / "candidate_navfn_1"
    if (trial / "container_id").exists() or (trial / "docker_stdout.log").exists():
        raise FileExistsError("trial already attempted")
    if base.git("rev-parse", "HEAD") != plan["run_commit"] or \
            base.git("status", "--porcelain") or base.image_id() != plan["image_id"]:
        raise ValueError("HEAD, image or worktree changed after freeze")
    base.source_check()
    for name, path in SOURCE.items():
        if base.sha(path) != plan["source_sha256"][name]:
            raise ValueError(f"source or binary changed: {name}")
    for name, digest in plan["derived_sha256"].items():
        if base.sha(series / name) != digest:
            raise ValueError(f"derived fixture changed: {name}")
    if base.sha(trial / "profile.yaml") != plan["profile_sha256"]:
        raise ValueError("profile changed")
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
                  f"TDT_PHASE_SECONDS={PHASE}",
                  "IGN_PARTITION=dynamic_prediction_near_x_crossing_20260930"):
        args.extend(("-e", value))
    args.extend((base.IMAGE_TAG, previous.mounted(series / "run_near_x.sh"),
                 previous.mounted(trial), previous.mounted(trial / "profile.yaml")))
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
