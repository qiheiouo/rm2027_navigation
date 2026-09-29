#!/usr/bin/env python3
"""Pre-register and run one independent Navfn+V1 phase-2 simulation capture."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
WORK = ROOT / "build"
IMAGE_TAG = "rm2027_navigation:dynamic-prediction-20260924"
ARCHIVE_PROFILE = Path("runs/dynamic_prediction_navfn_transfer_v1/candidate_navfn_1/profile.yaml")
ARCHIVE_PREFIX = "build/tdt_p2b/runs/dynamic_prediction_navfn_transfer_v1/candidate_navfn_1/"
PROFILE_SHA = "d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640"
SOURCE_COMMIT = "148a3a59e0f7936a5c06bd65a8ec2309306c0aef"
BRANCH = "codex/dynamic-differential-risk"
BINARIES = {
    "nav2_mppi_controller": WORK / "dynamic_prediction_trace_head_20260929/install/nav2_mppi_controller/lib/libmppi_controller.so",
    "nav2_mppi_critics": WORK / "dynamic_prediction_trace_head_20260929/install/nav2_mppi_controller/lib/libmppi_critics.so",
    "prediction_v1_critic": WORK / "dynamic_prediction_trace_head_20260929/install/rm_dynamic_prediction_critic/lib/librm_dynamic_prediction_critic.so",
    "simulation_moving_obstacle_controller": WORK / "dynamic_prediction_runtime_head_20260929/install/rm_simulation/lib/rm_simulation/moving_obstacle_controller",
    "shadow_tracker": WORK / "dynamic_prediction_runtime_head_20260929/install/rm_dynamic_obstacle_tracking/lib/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node",
}
FILES = {
    "docker/Dockerfile.humble": ROOT / "docker/Dockerfile.humble",
    "trace_source_manifest": WORK / "dynamic_prediction_trace_sources_head_20260929/trace_sources.json",
    "run_phase2_holdout.sh": HERE / "run_phase2_holdout.sh",
    **BINARIES,
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def image_id():
    return subprocess.check_output(
        ["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE_TAG],
        text=True).strip()


def historical_manifest_check(archive):
    manifest = json.loads((ROOT / "docs/dynamic_navigation/evidence/v1_navfn_transfer_20260924/manifest.json").read_text())
    selected = {name: digest for name, digest in manifest["files_sha256"].items()
                if name.startswith(ARCHIVE_PREFIX)}
    if len(selected) != 52:
        raise ValueError("historical candidate manifest entry count changed")
    for name, digest in selected.items():
        if sha(archive / name.removeprefix("build/tdt_p2b/")) != digest:
            raise ValueError(f"historical evidence hash mismatch: {name}")
    return len(selected)


def source_check():
    trace = json.loads(FILES["trace_source_manifest"].read_text())
    if subprocess.call(["git", "diff", "--quiet", SOURCE_COMMIT, "HEAD", "--",
                        "src/", "experiments/tdt_planner/",
                        "experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/",
                        "experiments/dynamic_prediction_v1/frozen_cycle/prepare_trace.py"],
                       cwd=ROOT):
        raise ValueError("runtime sources changed after the pinned build")
    if trace["original_plugin_source_sha256"] != sha(ROOT / "experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp"):
        raise ValueError("original critic source changed")
    for name, digest in trace["instrumented_source_sha256"].items():
        path = WORK / "dynamic_prediction_trace_sources_head_20260929" / name
        if sha(path) != digest:
            raise ValueError(f"instrumented source changed: {name}")
    if trace["patch_script_sha256"] != sha(HERE / "prepare_trace.py"):
        raise ValueError("trace patch script changed")
    for name, digest in trace["observer_source_sha256"].items():
        path = ROOT / "docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922" / name
        if sha(path) != digest:
            raise ValueError(f"observer source changed: {name}")
    return len(trace["instrumented_source_sha256"])


def preregister(archive, series):
    if series.exists():
        raise FileExistsError("series exists; refusing to overwrite")
    if not series.is_relative_to(WORK) or series == WORK:
        raise ValueError("series must be a new directory under this worktree's build/")
    if git("branch", "--show-current") != BRANCH or git("merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD"):
        raise ValueError("wrong branch or compiled source commit not in current history")
    if git("status", "--porcelain"):
        raise ValueError("tracked worktree is dirty")
    if sha(archive / ARCHIVE_PROFILE) != PROFILE_SHA:
        raise ValueError("historical profile changed")
    matched = historical_manifest_check(archive)
    sources = source_check()
    plan = {
        "schema": "rm_dynamic_prediction_phase2_holdout/v1",
        "scope": "One new simulation-only, instrumented Navfn+V1 phase-2 diagnostic; no deployment acceptance",
        "branch": BRANCH, "run_commit": git("rev-parse", "HEAD"),
        "compiled_source_commit": SOURCE_COMMIT,
        "image_tag": IMAGE_TAG, "image_id": image_id(),
        "historical_candidate_manifest_entries_verified": matched,
        "instrumented_source_files_verified": sources,
        "historical_profile_sha256": PROFILE_SHA,
        "phase_s": 2, "batch_size": 300,
        "selection_rule": (
            "If sampled physical octagon-to-box clearance first falls below 0.05 m, "
            "select the last complete accepted MPPI cycle at least 0.25 s before that event. "
            "Otherwise select the complete accepted cycle at least 0.25 s before "
            "the minimum sampled physical clearance. Keep this trial regardless of outcome."
        ),
        "no_repeat_for_favorable_phase": True, "accepted_for_deployment": False,
        "file_sha256": {name: sha(path) for name, path in FILES.items()},
    }
    trial = series / "candidate_navfn_1"
    trial.mkdir(parents=True)
    shutil.copy2(archive / ARCHIVE_PROFILE, trial / "profile.yaml")
    (series / "plan.json").write_text(json.dumps(plan, indent=2) + "\n")
    print(json.dumps({"series": str(series), "run_commit": plan["run_commit"],
                      "image_id": plan["image_id"], "historical_files_verified": matched,
                      "instrumented_sources_verified": sources}, indent=2))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / "candidate_navfn_1"
    if (trial / "docker_exit.txt").exists() or (trial / "container_id").exists():
        raise FileExistsError("trial already started; refusing to repeat")
    if git("rev-parse", "HEAD") != plan["run_commit"] or git("status", "--porcelain"):
        raise ValueError("source tree changed after pre-registration")
    if image_id() != plan["image_id"] or sha(trial / "profile.yaml") != plan["historical_profile_sha256"]:
        raise ValueError("image or profile changed after pre-registration")
    source_check()
    for name, path in FILES.items():
        if sha(path) != plan["file_sha256"][name]:
            raise ValueError(f"frozen input changed: {name}")
    target = "/work/" + str(trial.relative_to(WORK))
    (WORK / "tmp").mkdir(exist_ok=True)
    args = ["docker", "run", "--rm", "--init", "--network", "none",
            "--cpus", "2", "--memory", "6g", "--memory-swap", "8g",
            "--pids-limit", "1024", "--security-opt", "no-new-privileges", "--cap-drop", "ALL",
            "--user", f"{os.getuid()}:{os.getgid()}", "--entrypoint", "bash",
            "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work:rw",
            "--cidfile", str(trial / "container_id")]
    for value in ("HOME=/work/tmp", "ROS_DOMAIN_ID=174", "ROS_LOCALHOST_ONLY=1",
                  "PYTHONDONTWRITEBYTECODE=1", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
                  "TMPDIR=/work/tmp", "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                  "TDT_PHASE_SECONDS=2", "IGN_PARTITION=dynamic_prediction_phase2_holdout_20260929"):
        args.extend(("-e", value))
    args.extend((IMAGE_TAG, "/ws/experiments/dynamic_prediction_v1/frozen_cycle/run_phase2_holdout.sh",
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
    series = args.series.resolve()
    if args.command == "prepare":
        preregister(args.archive.resolve(), series)
    else:
        run(series)
