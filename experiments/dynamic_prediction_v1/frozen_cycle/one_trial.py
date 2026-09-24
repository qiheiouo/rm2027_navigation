#!/usr/bin/env python3
"""Pre-register and run one instrumented Navfn+V1 diagnostic series."""
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


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def image_id():
    return subprocess.check_output(
        ["docker", "image", "inspect", "--format", "{{.Id}}", IMAGE_TAG],
        text=True).strip()


def manifest_check(archive):
    manifest = json.loads((ROOT / "docs/dynamic_navigation/evidence/v1_navfn_transfer_20260924/manifest.json").read_text())
    selected = {name: digest for name, digest in manifest["files_sha256"].items()
                if name.startswith(ARCHIVE_PREFIX)}
    if len(selected) != 52:
        raise ValueError("historical candidate manifest entry count changed")
    for name, digest in selected.items():
        path = archive / name.removeprefix("build/tdt_p2b/")
        if sha(path) != digest:
            raise ValueError(f"historical evidence hash mismatch: {name}")
    return len(selected)


def prepare(archive, series):
    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    if branch != "experiment/dynamic-prediction-v1":
        raise ValueError(f"wrong branch: {branch}")
    matched = manifest_check(archive)
    if sha(archive / ARCHIVE_PROFILE) != PROFILE_SHA:
        raise ValueError("historical profile changed")
    binaries = {
        "nav2_mppi_controller": WORK / "dynamic_prediction_trace_v3/install/nav2_mppi_controller/lib/libmppi_controller.so",
        "nav2_mppi_critics": WORK / "dynamic_prediction_trace_v3/install/nav2_mppi_controller/lib/libmppi_critics.so",
        "prediction_v1_critic": WORK / "dynamic_prediction_trace_v3/install/rm_dynamic_prediction_critic/lib/librm_dynamic_prediction_critic.so",
        "simulation_moving_obstacle_controller": WORK / "dynamic_prediction_runtime/install/rm_simulation/lib/rm_simulation/moving_obstacle_controller",
    }
    profile = archive / ARCHIVE_PROFILE
    plan = {
        "schema": "rm_dynamic_prediction_frozen_cycle_probe/v1",
        "scope": "One new instrumented Navfn+V1 phase-0 diagnostic, not a historical replay or acceptance trial.",
        "source_commit": commit, "source_branch": branch,
        "image_tag": IMAGE_TAG, "image_id": image_id(),
        "historical_candidate_manifest_entries_verified": matched,
        "historical_profile_sha256": PROFILE_SHA,
        "phase_s": 0, "batch_size": 300, "selection_rule":
            "If sampled body clearance first falls below 0.05 m, select the last complete accepted MPPI cycle at least 0.25 s before that event. Otherwise select the complete accepted cycle at least 0.25 s before the minimum sampled body clearance. Retain this one trial regardless of outcome.",
        "no_repeat_for_favorable_phase": True,
        "accepted_for_deployment": False,
        "file_sha256": {
            "docker/Dockerfile.humble": sha(ROOT / "docker/Dockerfile.humble"),
            "trace_source_manifest": sha(WORK / "dynamic_prediction_trace_sources_v3/trace_sources.json"),
            "run_trace_trial.sh": sha(HERE / "run_trace_trial.sh"),
            **{name: sha(path) for name, path in binaries.items()},
        },
    }
    if series.exists():
        raise FileExistsError("series exists; refusing to overwrite")
    trial = series / "candidate_navfn_1"
    trial.mkdir(parents=True)
    shutil.copy2(profile, trial / "profile.yaml")
    (series / "plan.json").write_text(json.dumps(plan, indent=2) + "\n")
    print(json.dumps({"series": str(series), "commit": commit, "image_id": plan["image_id"],
                      "historical_files_verified": matched}, indent=2))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    trial = series / "candidate_navfn_1"
    if (trial / "docker_exit.txt").exists() or (trial / "container_id").exists():
        raise FileExistsError("trial already started; refusing to repeat")
    if image_id() != plan["image_id"] or sha(trial / "profile.yaml") != plan["historical_profile_sha256"]:
        raise ValueError("image or profile changed after pre-registration")
    checks = {
        "docker/Dockerfile.humble": ROOT / "docker/Dockerfile.humble",
        "trace_source_manifest": WORK / "dynamic_prediction_trace_sources_v3/trace_sources.json",
        "run_trace_trial.sh": HERE / "run_trace_trial.sh",
        "nav2_mppi_controller": WORK / "dynamic_prediction_trace_v3/install/nav2_mppi_controller/lib/libmppi_controller.so",
        "nav2_mppi_critics": WORK / "dynamic_prediction_trace_v3/install/nav2_mppi_controller/lib/libmppi_critics.so",
        "prediction_v1_critic": WORK / "dynamic_prediction_trace_v3/install/rm_dynamic_prediction_critic/lib/librm_dynamic_prediction_critic.so",
        "simulation_moving_obstacle_controller": WORK / "dynamic_prediction_runtime/install/rm_simulation/lib/rm_simulation/moving_obstacle_controller",
    }
    for name, path in checks.items():
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
                  "TDT_PHASE_SECONDS=0", "IGN_PARTITION=dynamic_prediction_frozen_cycle_probe_01"):
        args.extend(("-e", value))
    args.extend((IMAGE_TAG, "/ws/experiments/dynamic_prediction_v1/frozen_cycle/run_trace_trial.sh",
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
