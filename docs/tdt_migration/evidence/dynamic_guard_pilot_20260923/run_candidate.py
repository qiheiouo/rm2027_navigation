#!/usr/bin/env python3
"""Run a prespecified QP baseline/guard diagnostic pair in fresh directories."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT / "build/tdt_p2b"
RUNS = WORK / "runs/dynamic_guard_pilot_v1"
sys.path.insert(0, str(HERE.parent / "dynamic_reference_20260922"))
import audit_dynamic


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path, data):
    with path.open("x") as f:
        json.dump(data, f, indent=2, allow_nan=False)
        f.write("\n")


def prepare():
    RUNS.mkdir(exist_ok=False)
    (RUNS / "profiles").mkdir()
    source = WORK / "runs/dynamic_map_age_pilot_v1"
    old = json.loads((source / "inputs.json").read_text())
    profile = source / "tdt_qp_1/profile.yaml"
    target = RUNS / "profiles/tdt_qp.yaml"
    target.write_bytes(profile.read_bytes())
    assert sha(target) == old["profiles"]["tdt_qp"]
    paths = [*HERE.glob("*.py"), *HERE.glob("*.sh")]
    paths += [HERE / n for n in (
        "phase1_guard.launch.py", "comparison_guard.launch.py",
        "reference_guard.launch.py", "dynamic_guard.launch.py", "launch_diff.json")]
    paths += [ROOT / n for n in (
        "src/rm_simulation/models/moving_obstacle.sdf",
        "src/rm_simulation/worlds/phase1_omni.sdf",
        "src/rm_chassis_interface/src/chassis_interface_stub.cpp")]
    image = subprocess.check_output(
        ["docker", "image", "inspect", "rm2027_navigation:humble",
         "--format", "{{.Id}}"], text=True).strip()
    write_new(RUNS / "inputs.json", {
        "schema": "tdt_dynamic_guard_pilot_inputs/v1",
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True).strip(),
        "runtime_source_commit": "a419654a8fed1fc8321c234fb212abd0a6cabe04",
        "image_id": image, "profile_sha256": sha(target),
        "body_polygon_m": old["body_polygon_m"], "goal": [5.6, 0.0, 0.0],
        "phase_s": 0.0, "cases": ["baseline/tdt_qp_1", "guard/tdt_qp_1"],
        "files": {str(p.relative_to(ROOT)): sha(p) for p in paths},
        "scope": "Fixed-phase simulation diagnostic pair, not acceptance matrix. Full fixture sweep and assumed response only in guard case."
    })


def verify():
    m = json.loads((RUNS / "inputs.json").read_text())
    for name, digest in m["files"].items():
        assert sha(ROOT / name) == digest, name
    assert sha(RUNS / "profiles/tdt_qp.yaml") == m["profile_sha256"]
    assert not subprocess.check_output(
        ["git", "diff", "a419654a8fed1fc8321c234fb212abd0a6cabe04",
         "--", "experiments", "src"], text=True)
    return m


def run(mode):
    assert mode in ("baseline", "guard")
    m = verify()
    trial = RUNS / mode / "tdt_qp_1"
    trial.mkdir(parents=True, exist_ok=False)
    (trial / "profile.yaml").write_bytes((RUNS / "profiles/tdt_qp.yaml").read_bytes())
    write_new(trial / "metadata.json", {
        "planner": "tdt_qp", "trial": 1, "mode": mode,
        "source_commit": m["source_commit"],
        "runtime_source_commit": m["runtime_source_commit"],
        "image_id": m["image_id"], "profile_sha256": m["profile_sha256"],
        "scope": m["scope"], "performance_is_not_algorithm_rejection": True})
    args = ["docker", "run", "--rm", "--init", "--network", "none",
            "--user", f"{os.getuid()}:{os.getgid()}", "--entrypoint", "bash",
            "-v", f"{ROOT}:/ws:ro", "-v", f"{WORK}:/work",
            "--cidfile", str(trial / "container_id")]
    for env in ("ROS_DOMAIN_ID=174", "ROS_LOCALHOST_ONLY=1",
                "PYTHONDONTWRITEBYTECODE=1", "TMPDIR=/work/tmp",
                "LIBGL_ALWAYS_SOFTWARE=true", "QT_QPA_PLATFORM=offscreen",
                "TDT_PHASE_SECONDS=0",
                f"IGN_PARTITION=tdt_dynamic_guard_pilot_v1_{mode}"):
        args += ["-e", env]
    args += ["rm2027_navigation:humble",
             "/ws/docs/tdt_migration/evidence/dynamic_guard_pilot_20260923/run_trial.sh",
             "/work/" + str(trial.relative_to(WORK)),
             "/work/" + str((trial / "profile.yaml").relative_to(WORK)), mode]
    status = subprocess.call(args)
    (trial / "docker_exit.txt").write_text(f"{status}\n")
    try:
        # The dynamic auditor recomputes the raw observer summary and both
        # target/surrogate geometry checks, even when navigation failed.
        audit_dynamic.ex.analyze(trial, m)
        print(json.dumps({
            "mode": mode, "docker_exit": status,
            "summary": json.loads((trial / "dynamic_summary.json").read_text())
            ["limited_dynamic_geometry_and_goal_pass"]}, indent=2))
    except Exception as error:
        (trial / "audit_error.txt").write_text(repr(error) + "\n")
        raise
    return 0


if __name__ == "__main__":
    if sys.argv[1] == "prepare":
        prepare()
    elif sys.argv[1] == "run":
        sys.exit(run(sys.argv[2]))
