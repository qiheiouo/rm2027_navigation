#!/usr/bin/env python3
"""Run one fresh QP pilot with only the point-cloud raster margin corrected."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

from planner_sweep_v2 import sweep_points

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT / "build/tdt_p2b"
RUNS = WORK / "runs/dynamic_sweep_routing_pilot_v2"
GUARD = HERE.parent / "dynamic_guard_pilot_20260923"
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
    old = WORK / "runs/dynamic_sweep_routing_pilot_v1/profile.yaml"
    profile = RUNS / "profile.yaml"
    profile.write_bytes(old.read_bytes())
    assert sha(profile) == sha(old)
    points, source = sweep_points()
    script_paths = [*HERE.glob("*.py"), *HERE.glob("*.sh")]
    script_paths += [GUARD / n for n in (
        "guard_core.py", "guard_node.py", "observe_guard.py",
        "dynamic_guard.launch.py", "comparison_guard.launch.py",
        "reference_guard.launch.py", "phase1_guard.launch.py")]
    script_paths += [ROOT / n for n in (
        "src/rm_simulation/models/moving_obstacle.sdf",
        "src/rm_simulation/worlds/phase1_omni.sdf")]
    prior_manifest = GUARD / "manifest.json"
    write_new(RUNS / "inputs.json", {
        "schema": "tdt_dynamic_sweep_routing_inputs/v2",
        "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "runtime_source_commit": "a419654a8fed1fc8321c234fb212abd0a6cabe04",
        "image_id": subprocess.check_output(["docker", "image", "inspect",
                                             "rm2027_navigation:humble", "--format",
                                             "{{.Id}}"], text=True).strip(),
        "old_profile_sha256": sha(old), "profile_sha256": sha(profile),
        "previous_manifest_sha256": sha(prior_manifest),
        "body_polygon_m": json.loads((WORK / "runs/dynamic_guard_pilot_v1/inputs.json").read_text())["body_polygon_m"],
        "goal": [5.6, 0.0, 0.0], "phase_s": 0.0,
        "fixture_sweep": source, "published_points": len(points),
        "semantic_profile_difference": "None versus sweep-routing v1; profile is byte-identical. Only point-cloud raster margin changes from one cell to zero.",
        "files": {str(p.relative_to(ROOT)): sha(p) for p in script_paths},
        "scope": "One fixed-phase simulation diagnostic; no deployment/hardware acceptance."
    })


def verify():
    m = json.loads((RUNS / "inputs.json").read_text())
    for name, digest in m["files"].items():
        assert sha(ROOT / name) == digest, name
    assert sha(RUNS / "profile.yaml") == m["profile_sha256"]
    old = WORK / "runs/dynamic_sweep_routing_pilot_v1/profile.yaml"
    assert sha(RUNS / "profile.yaml") == sha(old)
    assert yaml.safe_load((RUNS / "profile.yaml").read_text()) == yaml.safe_load(old.read_text())
    assert not subprocess.check_output([
        "git", "diff", "a419654a8fed1fc8321c234fb212abd0a6cabe04",
        "--", "experiments", "src"], text=True)
    return m


def run():
    m = verify()
    trial = RUNS / "tdt_qp_1"
    trial.mkdir(exist_ok=False)
    (trial / "profile.yaml").write_bytes((RUNS / "profile.yaml").read_bytes())
    write_new(trial / "metadata.json", {
        "planner": "tdt_qp", "trial": 1, "mode": "guard_and_planner_sweep",
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
                "TDT_PHASE_SECONDS=0", "IGN_PARTITION=tdt_dynamic_sweep_routing_v2"):
        args += ["-e", env]
    args += ["rm2027_navigation:humble",
             "/ws/docs/tdt_migration/evidence/dynamic_sweep_routing_v2_20260923/run_trial.sh",
             "/work/" + str(trial.relative_to(WORK)),
             "/work/" + str((trial / "profile.yaml").relative_to(WORK))]
    status = subprocess.call(args)
    (trial / "docker_exit.txt").write_text(f"{status}\n")
    try:
        result = audit_dynamic.ex.analyze(trial, m)
        print(json.dumps({"docker_exit": status,
                          "evidence_valid": result["evidence_valid"],
                          "action": result["action_status"],
                          "recoveries": result["recoveries"],
                          "dynamic_gate": result["limited_dynamic_geometry_and_goal_pass"]}, indent=2))
    except Exception as error:
        (trial / "audit_error.txt").write_text(repr(error) + "\n")
        raise


if __name__ == "__main__":
    if sys.argv[1] == "prepare":
        prepare()
    elif sys.argv[1] == "run":
        run()
