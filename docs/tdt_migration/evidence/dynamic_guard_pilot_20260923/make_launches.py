#!/usr/bin/env python3
"""Generate exact simulation-only launch copies, changing only command routing."""
import difflib
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
BASE = "docs/tdt_migration/evidence"
PREFIX = "/ws/" + str(HERE.relative_to(ROOT))
SOURCES = [
    (ROOT / "src/rm_simulation/launch/phase1_5_gazebo.launch.py",
     HERE / "phase1_guard.launch.py",
     '"cmd_vel_topic": "/cmd_vel",', '"cmd_vel_topic": "/cmd_vel_guarded",'),
    (ROOT / "experiments/tdt_planner/rm_tdt_planner/launch/simulation_comparison.launch.py",
     HERE / "comparison_guard.launch.py",
     'PythonLaunchDescriptionSource(PathJoinSubstitution([\n'
     '                share, "launch", "phase1_5_gazebo.launch.py"]))',
     f'PythonLaunchDescriptionSource("{PREFIX}/phase1_guard.launch.py")'),
    (ROOT / f"{BASE}/snapshot_revalidation_20260922/reference.launch.py",
     HERE / "reference_guard.launch.py",
     f"/ws/experiments/tdt_planner/rm_tdt_planner/launch/simulation_comparison.launch.py",
     f"{PREFIX}/comparison_guard.launch.py"),
    (ROOT / f"{BASE}/dynamic_reference_20260922/dynamic.launch.py",
     HERE / "dynamic_guard.launch.py",
     f"/ws/{BASE}/snapshot_revalidation_20260922/reference.launch.py",
     f"{PREFIX}/reference_guard.launch.py"),
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    report = []
    for source, target, before, after in SOURCES:
        original = source.read_text()
        assert original.count(before) == 1, source
        changed = original.replace(before, after)
        assert changed.replace(after, before) == original
        target.write_text(changed)
        report.append({"source": str(source.relative_to(ROOT)),
                       "source_sha256": sha(source),
                       "candidate": str(target.relative_to(ROOT)),
                       "candidate_sha256": sha(target),
                       "diff": "".join(difflib.unified_diff(
                           original.splitlines(True), changed.splitlines(True),
                           fromfile=str(source.relative_to(ROOT)),
                           tofile=str(target.relative_to(ROOT))))})
    import json
    with (HERE / "launch_diff.json").open("x") as f:
        json.dump(report, f, indent=2)
        f.write("\n")


if __name__ == "__main__":
    main()
