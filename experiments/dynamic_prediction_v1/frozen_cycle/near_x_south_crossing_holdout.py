#!/usr/bin/env python3
"""Single opposite-side near-X crossing with the already validated fixture."""
import argparse
import json
from pathlib import Path

import near_x_crossing_holdout as north


START = (5.25, -1.10)
GOAL = (5.25, 1.10)


def set_geometry():
    north.START = START
    north.GOAL = GOAL


def prepare(archive, series):
    set_geometry()
    north.prepare(archive, series)
    plan_path = series / "plan.json"
    plan = json.loads(plan_path.read_text())
    if tuple(plan["robot_start_xy"]) != START or tuple(plan["goal_xy"]) != GOAL:
        raise ValueError("opposite-side geometry was not derived")
    plan["scenario"] = "south-start-to-north-goal-x-crossing"
    plan["south_wrapper_sha256"] = north.base.sha(Path(__file__).resolve())
    plan_path.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"scenario": plan["scenario"], "run_commit": plan["run_commit"]}))


def run(series):
    set_geometry()
    plan = json.loads((series / "plan.json").read_text())
    if plan.get("scenario") != "south-start-to-north-goal-x-crossing" or \
            tuple(plan["robot_start_xy"]) != START or \
            tuple(plan["goal_xy"]) != GOAL or \
            plan["south_wrapper_sha256"] != north.base.sha(Path(__file__).resolve()):
        raise ValueError("opposite-side plan or source changed")
    north.run(series)


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
