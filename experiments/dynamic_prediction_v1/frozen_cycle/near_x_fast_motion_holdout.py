#!/usr/bin/env python3
"""One-shot near-X crossing with only the moving box period changed to 6 s."""
import argparse
import json
from pathlib import Path

import near_x_crossing_holdout as north


PERIOD_S = 6.0
PERIOD_OLD = "'period':8.0"
PERIOD_NEW = "'period':6.0"


def prepare(archive, series):
    north.prepare(archive, series)
    launch = series / "dynamic_near_x.launch.py"
    changed = north.swap_once(launch.read_text(), PERIOD_OLD, PERIOD_NEW)
    launch.write_text(changed)
    plan_path = series / "plan.json"
    plan = json.loads(plan_path.read_text())
    if plan["phase_s"] != 6 or tuple(plan["robot_start_xy"]) != north.START or \
            tuple(plan["goal_xy"]) != north.GOAL:
        raise ValueError("fixed geometry or phase changed")
    plan["scenario"] = "north-start-x-slider-period-6s"
    plan["moving_box_period_s"] = PERIOD_S
    plan["moving_box_amplitude_m"] = .9
    plan["fast_wrapper_sha256"] = north.base.sha(Path(__file__).resolve())
    plan["derived_sha256"]["dynamic_near_x.launch.py"] = north.base.sha(launch)
    plan_path.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"scenario": plan["scenario"],
                      "run_commit": plan["run_commit"],
                      "period_s": PERIOD_S}))


def run(series):
    plan = json.loads((series / "plan.json").read_text())
    launch = series / "dynamic_near_x.launch.py"
    if plan.get("scenario") != "north-start-x-slider-period-6s" or \
            plan.get("moving_box_period_s") != PERIOD_S or \
            plan.get("moving_box_amplitude_m") != .9 or \
            plan["fast_wrapper_sha256"] != north.base.sha(Path(__file__).resolve()) or \
            launch.read_text().count(PERIOD_NEW) != 1:
        raise ValueError("fast-motion plan or derivation changed")
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
