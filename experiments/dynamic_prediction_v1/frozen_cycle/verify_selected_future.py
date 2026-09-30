#!/usr/bin/env python3
"""Check whether a fixed selected MPPI cycle has complete physical future labels."""
import argparse
import json
from pathlib import Path

import analyze


def check(trial):
    selection = json.loads((trial / "selection.json").read_text())
    cycle = Path(selection["selected_cycle_json"])
    truth = trial / "gazebo_poses.jsonl"
    if analyze.sha(cycle) != selection["selected_cycle_sha256"] or \
            analyze.sha(cycle.with_suffix(".bin")) != selection["selected_binary_sha256"] or \
            analyze.sha(truth) != selection["gazebo_truth_sha256"]:
        raise ValueError("selected cycle or physical truth changed")
    meta, _ = analyze.read_cycle(cycle)
    settings = analyze.event(meta, "settings")
    prediction = analyze.event(meta, "prediction.input")
    if prediction["status"] != "accepted":
        raise ValueError("selected prediction was not accepted")
    rows = analyze.rows_from_transport(truth)
    consumer = prediction["consumer_sim_s"]
    available = rows[-1]["t"] - consumer
    need_v1 = 9 * settings["dt"]
    need_mppi = settings["steps"] * settings["dt"]
    return {"schema": "rm_dynamic_prediction_selected_future_gate/v1",
            "cycle_id": meta["cycle_id"], "consumer_sim_s": consumer,
            "physical_truth_end_s": rows[-1]["t"],
            "available_future_s": available,
            "required_v1_future_s": need_v1,
            "required_mppi_future_s": need_mppi,
            "v1_future_complete": available >= need_v1,
            "mppi_future_complete": available >= need_mppi}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    args = parser.parse_args()
    result = check(args.trial)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["mppi_future_complete"] else 2)
