#!/usr/bin/env python3
"""Reconstruct executed physical sweeps and metrics independently of MPC decisions."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from temporal_mpc.fixtures import Scenario
from temporal_mpc.oracle import step_and_audit


def verify(directory):
    manifest = json.loads((directory / "manifest.json").read_text())
    summary = json.loads((directory / "summary.json").read_text())
    reports = []
    recorded, identities = [], set()
    for path in sorted(directory.glob("*__*.json")):
        data = json.loads(path.read_text())
        report, cfg = data["report"], data["config"]
        recorded.append(report)
        identities.add((report["scenario"], report["geometry_mode"], report["consumption"], cfg["horizon"]))
        scenario = Scenario(**data["scenario"])
        rows = [json.loads(line) for line in path.with_suffix(".jsonl").read_text().splitlines()]
        assert len(rows) == report["cycles"] > 0, path.name
        state = np.asarray(scenario.start, float)
        minimum, sampled_min, collision, waited, distance = float("inf"), float("inf"), False, 0., 0.
        statuses, failures = Counter(), []
        for k, row in enumerate(rows):
            assert row["cycle"] == k and row["evaluation_ns"] == round(k * cfg["dt"] * 1e9)
            np.testing.assert_allclose(state, row["initial"], atol=1e-12, rtol=0)
            acceleration = np.asarray(row["acceleration"])
            assert np.all(np.abs(acceleration) <= np.asarray(cfg["acceleration"]) + cfg["tolerance"])
            np.testing.assert_allclose(row["command"], state[3:] + cfg["dt"] * acceleration, atol=1e-12, rtol=0)
            source = scenario.observe(k * cfg["dt"], report["geometry_mode"])
            assert row["source_ns"] == source.source_ns
            next_state, lower, sampled, contact = step_and_audit(state, acceleration, k * cfg["dt"], cfg["dt"],
                                                               scenario.truth, scenario.domain, scenario.speed_bound)
            assert abs(lower - row["physical_clearance_lower"]) < 1e-12
            assert abs(sampled - row["physical_clearance_sampled"]) < 1e-12
            assert contact == row["physical_contact"]
            minimum, sampled_min, collision = min(minimum, lower), min(sampled_min, sampled), collision or contact
            distance += float(np.linalg.norm(np.asarray(next_state[:2]) - state[:2]))
            state = np.asarray(next_state)
            if np.linalg.norm(state[3:5]) < .05:
                waited += cfg["dt"]
            statuses[row["status"]] += 1
            if not row["model_feasible"]:
                failures.append(k)
        np.testing.assert_allclose(state, report["terminal_state"], atol=1e-12, rtol=0)
        assert abs(minimum - report["minimum_physical_clearance_lower_m"]) < 1e-12
        assert abs(sampled_min - report["minimum_physical_clearance_sampled_m"]) < 1e-12
        assert collision == report["physical_collision"] and dict(statuses) == report["statuses"]
        assert abs(distance - report["travel_distance_m"]) < 1e-10
        assert abs(waited - report["stop_wait_duration_s"]) < 1e-10
        reached = (np.linalg.norm(state[:2] - scenario.goal[:2]) <= .12
                   and abs(np.arctan2(np.sin(state[2] - scenario.goal[2]), np.cos(state[2] - scenario.goal[2]))) <= .10
                   and np.max(np.abs(state[3:])) <= .05)
        assert bool(reached) == report["model_goal_reached"]
        assert bool(reached and not collision and minimum >= .05) == report["offline_physical_task_gate"]
        assert report["false_block_count"] is None and report["false_block_duration_s"] is None
        assert report["ros_action_success"] is None and report["recovery_count"] is None and not report["deployment_accepted"]
        assert sum(r["elapsed_s"] > .05 for r in rows) == report["target_20hz_deadline_miss_count"]
        assert sum(r["elapsed_s"] > cfg["deadline_s"] for r in rows) == report["configured_deadline_miss_count"]
        assert sum(bool(r["model_feasible"]) for r in rows) == report["model_feasible_cycles"]
        reports.append({"file": path.name, "replayed_cycles": len(rows), "physical_task_gate": report["offline_physical_task_gate"],
                        "uncertified_model_cycles": failures, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    expected = (len(manifest["arguments"]["scenarios"]) * len(manifest["arguments"]["modes"])
                * len(manifest["arguments"]["consumptions"]) * len(manifest["arguments"]["horizons"]))
    assert len(reports) == len(summary["reports"]) == expected
    assert len(identities) == expected
    assert Counter(json.dumps(r, sort_keys=True) for r in recorded) == Counter(json.dumps(r, sort_keys=True) for r in summary["reports"])
    return {"verdict": "EXECUTED SWEEP AND METRICS REPLAY PASS", "trials": reports,
            "scope": "Replays executed commands through the independent physical oracle. Does not prove optimized-horizon safety or STVL superiority.",
            "deployment_accepted": False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.directory)
    with args.output.open("x") as stream:
        stream.write(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"verdict": result["verdict"], "trials": len(result["trials"])}))
