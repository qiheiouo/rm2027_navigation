#!/usr/bin/env python3
"""Fixture-bounded obstacle sweep, checked against frozen braking candidates.

This is a shadow calculation. It does not publish or replace commands.
"""
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE.parent / "dynamic_safety_contract_20260923"))
from braking_replay import braking_poses, placed, padded, polygon_distance, radius


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fixture_sweep():
    sdf_path = ROOT / "src/rm_simulation/models/moving_obstacle.sdf"
    launch_path = HERE.parent / "dynamic_reference_20260922/dynamic.launch.py"
    sdf = ET.parse(sdf_path)
    joint = sdf.find('.//joint[@name="slider_joint"]')
    assert joint.attrib["type"] == "prismatic"
    assert joint.find("axis/xyz").text.strip() == "0 1 0"
    low = float(joint.find("axis/limit/lower").text)
    high = float(joint.find("axis/limit/upper").text)
    size = [float(v) for v in sdf.find(
        './/link[@name="obstacle_link"]/collision/geometry/box/size').text.split()]
    match = re.search(r"'-x','([^']+)','-y','([^']+)'", launch_path.read_text())
    assert match
    center_x, center_y = map(float, match.groups())
    assert low < high and all(v > 0 for v in size)
    x0, x1 = center_x - size[0] / 2, center_x + size[0] / 2
    y0, y1 = center_y + low - size[1] / 2, center_y + high + size[1] / 2
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]], {
        "slider_limit_m": [low, high], "box_size_m": size,
        "model_spawn_xy_m": [center_x, center_y],
        "sdf_sha256": sha(sdf_path), "launch_sha256": sha(launch_path)}


def shadow_case(cycle, body, sweep, delay):
    # Identical unverified brake model to the prior hindsight replay. The
    # obstacle, unlike that replay, is its known full legal sweep at all times.
    speed = cycle["odom_speed"]
    step = 0.01
    poses = braking_poses(cycle["pose"], speed, delay, horizon=2.0, step=step)
    v = math.hypot(*speed[:2])
    results = {}
    for name, poly, gate in (("body", body, 0.05),
                             ("padded", padded(body, 0.03), 0.0)):
        sampled = min(polygon_distance(placed(poly, pose), sweep) for pose in poses)
        # The nearest integration sample is at most half a step away; the
        # static sweep includes every legal obstacle pose by construction.
        lower = sampled - step / 2 * (v + radius(poly) * abs(speed[2]))
        results[name] = {"sample_min_m": sampled, "continuous_bound_m": lower,
                         "passes_assumed_model": lower >= gate if gate else lower > gate}
    return {"delay_s": delay, "body": results["body"], "padded": results["padded"],
            "assumed_braking_gate_pass": (
                results["body"]["passes_assumed_model"] and
                results["padded"]["passes_assumed_model"])}


def main():
    previous_path = HERE.parent / "dynamic_safety_contract_20260923/braking_replay.json"
    previous = json.loads(previous_path.read_text())
    inputs_path = ROOT / "build/tdt_p2b/runs/dynamic_map_age_pilot_v1/inputs.json"
    inputs = json.loads(inputs_path.read_text())
    body = inputs["body_polygon_m"]
    sweep, source = fixture_sweep()
    result = {"schema": "tdt_fixture_sweep_shadow/v1",
              "scope": "Frozen instrumented trials; fixture joint bounds, reference polygon and hypothetical configured deceleration. No online control or verified physical stop certificate.",
              "hindsight_replay_sha256": sha(previous_path),
              "inputs_sha256": sha(inputs_path), "fixture": source,
              "sweep_polygon_m": sweep, "trials": []}
    for trial in previous["trials"]:
        name = trial["planner"]
        assert trial["profile_sha256"] == inputs["profiles"][name]
        output = {"planner": name, "first_body_gap_violation_sim_s":
                  trial["first_body_gap_violation_sim_s"], "delays": []}
        for delay in previous["assumptions"]["response_delays_s"]:
            cycles = []
            for row in trial["cycles"]:
                shadow = shadow_case(row, body, sweep, delay)
                hindsight = next(c for c in row["cases"] if c["delay_s"] == delay)
                # The actual sampled box lies inside the full fixture sweep.
                # A passing full-sweep stop must pass the less conservative
                # observed-future geometric check under the same brake model.
                if shadow["assumed_braking_gate_pass"]:
                    assert hindsight["assumed_braking_gate_pass"], (name, row["cycle_id"], delay)
                cycles.append({"cycle_id": row["cycle_id"], "sim_s": row["sim_s"],
                               "relative_to_first_violation_s":
                               row["relative_to_first_violation_s"],
                               "full_sweep": shadow,
                               "hindsight_pass": hindsight["assumed_braking_gate_pass"]})
            safe = [r for r in cycles if r["full_sweep"]["assumed_braking_gate_pass"]]
            unsafe = [r for r in cycles if not r["full_sweep"]["assumed_braking_gate_pass"]]
            output["delays"].append({
                "delay_s": delay, "cycles": cycles,
                "last_safe_cycle": safe[-1]["cycle_id"] if safe else None,
                "first_unsafe_cycle": unsafe[0]["cycle_id"] if unsafe else None,
                "first_unsafe_lead_before_actual_violation_s":
                -unsafe[0]["relative_to_first_violation_s"] if unsafe else None,
                "safe_to_unsafe_transitions": sum(
                    a["full_sweep"]["assumed_braking_gate_pass"] and
                    not b["full_sweep"]["assumed_braking_gate_pass"]
                    for a, b in zip(cycles, cycles[1:])),
            })
        result["trials"].append(output)
    with (HERE / "sweep_shadow.json").open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    for trial in result["trials"]:
        print(trial["planner"],
              [(d["delay_s"], d["last_safe_cycle"], d["first_unsafe_cycle"],
                d["first_unsafe_lead_before_actual_violation_s"]) for d in trial["delays"]])


if __name__ == "__main__":
    main()
