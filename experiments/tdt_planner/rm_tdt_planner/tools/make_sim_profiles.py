#!/usr/bin/env python3
"""Freeze four simulation profiles, changing only the global planner block."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--verify", action="store_true", help="Read-only verification of an existing profile set")
    args = parser.parse_args()
    baseline = yaml.safe_load(args.baseline.read_text())
    controller = baseline["controller_server"]["ros__parameters"]["FollowPath"]
    if controller["plugin"] != "nav2_mppi_controller::MPPIController" or controller["motion_model"] != "Omni":
        parser.error("expected the existing simulation MPPI Omni profile")
    for name in ("local_costmap", "global_costmap"):
        costmap = baseline[name][name]["ros__parameters"]
        footprint = yaml.safe_load(costmap["footprint"])
        if footprint != [[-.30, -.25], [-.30, .25], [.30, .25], [.30, -.25]] or costmap["footprint_padding"] != .03:
            parser.error("geometry recorder requires the unchanged Phase 1.5 fixture footprint/padding")
    planner = baseline["planner_server"]["ros__parameters"]
    if planner["planner_plugins"] != ["GridBased"]:
        parser.error("expected a single GridBased planner")
    fragment = yaml.safe_load((Path(__file__).resolve().parents[1] /
                               "config/planner_parameters.yaml").read_text())
    tdt = fragment["planner_server"]["ros__parameters"]["GridBased"]
    variants = {
        "navfn": planner["GridBased"],
        "smac2d": {"plugin": "nav2_smac_planner/SmacPlanner2D",
                   "tolerance": planner["GridBased"].get("tolerance", 0.5),
                   "allow_unknown": planner["GridBased"].get("allow_unknown", True),
                   "max_planning_time": 0.25},
        "tdt_astar": dict(tdt, optimize=False), "tdt_qp": dict(tdt, optimize=True),
    }
    if args.verify:
        manifest = json.loads((args.output / "manifest.json").read_text())
        if manifest["baseline_sha256"] != hashlib.sha256(args.baseline.read_bytes()).hexdigest():
            parser.error("baseline changed since profile freeze")
        for name, block in variants.items():
            path = args.output / f"{name}.yaml"
            expected = copy.deepcopy(baseline)
            expected["planner_server"]["ros__parameters"]["GridBased"] = block
            if yaml.safe_load(path.read_text()) != expected or manifest["profiles"][name] != hashlib.sha256(path.read_bytes()).hexdigest():
                parser.error(f"profile changed or differs from current generator: {name}")
        print("Four frozen profiles match the baseline and generator")
        return
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {"baseline_sha256": hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
                "only_changed_block": "planner_server.ros__parameters.GridBased", "profiles": {}}
    for name, block in variants.items():
        profile = copy.deepcopy(baseline)
        profile["planner_server"]["ros__parameters"]["GridBased"] = block
        path = args.output / f"{name}.yaml"
        path.write_text("# Simulation-only TDT P2B comparison.\n" + yaml.safe_dump(profile, sort_keys=False))
        manifest["profiles"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
