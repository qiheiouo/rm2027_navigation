#!/usr/bin/env python3
"""Create a new complete Nav2 candidate profile, refusing every overwrite."""
import argparse
from pathlib import Path
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--frontend-only", action="store_true")
    args = parser.parse_args()
    data = yaml.safe_load(args.baseline.read_text())
    fragment = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config/planner_parameters.yaml").read_text())
    params = data["planner_server"]["ros__parameters"]
    if params.get("planner_plugins") != ["GridBased"]:
        parser.error("expected a complete baseline using only GridBased; adapt explicitly")
    params["GridBased"] = fragment["planner_server"]["ros__parameters"]["GridBased"]
    params["GridBased"]["optimize"] = not args.frontend_only
    # Exclusive creation also rejects symlinks and identical source/destination.
    with args.output.open("x") as output:
        output.write("# Experimental T-DT candidate; not accepted for physical motion.\n")
        yaml.safe_dump(data, output, sort_keys=False, allow_unicode=True)


if __name__ == "__main__":
    main()
