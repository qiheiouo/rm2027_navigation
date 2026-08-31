"""Command-line validator for a map-bound dog-hole route sidecar."""

from __future__ import annotations

import argparse

from rm_dog_hole_entry_gate.route import load_route, validate_route_map_binding
from rm_path_annotations.core import RegionContractError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("route_file")
    parser.add_argument("--expected-map-id", required=True)
    parser.add_argument("--expected-map-revision", required=True)
    parser.add_argument("--expected-manifest-sha256", required=True)
    arguments = parser.parse_args(argv)
    try:
        route = load_route(arguments.route_file)
        validate_route_map_binding(
            route,
            expected_map_id=arguments.expected_map_id,
            expected_map_revision=arguments.expected_map_revision,
            expected_manifest_sha256=arguments.expected_manifest_sha256,
        )
    except RegionContractError as error:
        parser.exit(2, f"FAIL: {error}\n")
    print(
        "PASS: "
        f"{route.route_id}@{route.revision}, "
        f"contract_sha256={route.contract_sha256}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
