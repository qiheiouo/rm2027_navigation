from __future__ import annotations

import argparse
import json
import sys

from .map_bundle import MapBundleError, resolve_map_bundle_for_runtime


def main() -> int:
    parser = argparse.ArgumentParser(description="Resolve an RM 2027 map bundle for runtime use")
    parser.add_argument("manifest", help="Path to *.bundle.yaml")
    parser.add_argument(
        "--allow-test-map",
        action="store_true",
        help="Allow test_only/candidate bundles. Never use this on a robot.",
    )
    parser.add_argument(
        "--field",
        choices=[
            "manifest",
            "map_id",
            "revision",
            "deployment_status",
            "pcd_path",
            "occupancy_yaml_path",
            "occupancy_image_path",
        ],
        help="Print a single resolved field instead of JSON.",
    )
    arguments = parser.parse_args()

    try:
        result = resolve_map_bundle_for_runtime(
            arguments.manifest,
            allow_test_map=arguments.allow_test_map,
        )
    except MapBundleError as exc:
        print(f"map bundle unresolved: {exc}", file=sys.stderr)
        return 2

    if arguments.field:
        print(result[arguments.field])
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
