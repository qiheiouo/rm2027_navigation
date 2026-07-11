from __future__ import annotations

import argparse
import json
import sys

from .map_bundle import MapBundleError, validate_map_bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate an RM 2027 map bundle")
    parser.add_argument("manifest", help="Path to *.bundle.yaml")
    parser.add_argument(
        "--require-approved",
        action="store_true",
        help="Reject test_only and candidate bundles",
    )
    parser.add_argument("--json", action="store_true", help="Print machine-readable output")
    arguments = parser.parse_args()

    try:
        result = validate_map_bundle(arguments.manifest, arguments.require_approved)
    except MapBundleError as exc:
        print(f"map bundle invalid: {exc}", file=sys.stderr)
        return 2

    if arguments.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        pcd_summary = (
            f"pcd_points={result['pcd']['points']}"
            if result["pcd"] is not None
            else "pcd=none"
        )
        print(
            "map bundle valid: "
            f"{result['map_id']} revision={result['revision']} "
            f"status={result['deployment_status']} "
            f"map_type={result['map_type']} {pcd_summary}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
