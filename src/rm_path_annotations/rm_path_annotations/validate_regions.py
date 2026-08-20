from __future__ import annotations

import argparse
from collections import Counter
import json
from typing import Sequence

from rm_path_annotations.core import (
    RegionContractError,
    load_region_set,
    validate_map_binding,
)


def main(arguments: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate an immutable rm_semantic_regions/v1 YAML file."
    )
    parser.add_argument("--regions", required=True)
    parser.add_argument("--expected-map-id")
    parser.add_argument("--expected-map-revision")
    parser.add_argument("--expected-manifest-sha256")
    args = parser.parse_args(arguments)
    expected = (
        args.expected_map_id,
        args.expected_map_revision,
        args.expected_manifest_sha256,
    )
    if any(value is not None for value in expected) and not all(expected):
        parser.error("expected map binding arguments must be provided together")
    try:
        region_set = load_region_set(args.regions)
        if all(expected):
            validate_map_binding(
                region_set,
                expected_map_id=args.expected_map_id,
                expected_map_revision=args.expected_map_revision,
                expected_manifest_sha256=args.expected_manifest_sha256,
            )
    except RegionContractError as error:
        parser.error(str(error))
    counts = Counter(region.region_type.name.lower() for region in region_set.regions)
    print(
        json.dumps(
            {
                "schema": "rm_semantic_regions/v1",
                "region_set_id": region_set.region_set_id,
                "revision": region_set.revision,
                "region_set_sha256": region_set.contract_sha256,
                "map_id": region_set.map_binding.map_id,
                "map_revision": region_set.map_binding.map_revision,
                "manifest_sha256": region_set.map_binding.manifest_sha256,
                "region_count": len(region_set.regions),
                "region_type_counts": dict(sorted(counts.items())),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
