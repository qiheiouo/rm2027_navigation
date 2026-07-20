from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .map_bundle import MapBundleError
from .map_quality import (
    analyze_quality,
    unique_default_output_root,
    write_quality_outputs,
)


DIAGNOSTIC_ROOT = Path("/tmp/rm27_pcd_pgm_diag")


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def validate_output_location(output: str | Path, manifest: str | Path) -> Path:
    destination = Path(output).expanduser().resolve()
    diagnostic_root = DIAGNOSTIC_ROOT.resolve()
    if not _is_within(destination, diagnostic_root) or destination == diagnostic_root:
        raise MapBundleError(
            f"diagnostic output must be a new child of {diagnostic_root}"
        )
    bundle_directory = Path(manifest).expanduser().resolve().parent
    if _is_within(destination, bundle_directory):
        raise MapBundleError("diagnostic output must not be inside the map bundle")
    return destination


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze an immutable PCD/occupancy bundle without modifying it. "
            "Saved-PCD support is diagnostic evidence, not a deletion mask."
        )
    )
    parser.add_argument("manifest", help="path to a map bundle manifest")
    parser.add_argument(
        "--output",
        help=(
            "new output directory under /tmp/rm27_pcd_pgm_diag; "
            "defaults to a unique UTC run directory"
        ),
    )
    parser.add_argument(
        "--labels",
        help="optional map-frame known-free/protected-obstacle/landmark YAML",
    )
    parser.add_argument(
        "--bag",
        help=(
            "optional rosbag directory or metadata.yaml; this command checks topic "
            "coverage but does not replay the bag"
        ),
    )
    parser.add_argument(
        "--print-summary",
        action="store_true",
        help="also print the machine-readable summary JSON to stdout",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    output = arguments.output or unique_default_output_root()
    try:
        destination = validate_output_location(output, arguments.manifest)
        summary, components, images = analyze_quality(
            arguments.manifest,
            labels_path=arguments.labels,
            bag_path=arguments.bag,
        )
        write_quality_outputs(destination, summary, components, images)
    except (MapBundleError, OSError, ValueError) as exc:
        print(f"map quality analysis failed: {exc}", file=sys.stderr)
        return 2

    if arguments.print_summary:
        print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    else:
        print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
