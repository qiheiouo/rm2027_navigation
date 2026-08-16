"""Conservative ray-evidence cleanup for PCD candidates with frame sidecars."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

from .map_bundle import MapBundleError, validate_map_bundle
from .map_export import write_ascii_pcd
from .map_quality import read_ascii_xyz_pcd
from .immutable_output import (
    OUTPUT_FILE_MODE,
    new_output_path as _new_output_path,
    publish_new_file as _publish_new_file,
    require_distinct_paths as _require_distinct_paths,
    resolved_output_path as _resolved_output_path,
    write_new_text as _write_new_text,
)
from .ray_observations import (
    SCHEMA,
    RayObservation,
    load_ray_observations,
    stream_ray_observations,
)


__all__ = [
    "SCHEMA",
    "OUTPUT_FILE_MODE",
    "RayObservation",
    "CleanupResult",
    "clean_with_ray_evidence",
    "load_ray_observations",
    "stream_ray_observations",
    "traverse_voxels",
    "voxel_key",
]


@dataclass(frozen=True)
class CleanupResult:
    kept_points: np.ndarray
    report: dict[str, object]


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def voxel_key(
    point: Iterable[float], resolution: float
) -> tuple[int, int, int]:
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError("voxel resolution must be finite and positive")
    values = tuple(float(value) for value in point)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("voxel point must contain three finite values")
    return tuple(math.floor(value / resolution) for value in values)


def traverse_voxels(
    origin: Iterable[float], endpoint: Iterable[float], resolution: float
) -> tuple[tuple[int, int, int], ...]:
    """Amanatides-Woo traversal excluding origin and endpoint voxels."""
    start = np.asarray(tuple(origin), dtype=np.float64)
    end = np.asarray(tuple(endpoint), dtype=np.float64)
    if (
        start.shape != (3,)
        or end.shape != (3,)
        or not np.isfinite([start, end]).all()
    ):
        raise ValueError("ray origin and endpoint must be finite XYZ values")
    delta = end - start
    distance = float(np.linalg.norm(delta))
    if distance <= 1.0e-12:
        return ()
    current = list(voxel_key(start, resolution))
    target = voxel_key(end, resolution)
    if tuple(current) == target:
        return ()
    direction = delta / distance
    step = [
        1 if value > 0.0 else -1 if value < 0.0 else 0
        for value in direction
    ]
    infinity = math.inf
    t_max: list[float] = [infinity, infinity, infinity]
    t_delta: list[float] = [infinity, infinity, infinity]
    for axis in range(3):
        if step[axis] == 0:
            continue
        boundary = (
            (current[axis] + 1) * resolution
            if step[axis] > 0
            else current[axis] * resolution
        )
        t_max[axis] = (boundary - start[axis]) / direction[axis]
        t_delta[axis] = resolution / abs(direction[axis])

    traversed: list[tuple[int, int, int]] = []
    max_steps = sum(abs(target[axis] - current[axis]) for axis in range(3)) + 3
    for _ in range(max_steps):
        axis = min(range(3), key=lambda index: t_max[index])
        current[axis] += step[axis]
        t_max[axis] += t_delta[axis]
        key = tuple(current)
        if key == target:
            break
        traversed.append(key)
    return tuple(traversed)


def clean_with_ray_evidence(
    points: np.ndarray,
    observations: Iterable[RayObservation],
    voxel_resolution: float,
    min_pass_observations: int,
    pass_to_hit_ratio: float,
) -> CleanupResult:
    """Remove voxels repeatedly traversed after their endpoint hits."""
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] == 0:
        raise ValueError("cleanup requires a non-empty Nx3 point array")
    if not np.isfinite(xyz).all():
        raise ValueError("cleanup point cloud contains non-finite coordinates")
    if min_pass_observations <= 0 or pass_to_hit_ratio <= 0.0:
        raise ValueError("cleanup evidence thresholds must be positive")

    point_keys = [voxel_key(point, voxel_resolution) for point in xyz]
    candidate_keys = set(point_keys)
    hit_frames = {key: 0 for key in candidate_keys}
    pass_frames = {key: 0 for key in candidate_keys}
    observation_count = 0
    ray_count = 0
    for observation in observations:
        observation_count += 1
        frame_hits: set[tuple[int, int, int]] = set()
        frame_passes: set[tuple[int, int, int]] = set()
        for endpoint in observation.endpoints:
            ray_count += 1
            endpoint_key = voxel_key(endpoint, voxel_resolution)
            if endpoint_key in candidate_keys:
                frame_hits.add(endpoint_key)
            for key in traverse_voxels(
                observation.origin, endpoint, voxel_resolution
            ):
                if key in candidate_keys:
                    frame_passes.add(key)
        frame_passes.difference_update(frame_hits)
        for key in frame_hits:
            hit_frames[key] += 1
        for key in frame_passes:
            pass_frames[key] += 1

    removed_keys = {
        key
        for key in candidate_keys
        if pass_frames[key] >= min_pass_observations
        and pass_frames[key] > pass_to_hit_ratio * max(hit_frames[key], 1)
    }
    keep_mask = np.asarray(
        [key not in removed_keys for key in point_keys], dtype=bool
    )
    kept = xyz[keep_mask].astype(np.float32)
    covered_keys = sum(
        hit_frames[key] > 0 or pass_frames[key] > 0 for key in candidate_keys
    )
    evidence_rows = [
        {
            "voxel": list(key),
            "hit_observations": hit_frames[key],
            "pass_observations": pass_frames[key],
            "removed": key in removed_keys,
        }
        for key in sorted(candidate_keys)
        if hit_frames[key] > 0 or pass_frames[key] > 0
    ]
    report: dict[str, object] = {
        "schema": "rm_ray_evidence_cleanup_report/v1",
        "authority": "candidate_only_never_deployment",
        "voxel_resolution": voxel_resolution,
        "min_pass_observations": min_pass_observations,
        "pass_to_hit_ratio": pass_to_hit_ratio,
        "input_points": int(xyz.shape[0]),
        "kept_points": int(kept.shape[0]),
        "removed_points": int(xyz.shape[0] - kept.shape[0]),
        "candidate_voxels": len(candidate_keys),
        "covered_candidate_voxels": covered_keys,
        "coverage_ratio": covered_keys / len(candidate_keys),
        "removed_voxels": len(removed_keys),
        "observation_frames": observation_count,
        "rays": ray_count,
        "evidence": evidence_rows,
    }
    return CleanupResult(kept, report)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Create a new candidate PCD using timestamped ray evidence. "
            "A final merged PCD alone is intentionally insufficient."
        )
    )
    parser.add_argument(
        "--input-manifest",
        help=(
            "Validated occupancy_with_pcd map bundle containing both the PCD "
            "and ray_observations artifact. This is the safe field-data mode."
        ),
    )
    parser.add_argument(
        "--input-pcd",
        help="Direct PCD input for synthetic or legacy use only",
    )
    parser.add_argument(
        "--observations",
        help="Direct ray sidecar input for synthetic or legacy use only",
    )
    parser.add_argument("--report", required=True)
    parser.add_argument("--output-pcd")
    parser.add_argument("--write-candidate", action="store_true")
    parser.add_argument(
        "--allow-incomplete-observations",
        action="store_true",
        help=(
            "Explicitly permit candidate generation from a recorder sidecar "
            "whose header status is not complete. The unsafe override is "
            "recorded in the report."
        ),
    )
    parser.add_argument("--voxel-resolution", type=float, default=0.10)
    parser.add_argument("--min-pass-observations", type=int, default=4)
    parser.add_argument("--pass-to-hit-ratio", type=float, default=2.0)
    return parser


def _resolve_inputs(
    parsed: argparse.Namespace,
) -> tuple[
    Path,
    Path,
    Path | None,
    dict[str, object],
    dict[str, str] | None,
]:
    has_manifest = parsed.input_manifest is not None
    has_pcd = parsed.input_pcd is not None
    has_observations = parsed.observations is not None

    if has_manifest and (has_pcd or has_observations):
        raise SystemExit(
            "--input-manifest cannot be combined with --input-pcd or "
            "--observations"
        )
    if has_manifest:
        manifest_path = Path(parsed.input_manifest).expanduser().resolve()
        bundle = validate_map_bundle(manifest_path)
        if bundle["map_type"] != "occupancy_with_pcd":
            raise MapBundleError(
                "ray cleanup requires an occupancy_with_pcd map bundle"
            )
        pcd = bundle.get("pcd")
        if not isinstance(pcd, dict):
            raise MapBundleError(
                "ray cleanup requires a validated PCD artifact"
            )
        ray_observations = bundle.get("ray_observations")
        if not isinstance(ray_observations, dict):
            raise MapBundleError(
                "map bundle has no ray_observations artifact"
            )
        validated_manifest = Path(str(bundle["manifest"]))
        return (
            Path(str(pcd["path"])),
            Path(str(ray_observations["path"])),
            validated_manifest,
            {
                "input_manifest": str(validated_manifest),
                "input_manifest_sha256": _sha256(validated_manifest),
            },
            {
                "input_pcd": str(pcd["sha256"]),
                "observations": str(ray_observations["sha256"]),
            },
        )

    if has_pcd != has_observations:
        raise SystemExit(
            "--input-pcd and --observations must be provided together"
        )
    if not has_pcd:
        raise SystemExit(
            "provide --input-manifest, or provide both --input-pcd and "
            "--observations"
        )
    return (
        Path(parsed.input_pcd).expanduser().resolve(),
        Path(parsed.observations).expanduser().resolve(),
        None,
        {},
        None,
    )


def main(args: list[str] | None = None) -> int:
    parsed = build_parser().parse_args(args)
    if parsed.write_candidate and not parsed.output_pcd:
        raise SystemExit("--output-pcd is required with --write-candidate")
    if not parsed.write_candidate and parsed.output_pcd:
        raise SystemExit("--output-pcd requires explicit --write-candidate")

    (
        input_path,
        observations_path,
        manifest_path,
        input_report,
        expected_input_hashes,
    ) = _resolve_inputs(parsed)
    report_path = _resolved_output_path(parsed.report, "report")
    candidate_path = (
        _resolved_output_path(parsed.output_pcd, "candidate PCD")
        if parsed.write_candidate
        else None
    )
    role_paths = {
        "input PCD": input_path,
        "observations": observations_path,
        "report": report_path,
    }
    if manifest_path is not None:
        role_paths["input manifest"] = manifest_path
    if candidate_path is not None:
        role_paths["candidate PCD"] = candidate_path
    _require_distinct_paths(role_paths)

    points, pcd_metadata = read_ascii_xyz_pcd(input_path)
    with stream_ray_observations(observations_path) as (
        sidecar_header,
        observations,
    ):
        observations_status = sidecar_header.get("status")
        if (
            parsed.write_candidate
            and observations_status is not None
            and observations_status != "complete"
            and not parsed.allow_incomplete_observations
        ):
            raise ValueError(
                "recorder observation sidecar status must be 'complete' for "
                "candidate output; use --allow-incomplete-observations only "
                "for an explicitly reviewed experiment"
            )
        result = clean_with_ray_evidence(
            points,
            observations,
            parsed.voxel_resolution,
            parsed.min_pass_observations,
            parsed.pass_to_hit_ratio,
        )

    input_pcd_hash = _sha256(input_path)
    observations_hash = _sha256(observations_path)
    if expected_input_hashes is not None:
        actual_hashes = {
            "input_pcd": input_pcd_hash,
            "observations": observations_hash,
        }
        changed_inputs = [
            label
            for label, expected_hash in expected_input_hashes.items()
            if actual_hashes[label] != expected_hash
        ]
        if changed_inputs:
            raise MapBundleError(
                "map bundle input changed after validation: "
                + ", ".join(changed_inputs)
            )

    report_path = _new_output_path(parsed.report, "report")
    report = dict(result.report)
    report.update(
        {
            **input_report,
            "input_pcd": str(input_path),
            "input_pcd_sha256": input_pcd_hash,
            "input_pcd_metadata": pcd_metadata,
            "observations": str(observations_path),
            "observations_sha256": observations_hash,
            "observations_header": sidecar_header,
            "observations_status": observations_status or "legacy_unspecified",
            "allow_incomplete_observations": bool(
                parsed.allow_incomplete_observations
            ),
            "candidate_written": bool(parsed.write_candidate),
        }
    )
    if parsed.write_candidate:
        assert candidate_path is not None
        output_path = _new_output_path(parsed.output_pcd, "candidate PCD")
        if result.kept_points.shape[0] == 0:
            raise ValueError("cleanup would produce an empty candidate PCD")
        _publish_new_file(
            output_path,
            "candidate PCD",
            lambda path: write_ascii_pcd(path, result.kept_points),
        )
        report["output_pcd"] = str(output_path)
        report["output_pcd_sha256"] = _sha256(output_path)
    _write_new_text(
        report_path,
        "report",
        json.dumps(report, indent=2, sort_keys=True) + "\n",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
