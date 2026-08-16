"""Conservative ray-evidence cleanup for PCD candidates with frame sidecars."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable

import numpy as np

from .map_export import write_ascii_pcd
from .map_quality import read_ascii_xyz_pcd


SCHEMA = "rm_map_ray_observations/v1"
OUTPUT_FILE_MODE = 0o644


@dataclass(frozen=True)
class RayObservation:
    stamp: float
    origin: tuple[float, float, float]
    endpoints: tuple[tuple[float, float, float], ...]


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


def voxel_key(point: Iterable[float], resolution: float) -> tuple[int, int, int]:
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
    if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all():
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
    step = [1 if value > 0.0 else -1 if value < 0.0 else 0 for value in direction]
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


def load_ray_observations(
    path: str | Path,
) -> tuple[dict[str, object], list[RayObservation]]:
    source = Path(path)
    observations: list[RayObservation] = []
    with source.open("r", encoding="utf-8") as stream:
        lines = [line for line in stream if line.strip()]
    if not lines:
        raise ValueError("ray observation sidecar is empty")
    header = json.loads(lines[0])
    if header.get("schema") != SCHEMA or header.get("frame_id") != "map":
        raise ValueError(f"expected {SCHEMA!r} sidecar in canonical map frame")
    previous_stamp: float | None = None
    for line_number, line in enumerate(lines[1:], start=2):
        record = json.loads(line)
        if record.get("type") != "observation":
            raise ValueError(f"line {line_number}: expected observation record")
        stamp = float(record["stamp"])
        origin = tuple(float(value) for value in record["origin"])
        endpoints = tuple(
            tuple(float(value) for value in point) for point in record["endpoints"]
        )
        if (
            not math.isfinite(stamp)
            or stamp <= 0.0
            or len(origin) != 3
            or not all(math.isfinite(value) for value in origin)
            or not endpoints
            or any(
                len(point) != 3 or not all(math.isfinite(value) for value in point)
                for point in endpoints
            )
        ):
            raise ValueError(f"line {line_number}: invalid timestamp or XYZ data")
        if previous_stamp is not None and stamp <= previous_stamp:
            raise ValueError(
                f"line {line_number}: timestamps must be strictly increasing"
            )
        previous_stamp = stamp
        observations.append(RayObservation(stamp, origin, endpoints))
    if not observations:
        raise ValueError("ray observation sidecar contains no observations")
    return header, observations


def clean_with_ray_evidence(
    points: np.ndarray,
    observations: Iterable[RayObservation],
    voxel_resolution: float,
    min_pass_observations: int,
    pass_to_hit_ratio: float,
) -> CleanupResult:
    """Remove candidate voxels repeatedly traversed after their endpoint hits."""
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
    keep_mask = np.asarray([key not in removed_keys for key in point_keys], dtype=bool)
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


def _resolved_output_path(value: str, label: str) -> Path:
    requested_path = Path(value).expanduser()
    if requested_path.is_symlink():
        raise ValueError(
            f"{label} is a symlink; refusing indirect output: {requested_path}"
        )
    return requested_path.resolve()


def _new_output_path(value: str, label: str) -> Path:
    path = _resolved_output_path(value, label)
    if path.exists():
        raise ValueError(f"{label} already exists; refusing overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _require_distinct_paths(paths: dict[str, Path]) -> None:
    labels = tuple(paths)
    for index, left_label in enumerate(labels):
        for right_label in labels[index + 1 :]:
            if paths[left_label] == paths[right_label]:
                raise ValueError(
                    f"{left_label} and {right_label} paths must be distinct: "
                    f"{paths[left_label]}"
                )


def _publish_new_file(
    destination: Path,
    label: str,
    writer: Callable[[Path], object],
) -> None:
    """Atomically publish a complete file without replacing an existing path."""
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.",
        suffix=".tmp",
        dir=destination.parent,
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        writer(temporary_path)
        os.chmod(temporary_path, OUTPUT_FILE_MODE)
        with temporary_path.open("rb") as stream:
            os.fsync(stream.fileno())
        try:
            # A hard link is an atomic no-replace publication on the same
            # filesystem. The sibling staging file guarantees that condition.
            os.link(temporary_path, destination)
        except FileExistsError as exc:
            raise ValueError(
                f"{label} appeared while writing; refusing overwrite: {destination}"
            ) from exc
        directory_descriptor = os.open(
            destination.parent,
            os.O_RDONLY | os.O_DIRECTORY,
        )
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        temporary_path.unlink(missing_ok=True)


def _write_new_text(destination: Path, label: str, content: str) -> None:
    def write_staged(path: Path) -> None:
        path.write_text(content, encoding="utf-8", newline="\n")

    _publish_new_file(destination, label, write_staged)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Create a new candidate PCD using timestamped ray evidence. "
            "A final merged PCD alone is intentionally insufficient."
        )
    )
    parser.add_argument("--input-pcd", required=True)
    parser.add_argument("--observations", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--output-pcd")
    parser.add_argument("--write-candidate", action="store_true")
    parser.add_argument("--voxel-resolution", type=float, default=0.10)
    parser.add_argument("--min-pass-observations", type=int, default=4)
    parser.add_argument("--pass-to-hit-ratio", type=float, default=2.0)
    return parser


def main(args: list[str] | None = None) -> int:
    parsed = build_parser().parse_args(args)
    if parsed.write_candidate and not parsed.output_pcd:
        raise SystemExit("--output-pcd is required with --write-candidate")
    if not parsed.write_candidate and parsed.output_pcd:
        raise SystemExit("--output-pcd requires explicit --write-candidate")

    input_path = Path(parsed.input_pcd).expanduser().resolve()
    observations_path = Path(parsed.observations).expanduser().resolve()
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
    if candidate_path is not None:
        role_paths["candidate PCD"] = candidate_path
    _require_distinct_paths(role_paths)

    points, pcd_metadata = read_ascii_xyz_pcd(input_path)
    sidecar_header, observations = load_ray_observations(observations_path)
    result = clean_with_ray_evidence(
        points,
        observations,
        parsed.voxel_resolution,
        parsed.min_pass_observations,
        parsed.pass_to_hit_ratio,
    )
    report_path = _new_output_path(parsed.report, "report")
    report = dict(result.report)
    report.update(
        {
            "input_pcd": str(input_path),
            "input_pcd_sha256": _sha256(input_path),
            "input_pcd_metadata": pcd_metadata,
            "observations": str(observations_path),
            "observations_sha256": _sha256(observations_path),
            "observations_header": sidecar_header,
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
