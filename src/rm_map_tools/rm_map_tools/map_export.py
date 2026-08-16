from __future__ import annotations

import hashlib
import math
import re
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from .map_bundle import MapBundleError, validate_map_bundle
from .immutable_output import (
    fsync_directory,
    fsync_file,
    path_entry_exists,
    publish_new_directory,
)
from .ray_observations import (
    SOURCE_STAMP_SEMANTICS,
    RayObservationError,
    SCHEMA as RAY_OBSERVATION_SCHEMA,
    physical_source_frame,
    stream_ray_observations,
)


_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_CAPTURE_ID_PATTERN = re.compile(r"^[0-9a-f]{32}$")


def _checked_identifier(value: str, label: str) -> str:
    candidate = value.strip()
    if not _IDENTIFIER_PATTERN.fullmatch(candidate):
        raise MapBundleError(
            f"{label} must start with an alphanumeric character and contain "
            "only alphanumerics, '.', '_' or '-'"
        )
    return candidate


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _as_xyz(points: Iterable[Iterable[float]] | np.ndarray) -> np.ndarray:
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] == 0:
        raise MapBundleError("PCD export requires a non-empty Nx3 point array")
    if not np.isfinite(xyz).all():
        raise MapBundleError("PCD export contains non-finite coordinates")
    return xyz


def _ray_observation_metadata(path: Path) -> dict[str, Any]:
    try:
        with stream_ray_observations(path) as (header, observations):
            try:
                source_frame = physical_source_frame(
                    header.get("source_frame"),
                    label="ray observations header source_frame",
                )
            except RayObservationError as exc:
                raise MapBundleError(str(exc)) from exc
            stamp_semantics = header.get("stamp_semantics")
            if stamp_semantics != SOURCE_STAMP_SEMANTICS:
                raise MapBundleError(
                    "ray observations header stamp_semantics must be "
                    + repr(SOURCE_STAMP_SEMANTICS)
                )
            status = header.get("status")
            if status not in {"complete", "incomplete"}:
                raise MapBundleError(
                    "ray observations status must be 'complete' or 'incomplete'"
                )
            statistics = header.get("statistics")
            if not isinstance(statistics, dict):
                raise MapBundleError(
                    "ray observations header statistics must be a mapping"
                )
            expected_frames = statistics.get("accepted_frames")
            expected_rays = statistics.get("total_rays")
            if (
                isinstance(expected_frames, bool)
                or not isinstance(expected_frames, int)
                or expected_frames <= 0
                or isinstance(expected_rays, bool)
                or not isinstance(expected_rays, int)
                or expected_rays <= 0
            ):
                raise MapBundleError(
                    "ray observations header statistics must contain positive "
                    "accepted_frames and total_rays"
                )
            header_metadata = header.get("metadata")
            if not isinstance(header_metadata, dict):
                raise MapBundleError(
                    "ray observations header metadata must be a mapping"
                )
            capture_id = header_metadata.get("capture_id")
            if not isinstance(capture_id, str) or not _CAPTURE_ID_PATTERN.fullmatch(
                capture_id
            ):
                raise MapBundleError(
                    "ray observations metadata.capture_id must be 32 lowercase "
                    "hexadecimal characters"
                )
            observation_frames = 0
            rays = 0
            for observation in observations:
                if observation.source_stamp_ns is None:
                    raise MapBundleError(
                        "ray observations payload is missing source_stamp_ns"
                    )
                observation_frames += 1
                rays += len(observation.endpoints)
    except MapBundleError:
        raise
    except (OSError, RayObservationError) as exc:
        raise MapBundleError(f"invalid ray observations sidecar: {exc}") from exc
    if observation_frames != expected_frames or rays != expected_rays:
        raise MapBundleError(
            "ray observations payload does not match header statistics"
        )
    return {
        "schema": RAY_OBSERVATION_SCHEMA,
        "frame_id": "map",
        "source_frame": source_frame,
        "stamp_semantics": stamp_semantics,
        "status": status,
        "capture_id": capture_id,
        "bytes": path.stat().st_size,
        "observation_frames": observation_frames,
        "rays": rays,
    }


def write_ascii_pcd(path: str | Path, points: Iterable[Iterable[float]] | np.ndarray) -> None:
    destination = Path(path)
    xyz = _as_xyz(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION .7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {xyz.shape[0]}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {xyz.shape[0]}\n"
        "DATA ascii\n"
    )
    with destination.open("w", encoding="ascii", newline="\n") as stream:
        stream.write(header)
        np.savetxt(stream, xyz, fmt="%.6f %.6f %.6f")


def occupancy_values_to_pgm(
    values: Iterable[int],
    width: int,
    height: int,
    free_threshold: int = 25,
    occupied_threshold: int = 65,
) -> bytes:
    if width <= 0 or height <= 0:
        raise MapBundleError("occupancy dimensions must be positive")
    cells = np.asarray(list(values), dtype=np.int16)
    if cells.size != width * height:
        raise MapBundleError("occupancy data length does not match width*height")
    if not 0 <= free_threshold < occupied_threshold <= 100:
        raise MapBundleError("occupancy thresholds must satisfy 0 <= free < occupied <= 100")

    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[(cells >= 0) & (cells <= free_threshold)] = 254
    pixels[cells >= occupied_threshold] = 0
    image = pixels.reshape((height, width))[::-1, :]
    return f"P5\n{width} {height}\n255\n".encode("ascii") + image.tobytes()


def write_candidate_map_bundle(
    *,
    output_root: str | Path,
    map_id: str,
    revision: str,
    points: Iterable[Iterable[float]] | np.ndarray,
    occupancy_values: Iterable[int],
    occupancy_width: int,
    occupancy_height: int,
    occupancy_resolution: float,
    occupancy_origin: Iterable[float],
    source_method: str,
    source_details: dict[str, Any] | None = None,
    created_utc: str | None = None,
    ray_observations_path: str | Path | None = None,
) -> Path:
    clean_map_id = _checked_identifier(map_id, "map_id")
    clean_revision = _checked_identifier(revision, "revision")
    occupancy_cells = list(occupancy_values)
    if not any(value >= 0 for value in occupancy_cells):
        raise MapBundleError(
            "candidate occupancy grid has no known cells; refuse an all-unknown map"
        )
    if not math.isfinite(occupancy_resolution) or occupancy_resolution <= 0.0:
        raise MapBundleError("occupancy resolution must be positive and finite")
    origin = [float(value) for value in occupancy_origin]
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise MapBundleError("occupancy origin must contain x, y and yaw")
    if not source_method.strip():
        raise MapBundleError("source_method must not be empty")

    root = Path(output_root).expanduser().resolve()
    target = root / clean_map_id / clean_revision
    if path_entry_exists(target):
        raise MapBundleError(f"map bundle already exists: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{clean_revision}-", dir=target.parent))

    stem = clean_map_id
    pcd_path = staging / f"{stem}.pcd"
    pgm_path = staging / f"{stem}.pgm"
    occupancy_yaml_path = staging / f"{stem}.yaml"
    manifest_path = staging / f"{stem}.bundle.yaml"
    ray_sidecar_path = staging / f"{stem}.rays.jsonl"

    try:
        write_ascii_pcd(pcd_path, points)
        pgm_path.write_bytes(
            occupancy_values_to_pgm(
                occupancy_cells,
                occupancy_width,
                occupancy_height,
            )
        )
        occupancy_yaml = {
            "image": pgm_path.name,
            "mode": "trinary",
            "resolution": float(occupancy_resolution),
            "origin": origin,
            "negate": 0,
            "occupied_thresh": 0.65,
            # PGM unknown cells are encoded as 205, whose occupancy
            # probability is 50/255 ~= 0.196078 for negate=0. Keep the free
            # threshold just below that value so map_server reloads them as
            # unknown instead of silently converting them to free space.
            "free_thresh": 0.196,
        }
        occupancy_yaml_path.write_text(
            yaml.safe_dump(occupancy_yaml, sort_keys=False),
            encoding="utf-8",
            newline="\n",
        )

        ray_observations_metadata = None
        if ray_observations_path is not None:
            source_ray_sidecar = Path(ray_observations_path).expanduser().resolve()
            # Validate the bounded source before copying so an oversized or
            # malformed caller-provided file cannot consume staging disk. The
            # copied bytes are validated again below to close mutation races.
            source_ray_metadata = _ray_observation_metadata(source_ray_sidecar)
            source_ray_hash = _sha256(source_ray_sidecar)
            try:
                shutil.copyfile(source_ray_sidecar, ray_sidecar_path)
            except OSError as exc:
                raise MapBundleError(
                    f"cannot copy ray observations sidecar: {source_ray_sidecar}"
                ) from exc
            ray_observations_metadata = _ray_observation_metadata(
                ray_sidecar_path
            )
            if (
                ray_observations_metadata != source_ray_metadata
                or _sha256(ray_sidecar_path) != source_ray_hash
            ):
                raise MapBundleError(
                    "ray observations sidecar changed while being copied"
                )
            ray_source_details = (
                source_details.get("ray_observations")
                if isinstance(source_details, dict)
                else None
            )
            expected_capture_id = (
                ray_source_details.get("capture_id")
                if isinstance(ray_source_details, dict)
                else None
            )
            if expected_capture_id != ray_observations_metadata["capture_id"]:
                raise MapBundleError(
                    "ray observations capture_id must match "
                    "source_details.ray_observations.capture_id"
                )
            expected_source_frame = (
                ray_source_details.get("source_frame")
                if isinstance(ray_source_details, dict)
                else None
            )
            if expected_source_frame != ray_observations_metadata["source_frame"]:
                raise MapBundleError(
                    "ray observations source_frame must match "
                    "source_details.ray_observations.source_frame"
                )

        source: dict[str, Any] = {
            "method": source_method.strip(),
            "created_utc": created_utc
            or datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }
        if source_details:
            source["details"] = source_details

        manifest = {
            "schema_version": 1,
            "map_id": clean_map_id,
            "revision": clean_revision,
            "deployment_status": "candidate",
            "frame_id": "map",
            "source": source,
            "artifacts": {
                "pcd": {
                    "path": pcd_path.name,
                    "frame_id": "map",
                    "sha256": _sha256(pcd_path),
                },
                "occupancy": {
                    "yaml_path": occupancy_yaml_path.name,
                    "image_path": pgm_path.name,
                    "frame_id": "map",
                    "yaml_sha256": _sha256(occupancy_yaml_path),
                    "image_sha256": _sha256(pgm_path),
                },
            },
            "alignment": {
                "pcd_and_occupancy_share_map_origin": True,
                "review_status": "requires_human_landmark_review",
            },
        }
        if ray_observations_metadata is not None:
            manifest["artifacts"]["ray_observations"] = {
                "path": ray_sidecar_path.name,
                **ray_observations_metadata,
                "sha256": _sha256(ray_sidecar_path),
                "authority": "offline_evidence_only",
            }
        manifest_path.write_text(
            yaml.safe_dump(manifest, sort_keys=False),
            encoding="utf-8",
            newline="\n",
        )

        validate_map_bundle(manifest_path)
        artifact_paths = [
            pcd_path,
            pgm_path,
            occupancy_yaml_path,
            manifest_path,
        ]
        if ray_observations_metadata is not None:
            artifact_paths.append(ray_sidecar_path)
        for artifact_path in artifact_paths:
            fsync_file(artifact_path)
        fsync_directory(staging)
        try:
            publish_new_directory(staging, target, "map bundle")
        except ValueError as exc:
            raise MapBundleError(str(exc)) from exc
        return target / manifest_path.name
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
