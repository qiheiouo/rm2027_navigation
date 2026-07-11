from __future__ import annotations

import hashlib
import math
import os
import re
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from .map_bundle import MapBundleError, validate_map_bundle


_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


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
) -> Path:
    clean_map_id = _checked_identifier(map_id, "map_id")
    clean_revision = _checked_identifier(revision, "revision")
    if not math.isfinite(occupancy_resolution) or occupancy_resolution <= 0.0:
        raise MapBundleError("occupancy resolution must be positive and finite")
    origin = [float(value) for value in occupancy_origin]
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise MapBundleError("occupancy origin must contain x, y and yaw")
    if not source_method.strip():
        raise MapBundleError("source_method must not be empty")

    root = Path(output_root).expanduser().resolve()
    target = root / clean_map_id / clean_revision
    if target.exists():
        raise MapBundleError(f"map bundle already exists: {target}")
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{clean_revision}-", dir=target.parent))

    stem = clean_map_id
    pcd_path = staging / f"{stem}.pcd"
    pgm_path = staging / f"{stem}.pgm"
    occupancy_yaml_path = staging / f"{stem}.yaml"
    manifest_path = staging / f"{stem}.bundle.yaml"

    try:
        write_ascii_pcd(pcd_path, points)
        pgm_path.write_bytes(
            occupancy_values_to_pgm(
                occupancy_values,
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
            "free_thresh": 0.25,
        }
        occupancy_yaml_path.write_text(
            yaml.safe_dump(occupancy_yaml, sort_keys=False),
            encoding="utf-8",
            newline="\n",
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
        manifest_path.write_text(
            yaml.safe_dump(manifest, sort_keys=False),
            encoding="utf-8",
            newline="\n",
        )

        validate_map_bundle(manifest_path)
        os.replace(staging, target)
        return target / manifest_path.name
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
