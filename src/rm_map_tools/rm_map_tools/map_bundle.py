from __future__ import annotations

import hashlib
import math
import re
from pathlib import Path
from typing import Any

import yaml

from .ray_observations import (
    SOURCE_STAMP_SEMANTICS,
    RayObservationError,
    SCHEMA as RAY_OBSERVATION_SCHEMA,
    physical_source_frame,
    stream_ray_observations,
)


class MapBundleError(ValueError):
    pass


_MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_CAPTURE_ID_PATTERN = re.compile(r"^[0-9a-f]{32}$")
_DEPLOYMENT_STATES = {"test_only", "candidate", "approved"}
_MAP_TYPES = {"occupancy_only", "occupancy_with_pcd"}
_RUNTIME_ACCEPTANCE_POLICIES = {
    "approved_only": {"approved"},
    "allow_candidate": {"candidate", "approved"},
    "allow_test": _DEPLOYMENT_STATES,
}


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise MapBundleError(f"cannot read YAML '{path}': {exc}") from exc
    if not isinstance(data, dict):
        raise MapBundleError(f"YAML root must be a mapping: {path}")
    return data


def _required_mapping(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        raise MapBundleError(f"'{key}' must be a mapping")
    return value


def _required_string(parent: dict[str, Any], key: str) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not value.strip():
        raise MapBundleError(f"'{key}' must be a non-empty string")
    return value.strip()


def _resolve_artifact(
    root: Path,
    relative_path: str,
    label: str,
    containment_root: Path | None = None,
) -> Path:
    relative = Path(relative_path)
    if relative.is_absolute():
        raise MapBundleError(f"{label} path must be relative")
    resolved = (root / relative).resolve()
    allowed_root = root if containment_root is None else containment_root
    try:
        resolved.relative_to(allowed_root)
    except ValueError as exc:
        raise MapBundleError(f"{label} path escapes the map bundle") from exc
    if not resolved.is_file():
        raise MapBundleError(f"{label} does not exist: {relative_path}")
    return resolved


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_hash(path: Path, expected: str, label: str) -> str:
    if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected):
        raise MapBundleError(f"{label} sha256 must contain 64 hexadecimal characters")
    actual = _sha256(path)
    if actual.lower() != expected.lower():
        raise MapBundleError(
            f"{label} sha256 mismatch: expected {expected.lower()}, got {actual}"
        )
    return actual


def _required_positive_integer(parent: dict[str, Any], key: str) -> int:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise MapBundleError(f"'{key}' must be a positive integer")
    return value


def _required_capture_id(
    parent: dict[str, Any], key: str, label: str
) -> str:
    value = parent.get(key)
    if not isinstance(value, str) or not _CAPTURE_ID_PATTERN.fullmatch(value):
        raise MapBundleError(
            f"{label} must be 32 lowercase hexadecimal characters"
        )
    return value


def _parse_ray_observations(path: Path) -> dict[str, Any]:
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
            expected_frames = _required_positive_integer(
                statistics, "accepted_frames"
            )
            expected_rays = _required_positive_integer(statistics, "total_rays")
            metadata = header.get("metadata")
            if not isinstance(metadata, dict):
                raise MapBundleError(
                    "ray observations header metadata must be a mapping"
                )
            capture_id = _required_capture_id(
                metadata,
                "capture_id",
                "ray observations metadata.capture_id",
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
        "observation_frames": observation_frames,
        "rays": rays,
        "bytes": path.stat().st_size,
    }


def _parse_pcd(path: Path) -> dict[str, Any]:
    header: dict[str, list[str]] = {}
    data_offset = 0
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise MapBundleError("PCD header has no DATA entry")
            data_offset = stream.tell()
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise MapBundleError("PCD header must be ASCII") from exc
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
            if data_offset > 65536:
                raise MapBundleError("PCD header exceeds 64 KiB")

    fields = header.get("FIELDS", [])
    if not {"x", "y", "z"}.issubset(fields):
        raise MapBundleError("PCD FIELDS must include x, y and z")
    version = header.get("VERSION", [""])[0]
    if version not in {".7", "0.7"}:
        raise MapBundleError(f"unsupported PCD VERSION: {version}")
    try:
        points = int(header["POINTS"][0])
        width = int(header["WIDTH"][0])
        height = int(header["HEIGHT"][0])
    except (KeyError, IndexError, ValueError) as exc:
        raise MapBundleError("PCD requires integer POINTS, WIDTH and HEIGHT") from exc
    if points <= 0 or width <= 0 or height <= 0 or width * height != points:
        raise MapBundleError("PCD dimensions must be positive and WIDTH*HEIGHT must equal POINTS")

    data_mode = header.get("DATA", [""])[0].lower()
    if data_mode not in {"ascii", "binary", "binary_compressed"}:
        raise MapBundleError(f"unsupported PCD DATA mode: {data_mode}")

    if data_mode == "ascii":
        row_count = 0
        coordinate_indices = [fields.index(axis) for axis in ("x", "y", "z")]
        with path.open("rb") as stream:
            stream.seek(data_offset)
            for raw_line in stream:
                try:
                    row = raw_line.decode("ascii").strip()
                except UnicodeDecodeError as exc:
                    raise MapBundleError("ASCII PCD payload is not ASCII") from exc
                if not row or row.startswith("#"):
                    continue
                values = row.split()
                if len(values) < len(fields):
                    raise MapBundleError("ASCII PCD row has fewer values than FIELDS")
                try:
                    coordinates = [float(values[index]) for index in coordinate_indices]
                except ValueError as exc:
                    raise MapBundleError("ASCII PCD coordinate is not numeric") from exc
                if not all(math.isfinite(value) for value in coordinates):
                    raise MapBundleError("ASCII PCD contains a non-finite coordinate")
                row_count += 1
        if row_count != points:
            raise MapBundleError(
                f"ASCII PCD row count {row_count} does not match POINTS {points}"
            )
    elif path.stat().st_size <= data_offset:
        raise MapBundleError("binary PCD payload is empty")

    return {
        "fields": fields,
        "version": version,
        "points": points,
        "width": width,
        "height": height,
        "data_mode": data_mode,
    }


def _pgm_tokens(data: bytes) -> list[bytes]:
    tokens: list[bytes] = []
    for line in data.splitlines():
        content = line.split(b"#", 1)[0]
        tokens.extend(content.split())
    return tokens


def _pgm_header(data: bytes) -> tuple[list[bytes], int]:
    tokens: list[bytes] = []
    index = 0
    while len(tokens) < 4:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            newline = data.find(b"\n", index)
            if newline < 0:
                break
            index = newline + 1
            continue
        start = index
        while index < len(data) and data[index] not in b" \t\r\n#":
            index += 1
        if start == index:
            break
        tokens.append(data[start:index])

    if len(tokens) < 4 or index >= len(data) or data[index] not in b" \t\r\n":
        raise MapBundleError("PGM header is incomplete")
    if data[index] == ord("\r") and index + 1 < len(data) and data[index + 1] == ord("\n"):
        return tokens, index + 2
    return tokens, index + 1


def _parse_pgm(path: Path) -> dict[str, int | str]:
    data = path.read_bytes()
    tokens, payload_offset = _pgm_header(data)
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise MapBundleError("occupancy image must be a P2 or P5 PGM")
    try:
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
    except ValueError as exc:
        raise MapBundleError("PGM dimensions and max value must be integers") from exc
    if width <= 0 or height <= 0 or not 0 < max_value <= 65535:
        raise MapBundleError("PGM dimensions or max value are invalid")
    if tokens[0] == b"P2":
        payload_tokens = _pgm_tokens(data[payload_offset:])
        if len(payload_tokens) != width * height:
            raise MapBundleError("P2 PGM pixel count does not match its dimensions")
        try:
            pixels = [int(token) for token in payload_tokens]
        except ValueError as exc:
            raise MapBundleError("P2 PGM pixels must be integers") from exc
        if any(pixel < 0 or pixel > max_value for pixel in pixels):
            raise MapBundleError("P2 PGM pixel is outside the declared range")
    else:
        bytes_per_pixel = 1 if max_value < 256 else 2
        expected_size = width * height * bytes_per_pixel
        actual_size = len(data) - payload_offset
        if actual_size != expected_size:
            raise MapBundleError(
                f"P5 PGM payload size {actual_size} does not match expected {expected_size}"
            )
    return {
        "format": tokens[0].decode("ascii"),
        "width": width,
        "height": height,
        "max_value": max_value,
    }


def _validate_occupancy_yaml(path: Path, bundle_root: Path) -> tuple[dict[str, Any], Path]:
    data = _load_yaml(path)
    image_value = _required_string(data, "image")
    image_path = _resolve_artifact(
        path.parent,
        image_value,
        "occupancy image",
        containment_root=bundle_root,
    )

    resolution = data.get("resolution")
    origin = data.get("origin")
    free_threshold = data.get("free_thresh")
    occupied_threshold = data.get("occupied_thresh")
    negate = data.get("negate")
    mode = data.get("mode", "trinary")
    if (
        not isinstance(resolution, (int, float))
        or not math.isfinite(resolution)
        or resolution <= 0
    ):
        raise MapBundleError("occupancy resolution must be a positive finite number")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(isinstance(value, (int, float)) and math.isfinite(value) for value in origin)
    ):
        raise MapBundleError("occupancy origin must contain three finite numbers")
    if not isinstance(negate, int) or negate not in {0, 1}:
        raise MapBundleError("occupancy negate must be 0 or 1")
    if mode not in {"trinary", "scale", "raw"}:
        raise MapBundleError("occupancy mode must be trinary, scale or raw")
    if not all(isinstance(value, (int, float)) for value in [free_threshold, occupied_threshold]):
        raise MapBundleError("occupancy thresholds must be numeric")
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise MapBundleError("occupancy thresholds must satisfy 0 <= free < occupied <= 1")
    return data, image_path


def validate_map_bundle(
    manifest_path: str | Path,
    require_approved: bool = False,
    *,
    validate_offline_artifacts: bool = True,
) -> dict[str, Any]:
    """Validate map artifacts and, by default, offline evidence artifacts.

    ``validate_offline_artifacts=False`` is reserved for runtime resolution,
    where offline-only evidence is neither authoritative nor consumed.
    """
    manifest = Path(manifest_path).resolve()
    if not manifest.is_file():
        raise MapBundleError(f"manifest does not exist: {manifest}")
    root = manifest.parent.resolve()
    data = _load_yaml(manifest)

    schema_version = data.get("schema_version")
    if schema_version not in {1, 2}:
        raise MapBundleError("schema_version must be 1 or 2")
    if schema_version == 1:
        map_type = "occupancy_with_pcd"
    else:
        map_type = _required_string(data, "map_type")
        if map_type not in _MAP_TYPES:
            raise MapBundleError(f"unsupported map_type: {map_type}")
    map_id = _required_string(data, "map_id")
    if not _MAP_ID_PATTERN.fullmatch(map_id):
        raise MapBundleError("map_id contains unsupported characters")
    revision = _required_string(data, "revision")
    frame_id = _required_string(data, "frame_id")
    if frame_id != "map":
        raise MapBundleError("frame_id must be canonical 'map'")
    deployment_status = _required_string(data, "deployment_status")
    if deployment_status not in _DEPLOYMENT_STATES:
        raise MapBundleError(f"unsupported deployment_status: {deployment_status}")
    if require_approved and deployment_status != "approved":
        raise MapBundleError("map bundle is not approved for deployment")

    source = _required_mapping(data, "source")
    _required_string(source, "method")
    _required_string(source, "created_utc")
    artifacts = _required_mapping(data, "artifacts")
    occupancy = _required_mapping(artifacts, "occupancy")

    if _required_string(occupancy, "frame_id") != frame_id:
        raise MapBundleError("occupancy frame_id must match bundle frame_id")

    pcd_metadata = None
    if map_type == "occupancy_with_pcd":
        pcd = _required_mapping(artifacts, "pcd")
        if _required_string(pcd, "frame_id") != frame_id:
            raise MapBundleError("PCD frame_id must match bundle frame_id")
        pcd_path = _resolve_artifact(root, _required_string(pcd, "path"), "PCD")
        pcd_hash = _verify_hash(pcd_path, pcd.get("sha256"), "PCD")
        pcd_metadata = {
            **_parse_pcd(pcd_path),
            "path": str(pcd_path),
            "sha256": pcd_hash,
        }
    elif "pcd" in artifacts:
        raise MapBundleError("occupancy_only bundle must not contain a PCD artifact")

    ray_observations_metadata = None
    if validate_offline_artifacts and "ray_observations" in artifacts:
        ray_observations = _required_mapping(artifacts, "ray_observations")
        declared_capture_id = _required_capture_id(
            ray_observations,
            "capture_id",
            "ray observations artifact.capture_id",
        )
        source_details = source.get("details")
        if not isinstance(source_details, dict):
            raise MapBundleError(
                "source.details.ray_observations must be a mapping when "
                "ray observations are present"
            )
        source_ray_observations = source_details.get("ray_observations")
        if not isinstance(source_ray_observations, dict):
            raise MapBundleError(
                "source.details.ray_observations must be a mapping when "
                "ray observations are present"
            )
        source_capture_id = _required_capture_id(
            source_ray_observations,
            "capture_id",
            "source.details.ray_observations.capture_id",
        )
        try:
            declared_source_frame = physical_source_frame(
                ray_observations.get("source_frame"),
                label="ray observations artifact.source_frame",
            )
            source_source_frame = physical_source_frame(
                source_ray_observations.get("source_frame"),
                label="source.details.ray_observations.source_frame",
            )
        except RayObservationError as exc:
            raise MapBundleError(str(exc)) from exc
        if _required_string(ray_observations, "schema") != RAY_OBSERVATION_SCHEMA:
            raise MapBundleError(
                "ray observations schema must be " + repr(RAY_OBSERVATION_SCHEMA)
            )
        if _required_string(ray_observations, "frame_id") != frame_id:
            raise MapBundleError(
                "ray observations frame_id must match bundle frame_id"
            )
        if (
            _required_string(ray_observations, "stamp_semantics")
            != SOURCE_STAMP_SEMANTICS
        ):
            raise MapBundleError(
                "ray observations stamp_semantics must be "
                + repr(SOURCE_STAMP_SEMANTICS)
            )
        declared_status = _required_string(ray_observations, "status")
        if declared_status not in {"complete", "incomplete"}:
            raise MapBundleError(
                "ray observations status must be 'complete' or 'incomplete'"
            )
        if _required_string(ray_observations, "authority") != "offline_evidence_only":
            raise MapBundleError(
                "ray observations authority must be 'offline_evidence_only'"
            )
        ray_observations_path = _resolve_artifact(
            root,
            _required_string(ray_observations, "path"),
            "ray observations",
        )
        ray_observations_hash = _verify_hash(
            ray_observations_path,
            ray_observations.get("sha256"),
            "ray observations",
        )
        declared_bytes = _required_positive_integer(ray_observations, "bytes")
        declared_frames = _required_positive_integer(
            ray_observations, "observation_frames"
        )
        declared_rays = _required_positive_integer(ray_observations, "rays")
        ray_observations_metadata = _parse_ray_observations(
            ray_observations_path
        )
        capture_ids = {
            ray_observations_metadata["capture_id"],
            declared_capture_id,
            source_capture_id,
        }
        if len(capture_ids) != 1:
            raise MapBundleError(
                "ray observations capture_id mismatch across sidecar metadata, "
                "manifest artifact and source.details"
            )
        source_frames = {
            ray_observations_metadata["source_frame"],
            declared_source_frame,
            source_source_frame,
        }
        if len(source_frames) != 1:
            raise MapBundleError(
                "ray observations source_frame mismatch across sidecar header, "
                "manifest artifact and source.details"
            )
        if (
            ray_observations_metadata["stamp_semantics"]
            != ray_observations["stamp_semantics"]
        ):
            raise MapBundleError(
                "ray observations stamp_semantics does not match sidecar header"
            )
        if ray_observations_metadata["status"] != declared_status:
            raise MapBundleError(
                "ray observations status does not match manifest"
            )
        if ray_observations_metadata["bytes"] != declared_bytes:
            raise MapBundleError(
                "ray observations byte count does not match manifest"
            )
        if ray_observations_metadata["observation_frames"] != declared_frames:
            raise MapBundleError(
                "ray observations frame count does not match manifest"
            )
        if ray_observations_metadata["rays"] != declared_rays:
            raise MapBundleError(
                "ray observations ray count does not match manifest"
            )
        ray_observations_metadata.update(
            {
                "path": str(ray_observations_path),
                "sha256": ray_observations_hash,
                "authority": "offline_evidence_only",
            }
        )

    occupancy_yaml_path = _resolve_artifact(
        root, _required_string(occupancy, "yaml_path"), "occupancy YAML"
    )
    occupancy_image_manifest = _resolve_artifact(
        root, _required_string(occupancy, "image_path"), "occupancy image"
    )

    occupancy_yaml_hash = _verify_hash(
        occupancy_yaml_path, occupancy.get("yaml_sha256"), "occupancy YAML"
    )
    occupancy_image_hash = _verify_hash(
        occupancy_image_manifest, occupancy.get("image_sha256"), "occupancy image"
    )

    occupancy_yaml, occupancy_image_from_yaml = _validate_occupancy_yaml(
        occupancy_yaml_path, root
    )
    if occupancy_image_from_yaml != occupancy_image_manifest:
        raise MapBundleError("occupancy YAML image does not match manifest image_path")
    pgm_metadata = _parse_pgm(occupancy_image_manifest)

    alignment = _required_mapping(data, "alignment")
    shared_origin = None
    occupancy_origin_reviewed = None
    if map_type == "occupancy_with_pcd":
        shared_origin = alignment.get("pcd_and_occupancy_share_map_origin")
        if not isinstance(shared_origin, bool):
            raise MapBundleError("alignment shared-origin flag must be boolean")
        if (require_approved or deployment_status == "approved") and not shared_origin:
            raise MapBundleError("approved map bundle must confirm a shared map origin")
    else:
        occupancy_origin_reviewed = alignment.get("occupancy_origin_reviewed")
        if not isinstance(occupancy_origin_reviewed, bool):
            raise MapBundleError("occupancy_only alignment review flag must be boolean")
        if (
            require_approved or deployment_status == "approved"
        ) and not occupancy_origin_reviewed:
            raise MapBundleError("approved occupancy map must confirm origin review")

    result = {
        "manifest": str(manifest),
        "map_id": map_id,
        "revision": revision,
        "schema_version": schema_version,
        "map_type": map_type,
        "frame_id": frame_id,
        "deployment_status": deployment_status,
        "shared_origin_confirmed": shared_origin,
        "occupancy_origin_reviewed": occupancy_origin_reviewed,
        "pcd": pcd_metadata,
        "occupancy": {
            "yaml_path": str(occupancy_yaml_path),
            "image_path": str(occupancy_image_manifest),
            "yaml_sha256": occupancy_yaml_hash,
            "image_sha256": occupancy_image_hash,
            "resolution": float(occupancy_yaml["resolution"]),
            "origin": occupancy_yaml["origin"],
            **pgm_metadata,
        },
    }
    if ray_observations_metadata is not None:
        result["ray_observations"] = ray_observations_metadata
    return result


def resolve_map_bundle_for_runtime(
    manifest_path: str | Path,
    allow_test_map: bool = False,
    acceptance_policy: str | None = None,
) -> dict[str, Any]:
    """Validate a bundle and return the runtime paths consumed by launch files."""
    policy = acceptance_policy or "approved_only"
    if allow_test_map:
        if policy not in {"approved_only", "allow_test"}:
            raise MapBundleError(
                "allow_test_map conflicts with acceptance_policy=" + repr(policy)
            )
        policy = "allow_test"
    if policy not in _RUNTIME_ACCEPTANCE_POLICIES:
        choices = ", ".join(sorted(_RUNTIME_ACCEPTANCE_POLICIES))
        raise MapBundleError(
            f"unsupported runtime acceptance policy '{policy}'; expected one of: {choices}"
        )

    result = validate_map_bundle(
        manifest_path,
        require_approved=policy == "approved_only",
        # Offline evidence has no runtime authority. Avoid even resolving its
        # path so a large or damaged sidecar cannot delay map startup.
        validate_offline_artifacts=False,
    )
    if result["deployment_status"] not in _RUNTIME_ACCEPTANCE_POLICIES[policy]:
        raise MapBundleError(
            "map bundle status "
            f"'{result['deployment_status']}' is not allowed by runtime policy '{policy}'"
        )
    return {
        "manifest": result["manifest"],
        "map_id": result["map_id"],
        "revision": result["revision"],
        "map_type": result["map_type"],
        "deployment_status": result["deployment_status"],
        "frame_id": result["frame_id"],
        "pcd_path": result["pcd"]["path"] if result["pcd"] else None,
        "occupancy_yaml_path": result["occupancy"]["yaml_path"],
        "occupancy_image_path": result["occupancy"]["image_path"],
        "shared_origin_confirmed": result["shared_origin_confirmed"],
        "occupancy_origin_reviewed": result["occupancy_origin_reviewed"],
        "acceptance_policy": policy,
    }
