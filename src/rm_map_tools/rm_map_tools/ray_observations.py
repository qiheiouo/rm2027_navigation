"""Streaming ray-observation I/O and a bounded records-only sidecar recorder."""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
import hashlib
from itertools import islice
import json
import math
import os
from pathlib import Path
import re
import stat
import threading
from typing import BinaryIO

from .immutable_output import new_output_path, publish_new_file


SCHEMA = "rm_map_ray_observations/v1"
SOURCE_STAMP_SEMANTICS = "source_integer_nanoseconds"

_POSITIVE_DECIMAL_INTEGER_PATTERN = re.compile(r"^[1-9][0-9]*$")
_MAX_SOURCE_STAMP_NS_DECIMAL_DIGITS = 20
_NON_SENSOR_SOURCE_FRAMES = {
    "map",
    "odom",
    "base_link",
    "base_footprint",
    "lio_imu_link",
}

DEFAULT_MIN_SAMPLE_PERIOD_NS = 200_000_000
DEFAULT_MAX_FRAMES = 10_000
DEFAULT_MAX_RAYS_PER_FRAME = 10_000
DEFAULT_MAX_TOTAL_RAYS = 10_000_000
DEFAULT_MAX_BYTES = 512 * 1024 * 1024
DEFAULT_HEADER_RESERVE_BYTES = 64 * 1024
DEFAULT_MAX_LINE_BYTES = 4 * 1024 * 1024


class RayObservationError(ValueError):
    """A ray sidecar or recorder contract was violated."""


class RaySidecarLimitError(RayObservationError):
    """A configured recorder or reader resource limit was reached."""


class RaySidecarHaltedError(RayObservationError):
    """The recorder cannot accept observations after a fatal condition."""


class RaySidecarIOError(RayObservationError):
    """The recorder could not durably write or snapshot its spool."""


@dataclass(frozen=True)
class RayObservation:
    stamp: float
    origin: tuple[float, float, float]
    endpoints: tuple[tuple[float, float, float], ...]
    source_stamp_ns: int | None = None


@dataclass(frozen=True)
class PreparedRayObservation:
    """One sensor observation transformed and reduced for recorder input."""

    stamp_ns: int
    origin: tuple[float, float, float]
    endpoints: tuple[tuple[float, float, float], ...]
    raw_point_count: int
    finite_point_count: int
    in_range_point_count: int


@dataclass(frozen=True)
class RayObservationReadLimits:
    max_bytes: int = DEFAULT_MAX_BYTES
    max_header_bytes: int = DEFAULT_HEADER_RESERVE_BYTES
    max_line_bytes: int = DEFAULT_MAX_LINE_BYTES
    max_frames: int = DEFAULT_MAX_FRAMES
    max_rays_per_frame: int = DEFAULT_MAX_RAYS_PER_FRAME
    max_total_rays: int = DEFAULT_MAX_TOTAL_RAYS

    def __post_init__(self) -> None:
        if min(
            self.max_bytes,
            self.max_header_bytes,
            self.max_line_bytes,
            self.max_frames,
            self.max_rays_per_frame,
            self.max_total_rays,
        ) <= 0:
            raise ValueError("ray observation read limits must be positive")
        if self.max_header_bytes > self.max_bytes:
            raise ValueError("max_header_bytes must not exceed max_bytes")
        if self.max_line_bytes > self.max_bytes:
            raise ValueError("max_line_bytes must not exceed max_bytes")


@dataclass(frozen=True)
class RaySidecarStats:
    status: str
    halt_reason: str | None
    source_frame: str | None
    received_frames: int
    accepted_frames: int
    skipped_min_period_frames: int
    total_rays: int
    record_bytes: int
    first_accepted_stamp_ns: int | None
    last_accepted_stamp_ns: int | None
    last_seen_stamp_ns: int | None


@dataclass(frozen=True)
class RaySidecarSnapshot:
    path: Path
    status: str
    sha256: str
    bytes: int
    observation_frames: int
    rays: int


def _read_bounded_line(
    stream: BinaryIO,
    limit: int,
    line_number: int,
    *,
    max_total_bytes: int | None = None,
) -> bytes:
    raw = stream.readline(limit + 1)
    if len(raw) > limit:
        raise RaySidecarLimitError(
            f"line {line_number}: JSONL line exceeds {limit} bytes"
        )
    if max_total_bytes is not None and stream.tell() > max_total_bytes:
        raise RaySidecarLimitError(
            "ray observation sidecar exceeds max_bytes="
            f"{max_total_bytes} while reading line {line_number}"
        )
    return raw


def _decode_json_object(raw: bytes, line_number: int) -> dict[str, object]:
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RayObservationError(f"line {line_number}: invalid UTF-8 JSON") from exc
    if not isinstance(value, dict):
        raise RayObservationError(f"line {line_number}: JSON value must be an object")
    return value


def _parse_observation(
    record: Mapping[str, object],
    line_number: int,
    *,
    require_source_stamp_ns: bool = False,
) -> RayObservation:
    if record.get("type") != "observation":
        raise RayObservationError(f"line {line_number}: expected observation record")
    raw_stamp = record.get("stamp")
    raw_origin = record.get("origin")
    raw_endpoints = record.get("endpoints")
    if (
        isinstance(raw_stamp, bool)
        or not isinstance(raw_stamp, (int, float))
        or not isinstance(raw_origin, list)
        or not isinstance(raw_endpoints, list)
    ):
        raise RayObservationError(
            f"line {line_number}: invalid timestamp or XYZ data"
        )
    vectors = [raw_origin, *raw_endpoints]
    if any(
        not isinstance(vector, list)
        or len(vector) != 3
        or any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in vector
        )
        for vector in vectors
    ):
        raise RayObservationError(
            f"line {line_number}: invalid timestamp or XYZ data"
        )
    stamp = float(raw_stamp)
    origin = tuple(float(value) for value in raw_origin)
    endpoints = tuple(
        tuple(float(value) for value in point) for point in raw_endpoints
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
        raise RayObservationError(
            f"line {line_number}: invalid timestamp or XYZ data"
        )
    raw_source_stamp_ns = record.get("source_stamp_ns")
    source_stamp_ns: int | None = None
    if raw_source_stamp_ns is not None:
        if not isinstance(
            raw_source_stamp_ns, str
        ) or not _POSITIVE_DECIMAL_INTEGER_PATTERN.fullmatch(
            raw_source_stamp_ns
        ) or len(raw_source_stamp_ns) > _MAX_SOURCE_STAMP_NS_DECIMAL_DIGITS:
            raise RayObservationError(
                f"line {line_number}: source_stamp_ns must be a positive "
                "decimal integer string of at most "
                f"{_MAX_SOURCE_STAMP_NS_DECIMAL_DIGITS} digits"
            )
        source_stamp_ns = int(raw_source_stamp_ns)
        if stamp != source_stamp_ns / 1_000_000_000.0:
            raise RayObservationError(
                f"line {line_number}: stamp does not match source_stamp_ns"
            )
    elif require_source_stamp_ns:
        raise RayObservationError(
            f"line {line_number}: source_stamp_ns is required by "
            f"stamp_semantics={SOURCE_STAMP_SEMANTICS!r}"
        )
    return RayObservation(stamp, origin, endpoints, source_stamp_ns)


def physical_source_frame(value: object, *, label: str = "source_frame") -> str:
    """Return a non-empty sensor frame and reject known world/body frames."""
    if not isinstance(value, str) or not value.strip():
        raise RayObservationError(f"{label} must be a non-empty string")
    source_frame = value.strip()
    comparable_frame = source_frame.lstrip("/")
    if not comparable_frame or comparable_frame in _NON_SENSOR_SOURCE_FRAMES:
        forbidden = ", ".join(sorted(_NON_SENSOR_SOURCE_FRAMES))
        raise RayObservationError(
            f"{label} must identify a physical lidar frame, not one of: "
            f"{forbidden}"
        )
    return source_frame


@contextmanager
def stream_ray_observations(
    path: str | Path,
    *,
    limits: RayObservationReadLimits | None = None,
) -> Iterator[tuple[dict[str, object], Iterator[RayObservation]]]:
    """Open a bounded JSONL sidecar and yield its header and one-pass iterator."""
    source = Path(path)
    selected_limits = limits or RayObservationReadLimits()
    try:
        source_size = source.stat().st_size
    except OSError as exc:
        raise RayObservationError(f"cannot stat ray observation sidecar: {source}") from exc
    if source_size > selected_limits.max_bytes:
        raise RaySidecarLimitError(
            "ray observation sidecar exceeds max_bytes="
            f"{selected_limits.max_bytes}: {source}"
        )

    with source.open("rb") as stream:
        line_number = 0
        header: dict[str, object] | None = None
        while True:
            line_number += 1
            raw = _read_bounded_line(
                stream,
                selected_limits.max_header_bytes,
                line_number,
                max_total_bytes=selected_limits.max_bytes,
            )
            if not raw:
                break
            if raw.strip():
                header = _decode_json_object(raw, line_number)
                break
        if header is None:
            raise RayObservationError("ray observation sidecar is empty")
        if header.get("schema") != SCHEMA or header.get("frame_id") != "map":
            raise RayObservationError(
                f"expected {SCHEMA!r} sidecar in canonical map frame"
            )
        require_source_stamp_ns = (
            header.get("stamp_semantics") == SOURCE_STAMP_SEMANTICS
        )

        def observations() -> Iterator[RayObservation]:
            nonlocal line_number
            previous_stamp: float | None = None
            previous_source_stamp_ns: int | None = None
            observation_count = 0
            ray_count = 0
            while True:
                line_number += 1
                raw = _read_bounded_line(
                    stream,
                    selected_limits.max_line_bytes,
                    line_number,
                    max_total_bytes=selected_limits.max_bytes,
                )
                if not raw:
                    break
                if not raw.strip():
                    continue
                record = _decode_json_object(raw, line_number)
                observation = _parse_observation(
                    record,
                    line_number,
                    require_source_stamp_ns=require_source_stamp_ns,
                )
                if len(observation.endpoints) > selected_limits.max_rays_per_frame:
                    raise RaySidecarLimitError(
                        f"line {line_number}: observation exceeds "
                        f"max_rays_per_frame={selected_limits.max_rays_per_frame}"
                    )
                observation_count += 1
                ray_count += len(observation.endpoints)
                if observation_count > selected_limits.max_frames:
                    raise RaySidecarLimitError(
                        "ray observation sidecar exceeds max_frames="
                        f"{selected_limits.max_frames}"
                    )
                if ray_count > selected_limits.max_total_rays:
                    raise RaySidecarLimitError(
                        "ray observation sidecar exceeds max_total_rays="
                        f"{selected_limits.max_total_rays}"
                    )
                if previous_stamp is not None and observation.stamp <= previous_stamp:
                    raise RayObservationError(
                        f"line {line_number}: timestamps must be strictly increasing"
                    )
                if observation.source_stamp_ns is not None:
                    if (
                        previous_source_stamp_ns is not None
                        and observation.source_stamp_ns
                        <= previous_source_stamp_ns
                    ):
                        raise RayObservationError(
                            f"line {line_number}: source_stamp_ns values must be "
                            "strictly increasing"
                        )
                    previous_source_stamp_ns = observation.source_stamp_ns
                previous_stamp = observation.stamp
                yield observation

            if observation_count == 0:
                raise RayObservationError(
                    "ray observation sidecar contains no observations"
                )
            statistics = header.get("statistics")
            if isinstance(statistics, dict):
                expected_frames = statistics.get("accepted_frames")
                expected_rays = statistics.get("total_rays")
                if (
                    isinstance(expected_frames, int)
                    and expected_frames != observation_count
                ):
                    raise RayObservationError(
                        "ray observation header accepted_frames does not match payload"
                    )
                if isinstance(expected_rays, int) and expected_rays != ray_count:
                    raise RayObservationError(
                        "ray observation header total_rays does not match payload"
                    )

        yield header, observations()


def load_ray_observations(
    path: str | Path,
    *,
    limits: RayObservationReadLimits | None = None,
) -> tuple[dict[str, object], list[RayObservation]]:
    """Compatibility helper for callers that intentionally need a small list."""
    with stream_ray_observations(path, limits=limits) as (header, observations):
        return header, list(observations)


def _finite_xyz(value: Iterable[float], label: str) -> tuple[float, float, float]:
    try:
        xyz = tuple(float(component) for component in islice(value, 4))
    except (TypeError, ValueError) as exc:
        raise RayObservationError(f"{label} must contain three finite values") from exc
    if len(xyz) != 3 or not all(math.isfinite(component) for component in xyz):
        raise RayObservationError(f"{label} must contain three finite values")
    return xyz


def _validated_rotation_matrix(
    *,
    quaternion_xyzw: Iterable[float] | None,
    rotation_matrix: Iterable[Iterable[float]] | None,
) -> tuple[tuple[float, float, float], ...]:
    if (quaternion_xyzw is None) == (rotation_matrix is None):
        raise ValueError(
            "provide exactly one of quaternion_xyzw or rotation_matrix"
        )

    if quaternion_xyzw is not None:
        try:
            quaternion = tuple(
                float(value) for value in islice(quaternion_xyzw, 5)
            )
        except (TypeError, ValueError) as exc:
            raise RayObservationError(
                "quaternion_xyzw must contain four finite values"
            ) from exc
        if len(quaternion) != 4 or not all(
            math.isfinite(value) for value in quaternion
        ):
            raise RayObservationError(
                "quaternion_xyzw must contain four finite values"
            )
        norm = math.hypot(*quaternion)
        if not math.isfinite(norm) or norm <= 1.0e-12:
            raise RayObservationError(
                "quaternion_xyzw must have non-zero norm"
            )
        x, y, z, w = (value / norm for value in quaternion)
        return (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        )

    try:
        matrix = tuple(
            tuple(float(value) for value in islice(row, 4))
            for row in islice(rotation_matrix, 4)
        )
    except (TypeError, ValueError) as exc:
        raise RayObservationError(
            "rotation_matrix must be a finite 3x3 matrix"
        ) from exc
    if any(
        (
            len(matrix) != 3,
            any(len(row) != 3 for row in matrix),
            not all(math.isfinite(value) for row in matrix for value in row),
        )
    ):
        raise RayObservationError(
            "rotation_matrix must be a finite 3x3 matrix"
        )

    tolerance = 1.0e-6
    for left in range(3):
        for right in range(3):
            dot = sum(
                matrix[left][axis] * matrix[right][axis] for axis in range(3)
            )
            expected = 1.0 if left == right else 0.0
            if not math.isclose(dot, expected, rel_tol=0.0, abs_tol=tolerance):
                raise RayObservationError(
                    "rotation_matrix must be orthonormal"
                )
    first_term = matrix[0][0] * (
        matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]
    )
    second_term = matrix[0][1] * (
        matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]
    )
    third_term = matrix[0][2] * (
        matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]
    )
    determinant = first_term - second_term + third_term
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=tolerance):
        raise RayObservationError(
            "rotation_matrix must be a proper rotation with determinant +1"
        )
    return matrix


def prepare_map_ray_observation(
    points_sensor: Iterable[Iterable[float]],
    *,
    stamp_ns: int,
    translation: Iterable[float],
    quaternion_xyzw: Iterable[float] | None = None,
    rotation_matrix: Iterable[Iterable[float]] | None = None,
    min_range: float,
    max_range: float,
    voxel_size: float,
    max_rays_per_frame: int,
) -> PreparedRayObservation:
    """Transform raw sensor returns into deterministic map-frame ray endpoints.

    Range filtering is performed in the sensor frame before transformation. A
    return beyond ``max_range`` is discarded, never shortened into a false hit.
    Map-frame voxel deduplication keeps the return closest to the sensor origin;
    ties and output ordering are deterministic and independent of input order.
    """
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
        raise ValueError("stamp_ns must be a positive integer")
    if stamp_ns <= 0:
        raise ValueError("stamp_ns must be a positive integer")
    numeric_options = {
        "min_range": min_range,
        "max_range": max_range,
        "voxel_size": voxel_size,
    }
    if any(isinstance(value, bool) for value in numeric_options.values()):
        raise ValueError(
            "ray geometry ranges and voxel_size must be finite numbers"
        )
    try:
        clean_min_range = float(min_range)
        clean_max_range = float(max_range)
        clean_voxel_size = float(voxel_size)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "ray geometry ranges and voxel_size must be finite numbers"
        ) from exc
    if not all(
        math.isfinite(value)
        for value in (clean_min_range, clean_max_range, clean_voxel_size)
    ):
        raise ValueError(
            "ray geometry ranges and voxel_size must be finite numbers"
        )
    if clean_min_range < 0.0 or clean_max_range <= clean_min_range:
        raise ValueError("require 0 <= min_range < max_range")
    if clean_voxel_size <= 0.0:
        raise ValueError("voxel_size must be positive")
    if isinstance(max_rays_per_frame, bool) or not isinstance(
        max_rays_per_frame, int
    ):
        raise ValueError("max_rays_per_frame must be a positive integer")
    if max_rays_per_frame <= 0:
        raise ValueError("max_rays_per_frame must be a positive integer")

    origin = _finite_xyz(translation, "translation")
    rotation = _validated_rotation_matrix(
        quaternion_xyzw=quaternion_xyzw,
        rotation_matrix=rotation_matrix,
    )

    raw_point_count = 0
    finite_point_count = 0
    in_range_point_count = 0
    selected: dict[
        tuple[int, int, int],
        tuple[float, tuple[float, float, float]],
    ] = {}
    for point_number, raw_point in enumerate(points_sensor, start=1):
        raw_point_count += 1
        try:
            sensor_point = tuple(
                float(value) for value in islice(raw_point, 4)
            )
        except (TypeError, ValueError) as exc:
            raise RayObservationError(
                f"sensor point {point_number} must contain three "
                "numeric values"
            ) from exc
        if len(sensor_point) != 3:
            raise RayObservationError(
                f"sensor point {point_number} must contain exactly "
                "three values"
            )
        if not all(math.isfinite(value) for value in sensor_point):
            continue
        finite_point_count += 1
        sensor_range = math.hypot(*sensor_point)
        if sensor_range < clean_min_range or sensor_range > clean_max_range:
            continue
        in_range_point_count += 1

        endpoint = tuple(
            sum(
                (
                    rotation[row][axis] * sensor_point[axis]
                    for axis in range(3)
                ),
                start=origin[row],
            )
            for row in range(3)
        )
        if not all(math.isfinite(value) for value in endpoint):
            raise RayObservationError(
                f"sensor point {point_number} overflowed the rigid transform"
            )
        endpoint = tuple(0.0 if value == 0.0 else value for value in endpoint)
        voxel = tuple(
            math.floor(value / clean_voxel_size) for value in endpoint
        )
        candidate = (sensor_range, endpoint)
        current = selected.get(voxel)
        if current is None:
            selected[voxel] = candidate
            if len(selected) > max_rays_per_frame:
                raise RaySidecarLimitError(
                    "deduplicated observation exceeds max_rays_per_frame="
                    f"{max_rays_per_frame}"
                )
        elif candidate < current:
            selected[voxel] = candidate

    endpoints = tuple(selected[voxel][1] for voxel in sorted(selected))
    return PreparedRayObservation(
        stamp_ns=stamp_ns,
        origin=origin,
        endpoints=endpoints,
        raw_point_count=raw_point_count,
        finite_point_count=finite_point_count,
        in_range_point_count=in_range_point_count,
    )


class RaySidecarRecorder:
    """Bounded records-only spool whose snapshots are immutable v1 sidecars."""

    def __init__(
        self,
        spool_path: str | Path,
        *,
        source_frame: str | None = None,
        min_sample_period_ns: int = DEFAULT_MIN_SAMPLE_PERIOD_NS,
        max_frames: int = DEFAULT_MAX_FRAMES,
        max_rays_per_frame: int = DEFAULT_MAX_RAYS_PER_FRAME,
        max_total_rays: int = DEFAULT_MAX_TOTAL_RAYS,
        max_bytes: int = DEFAULT_MAX_BYTES,
        header_reserve_bytes: int = DEFAULT_HEADER_RESERVE_BYTES,
        metadata: Mapping[str, object] | None = None,
    ) -> None:
        integer_values = {
            "min_sample_period_ns": min_sample_period_ns,
            "max_frames": max_frames,
            "max_rays_per_frame": max_rays_per_frame,
            "max_total_rays": max_total_rays,
            "max_bytes": max_bytes,
            "header_reserve_bytes": header_reserve_bytes,
        }
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in integer_values.values()
        ):
            raise ValueError("ray sidecar limits and periods must be integers")
        if min_sample_period_ns < 0:
            raise ValueError("min_sample_period_ns must be non-negative")
        if min(
            max_frames,
            max_rays_per_frame,
            max_total_rays,
            max_bytes,
            header_reserve_bytes,
        ) <= 0:
            raise ValueError("ray sidecar limits must be positive")
        if header_reserve_bytes >= max_bytes:
            raise ValueError("header_reserve_bytes must be smaller than max_bytes")
        if max_rays_per_frame > max_total_rays:
            raise ValueError("max_rays_per_frame must not exceed max_total_rays")

        clean_source_frame = self._clean_source_frame(source_frame)
        metadata_value = dict(metadata or {})
        try:
            metadata_json = json.dumps(
                metadata_value,
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("ray sidecar metadata must be finite JSON data") from exc
        self._metadata = json.loads(metadata_json)

        requested_spool = Path(spool_path).expanduser()
        if requested_spool.is_symlink():
            raise ValueError(
                f"ray sidecar spool is a symlink; refusing: {requested_spool}"
            )
        self._spool_path = requested_spool.resolve()
        self._spool_path.parent.mkdir(parents=True, exist_ok=True)
        flags = os.O_RDWR | os.O_CREAT | os.O_EXCL
        flags |= getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        try:
            descriptor = os.open(self._spool_path, flags, 0o600)
        except OSError as exc:
            raise RaySidecarIOError(
                f"cannot create exclusive ray sidecar spool: {self._spool_path}"
            ) from exc
        try:
            descriptor_stat = os.fstat(descriptor)
            if not stat.S_ISREG(descriptor_stat.st_mode):
                raise OSError("ray sidecar spool descriptor is not a regular file")
            self._stream = os.fdopen(descriptor, "r+b", buffering=0)
        except Exception:
            os.close(descriptor)
            raise

        self._state_lock = threading.RLock()
        self._spool_identity = (descriptor_stat.st_dev, descriptor_stat.st_ino)
        self._configured_source_frame = clean_source_frame
        self._source_frame = clean_source_frame
        self._min_sample_period_ns = min_sample_period_ns
        self._max_frames = max_frames
        self._max_rays_per_frame = max_rays_per_frame
        self._max_total_rays = max_total_rays
        self._max_bytes = max_bytes
        self._header_reserve_bytes = header_reserve_bytes
        self._closed = False
        self._halt_reason: str | None = None
        self._received_frames = 0
        self._accepted_frames = 0
        self._skipped_min_period_frames = 0
        self._total_rays = 0
        self._record_bytes = 0
        self._first_accepted_stamp_ns: int | None = None
        self._last_accepted_stamp_ns: int | None = None
        self._last_seen_stamp_ns: int | None = None
        self._last_serialized_stamp: float | None = None

    @staticmethod
    def _clean_source_frame(value: str | None) -> str | None:
        if value is None:
            return None
        if not isinstance(value, str) or not value.strip():
            raise ValueError("source_frame must be a non-empty string when set")
        cleaned = value.strip().lstrip("/")
        if not cleaned:
            raise ValueError("source_frame must be a non-empty string when set")
        return cleaned

    @property
    def spool_path(self) -> Path:
        return self._spool_path

    @property
    def spool_identity(self) -> tuple[int, int]:
        """Return the creation-time ``(device, inode)`` of the held spool."""
        return self._spool_identity

    @property
    def status(self) -> str:
        with self._state_lock:
            if self._halt_reason is not None:
                return "halted"
            if self._closed:
                return "closed"
            return "recording"

    @property
    def stats(self) -> RaySidecarStats:
        with self._state_lock:
            return RaySidecarStats(
                status=self.status,
                halt_reason=self._halt_reason,
                source_frame=self._source_frame,
                received_frames=self._received_frames,
                accepted_frames=self._accepted_frames,
                skipped_min_period_frames=self._skipped_min_period_frames,
                total_rays=self._total_rays,
                record_bytes=self._record_bytes,
                first_accepted_stamp_ns=self._first_accepted_stamp_ns,
                last_accepted_stamp_ns=self._last_accepted_stamp_ns,
                last_seen_stamp_ns=self._last_seen_stamp_ns,
            )

    def _halt(self, reason: str) -> None:
        if self._halt_reason is None:
            self._halt_reason = reason

    def halt(self, reason: str) -> None:
        """Halt recording for an external fatal contract violation.

        The first halt reason is retained so repeated error handling is
        idempotent and the eventual incomplete snapshot preserves root cause.
        """
        with self._state_lock:
            self._halt_requested(reason)

    def _halt_requested(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError(
                "ray sidecar halt reason must be a non-empty string"
            )
        cleaned_reason = reason.strip()
        if len(cleaned_reason) > 128 or any(
            ord(character) < 0x20 for character in cleaned_reason
        ):
            raise ValueError(
                "ray sidecar halt reason must be at most 128 printable characters"
            )
        if self._closed:
            raise RaySidecarHaltedError(
                "cannot halt a closed ray sidecar recorder"
            )
        self._halt(cleaned_reason)

    def _ensure_open_for_recording(self) -> None:
        if self._closed:
            raise RaySidecarHaltedError("ray sidecar recorder is closed")
        if self._halt_reason is not None:
            raise RaySidecarHaltedError(
                f"ray sidecar recorder is halted: {self._halt_reason}"
            )

    def _fatal(
        self,
        reason: str,
        message: str,
        error_type: type[RayObservationError] = RayObservationError,
    ) -> None:
        self._halt(reason)
        raise error_type(message)

    def record_observation(
        self,
        *,
        stamp_ns: int,
        source_frame: str,
        origin: Iterable[float],
        endpoints: Iterable[Iterable[float]],
    ) -> bool:
        """Record one whole observation, or return False for period sampling."""
        with self._state_lock:
            return self._record_observation_locked(
                stamp_ns=stamp_ns,
                source_frame=source_frame,
                origin=origin,
                endpoints=endpoints,
            )

    def _record_observation_locked(
        self,
        *,
        stamp_ns: int,
        source_frame: str,
        origin: Iterable[float],
        endpoints: Iterable[Iterable[float]],
    ) -> bool:
        self._ensure_open_for_recording()
        self._received_frames += 1
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
            self._fatal(
                "invalid_source_time",
                "source stamp_ns must be a positive integer",
            )
        try:
            clean_frame = self._clean_source_frame(source_frame)
        except ValueError as exc:
            self._fatal("invalid_source_frame", str(exc))
            raise AssertionError("unreachable") from exc
        assert clean_frame is not None
        if self._source_frame is None:
            self._source_frame = clean_frame
        elif clean_frame != self._source_frame:
            self._fatal(
                "source_frame_changed",
                f"source frame changed from {self._source_frame!r} to {clean_frame!r}",
            )
        if (
            self._last_seen_stamp_ns is not None
            and stamp_ns <= self._last_seen_stamp_ns
        ):
            self._fatal(
                "non_increasing_source_time",
                "source timestamps must be strictly increasing integer nanoseconds",
            )
        self._last_seen_stamp_ns = stamp_ns
        if (
            self._last_accepted_stamp_ns is not None
            and stamp_ns - self._last_accepted_stamp_ns
            < self._min_sample_period_ns
        ):
            self._skipped_min_period_frames += 1
            return False
        if self._accepted_frames >= self._max_frames:
            self._fatal(
                "max_frames",
                f"ray sidecar reached max_frames={self._max_frames}",
                RaySidecarLimitError,
            )

        try:
            clean_origin = _finite_xyz(origin, "origin")
            clean_endpoints: list[tuple[float, float, float]] = []
            for point in endpoints:
                if len(clean_endpoints) >= self._max_rays_per_frame:
                    self._fatal(
                        "max_rays_per_frame",
                        "observation exceeds max_rays_per_frame="
                        f"{self._max_rays_per_frame}",
                        RaySidecarLimitError,
                    )
                clean_endpoints.append(_finite_xyz(point, "endpoint"))
        except RayObservationError:
            if self._halt_reason is None:
                self._halt("invalid_xyz")
            raise
        if not clean_endpoints:
            self._fatal("empty_observation", "observation endpoints must not be empty")
        next_total_rays = self._total_rays + len(clean_endpoints)
        if next_total_rays > self._max_total_rays:
            self._fatal(
                "max_total_rays",
                f"ray sidecar would exceed max_total_rays={self._max_total_rays}",
                RaySidecarLimitError,
            )

        stamp = stamp_ns / 1_000_000_000.0
        if self._last_serialized_stamp is not None and stamp <= self._last_serialized_stamp:
            self._fatal(
                "float_stamp_precision",
                "source nanoseconds cannot be represented as strictly increasing "
                "v1 floating-point stamps",
            )
        record = {
            "type": "observation",
            "stamp": stamp,
            "source_stamp_ns": str(stamp_ns),
            "origin": list(clean_origin),
            "endpoints": [list(point) for point in clean_endpoints],
        }
        try:
            encoded = (
                json.dumps(
                    record,
                    allow_nan=False,
                    separators=(",", ":"),
                    sort_keys=True,
                )
                + "\n"
            ).encode("utf-8")
        except (TypeError, ValueError) as exc:
            self._fatal("serialization_error", f"cannot serialize observation: {exc}")
            raise AssertionError("unreachable") from exc
        if (
            self._header_reserve_bytes + self._record_bytes + len(encoded)
            > self._max_bytes
        ):
            self._fatal(
                "max_bytes",
                f"ray sidecar would exceed max_bytes={self._max_bytes}",
                RaySidecarLimitError,
            )

        start_offset = self._record_bytes
        try:
            written = self._stream.write(encoded)
            if written != len(encoded):
                raise OSError(
                    f"short spool write: expected {len(encoded)}, wrote {written}"
                )
        except OSError as exc:
            try:
                self._stream.truncate(start_offset)
                self._stream.seek(start_offset)
            except OSError:
                pass
            self._halt("spool_io_error")
            raise RaySidecarIOError("cannot write complete observation to spool") from exc

        self._record_bytes += len(encoded)
        self._accepted_frames += 1
        self._total_rays = next_total_rays
        if self._first_accepted_stamp_ns is None:
            self._first_accepted_stamp_ns = stamp_ns
        self._last_accepted_stamp_ns = stamp_ns
        self._last_serialized_stamp = stamp
        return True

    def _snapshot_header(self) -> dict[str, object]:
        if self._accepted_frames <= 0 or self._source_frame is None:
            raise RayObservationError(
                "cannot snapshot a ray sidecar with no accepted observations"
            )
        status = "incomplete" if self._halt_reason is not None else "complete"
        header: dict[str, object] = {
            "schema": SCHEMA,
            "frame_id": "map",
            "status": status,
            "producer": "rm_map_tools/ray_observations.RaySidecarRecorder",
            "source_frame": self._source_frame,
            "stamp_semantics": SOURCE_STAMP_SEMANTICS,
            "min_sample_period_ns": self._min_sample_period_ns,
            "limits": {
                "max_frames": self._max_frames,
                "max_rays_per_frame": self._max_rays_per_frame,
                "max_total_rays": self._max_total_rays,
                "max_bytes": self._max_bytes,
                "header_reserve_bytes": self._header_reserve_bytes,
            },
            "statistics": {
                "received_frames": self._received_frames,
                "accepted_frames": self._accepted_frames,
                "skipped_min_period_frames": self._skipped_min_period_frames,
                "total_rays": self._total_rays,
                "record_bytes": self._record_bytes,
                "first_accepted_stamp_ns": str(self._first_accepted_stamp_ns),
                "last_accepted_stamp_ns": str(self._last_accepted_stamp_ns),
                "last_seen_stamp_ns": str(self._last_seen_stamp_ns),
            },
            "metadata": self._metadata,
        }
        if self._halt_reason is not None:
            header["halt_reason"] = self._halt_reason
        return header

    def snapshot(self, destination: str | Path) -> RaySidecarSnapshot:
        """Publish a complete immutable v1 header plus the committed spool prefix."""
        with self._state_lock:
            return self._snapshot_locked(destination)

    def _snapshot_locked(self, destination: str | Path) -> RaySidecarSnapshot:
        if self._closed:
            raise RaySidecarHaltedError(
                "cannot snapshot a closed ray sidecar recorder"
            )
        header = self._snapshot_header()
        try:
            header_bytes = (
                json.dumps(
                    header,
                    allow_nan=False,
                    separators=(",", ":"),
                    sort_keys=True,
                )
                + "\n"
            ).encode("utf-8")
        except (TypeError, ValueError) as exc:
            self._halt("header_serialization_error")
            raise RayObservationError("cannot serialize ray sidecar header") from exc
        if len(header_bytes) > self._header_reserve_bytes:
            self._halt("header_reserve_exceeded")
            raise RaySidecarLimitError(
                "ray sidecar header exceeds header_reserve_bytes="
                f"{self._header_reserve_bytes}"
            )
        frozen_record_bytes = self._record_bytes
        frozen_observation_frames = self._accepted_frames
        frozen_rays = self._total_rays
        final_size = len(header_bytes) + frozen_record_bytes
        if final_size > self._max_bytes:
            self._halt("max_bytes")
            raise RaySidecarLimitError(
                f"ray sidecar snapshot exceeds max_bytes={self._max_bytes}"
            )

        try:
            output_path = new_output_path(destination, "ray observation sidecar")
        except ValueError:
            raise
        except OSError as exc:
            self._halt("snapshot_io_error")
            raise RaySidecarIOError(
                "cannot prepare ray sidecar snapshot output"
            ) from exc
        spool_descriptor: int | None = None
        try:
            self._stream.flush()
            stream_descriptor = self._stream.fileno()
            os.fsync(stream_descriptor)
            descriptor_stat = os.fstat(stream_descriptor)
            descriptor_identity = (
                descriptor_stat.st_dev,
                descriptor_stat.st_ino,
            )
            if (
                not stat.S_ISREG(descriptor_stat.st_mode)
                or descriptor_identity != self._spool_identity
                or descriptor_stat.st_size < frozen_record_bytes
            ):
                raise OSError(
                    "held ray sidecar spool descriptor no longer matches "
                    "its committed regular-file prefix"
                )
            # The duplicate pins the creation-time inode for the complete
            # publication callback. pread() leaves the writer's shared file
            # offset untouched and reads only the prefix frozen above.
            spool_descriptor = os.dup(stream_descriptor)

            def write_snapshot(path: Path) -> tuple[str, int]:
                digest = hashlib.sha256()
                written_bytes = 0
                with path.open("wb") as output:
                    header_written = output.write(header_bytes)
                    if header_written != len(header_bytes):
                        raise OSError(
                            "short snapshot header write: expected "
                            f"{len(header_bytes)}, wrote {header_written}"
                        )
                    digest.update(header_bytes)
                    written_bytes += header_written
                    remaining = frozen_record_bytes
                    offset = 0
                    while remaining:
                        assert spool_descriptor is not None
                        block = os.pread(
                            spool_descriptor,
                            min(1024 * 1024, remaining),
                            offset,
                        )
                        if not block:
                            raise OSError(
                                "ray sidecar spool ended before committed prefix"
                            )
                        block_written = output.write(block)
                        if block_written != len(block):
                            raise OSError(
                                "short snapshot record write: expected "
                                f"{len(block)}, wrote {block_written}"
                            )
                        digest.update(block)
                        written_bytes += block_written
                        remaining -= block_written
                        offset += len(block)
                if written_bytes != final_size:
                    raise OSError(
                        f"ray sidecar snapshot size {written_bytes} != "
                        f"expected {final_size}"
                    )
                return digest.hexdigest(), written_bytes

            snapshot_sha256, actual_size = publish_new_file(
                output_path,
                "ray observation sidecar",
                write_snapshot,
            )
        except ValueError:
            raise
        except OSError as exc:
            self._halt("snapshot_io_error")
            raise RaySidecarIOError("cannot publish ray sidecar snapshot") from exc
        finally:
            if spool_descriptor is not None:
                os.close(spool_descriptor)

        return RaySidecarSnapshot(
            path=output_path,
            status=str(header["status"]),
            sha256=snapshot_sha256,
            bytes=actual_size,
            observation_frames=frozen_observation_frames,
            rays=frozen_rays,
        )

    def reset(self) -> None:
        """Discard the current records and start a fresh bounded session."""
        with self._state_lock:
            self._reset_locked()

    def _reset_locked(self) -> None:
        if self._closed:
            raise RaySidecarHaltedError("cannot reset a closed ray sidecar recorder")
        try:
            self._stream.truncate(0)
            self._stream.seek(0)
        except OSError as exc:
            self._halt("reset_io_error")
            raise RaySidecarIOError("cannot reset ray sidecar spool") from exc
        self._source_frame = self._configured_source_frame
        self._halt_reason = None
        self._received_frames = 0
        self._accepted_frames = 0
        self._skipped_min_period_frames = 0
        self._total_rays = 0
        self._record_bytes = 0
        self._first_accepted_stamp_ns = None
        self._last_accepted_stamp_ns = None
        self._last_seen_stamp_ns = None
        self._last_serialized_stamp = None

    def close(self) -> None:
        """Flush and close the spool without publishing a final sidecar."""
        with self._state_lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self._closed:
            return
        try:
            self._stream.flush()
            os.fsync(self._stream.fileno())
        except OSError as exc:
            self._halt("close_io_error")
            raise RaySidecarIOError("cannot durably close ray sidecar spool") from exc
        finally:
            self._stream.close()
            self._closed = True

    def __enter__(self) -> "RaySidecarRecorder":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        del exc_type, exc_value, traceback
        self.close()
