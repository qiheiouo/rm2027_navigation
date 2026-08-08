"""Pure algorithms for localization integrity monitoring.

This module intentionally has no ROS imports so its geometry, distance-field,
and state-classification behavior can be tested on Windows.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import heapq
import math
from typing import Iterable, Sequence


class IntegrityState(str, Enum):
    GOOD = "GOOD"
    SUSPECT = "SUSPECT"
    REJECT = "REJECT"
    UNKNOWN = "UNKNOWN"


class Reason(str, Enum):
    MAP_UNAVAILABLE = "MAP_UNAVAILABLE"
    GLOBAL_POSE_UNAVAILABLE = "GLOBAL_POSE_UNAVAILABLE"
    GLOBAL_POSE_STALE = "GLOBAL_POSE_STALE"
    GLOBAL_POSE_TIME_RESET = "GLOBAL_POSE_TIME_RESET"
    SCAN_UNAVAILABLE = "SCAN_UNAVAILABLE"
    SCAN_STALE = "SCAN_STALE"
    SCAN_TIME_RESET = "SCAN_TIME_RESET"
    POSE_SCAN_TIME_MISMATCH = "POSE_SCAN_TIME_MISMATCH"
    ODOM_UNAVAILABLE = "ODOM_UNAVAILABLE"
    ODOM_TIME_MISMATCH = "ODOM_TIME_MISMATCH"
    ODOM_TIME_RESET = "ODOM_TIME_RESET"
    TF_LOOKUP_FAILED = "TF_LOOKUP_FAILED"
    TF_TIME_MISMATCH = "TF_TIME_MISMATCH"
    INSUFFICIENT_SCAN_POINTS = "INSUFFICIENT_SCAN_POINTS"
    SCAN_MAP_EVIDENCE_UNAVAILABLE = "SCAN_MAP_EVIDENCE_UNAVAILABLE"
    GLOBAL_TRANSLATION_JUMP = "GLOBAL_TRANSLATION_JUMP"
    GLOBAL_YAW_JUMP = "GLOBAL_YAW_JUMP"
    CORRECTION_TRANSLATION_JUMP = "CORRECTION_TRANSLATION_JUMP"
    CORRECTION_YAW_JUMP = "CORRECTION_YAW_JUMP"
    CORRECTION_TRANSLATION_RATE = "CORRECTION_TRANSLATION_RATE"
    CORRECTION_YAW_RATE = "CORRECTION_YAW_RATE"
    SCAN_MAP_AGREEMENT_LOW = "SCAN_MAP_AGREEMENT_LOW"
    SCAN_MAP_RESIDUAL_HIGH = "SCAN_MAP_RESIDUAL_HIGH"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Transform3D:
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class ScanMapMetrics:
    valid_point_count: int
    sampled_point_count: int
    points_in_map: int
    supported_point_count: int
    agreement: float | None
    mean_residual_m: float | None
    p95_residual_m: float | None


@dataclass
class IntegrityMetrics:
    stamp_sec: float = 0.0
    profile: str = "generic"
    map_ready: bool = False
    global_pose_ready: bool = False
    scan_ready: bool = False
    odom_ready: bool = False
    tf_ready: bool = False
    scan_map_ready: bool = False
    global_pose_time_reset: bool = False
    scan_time_reset: bool = False
    odom_time_reset: bool = False
    global_pose_age_sec: float | None = None
    scan_age_sec: float | None = None
    pose_scan_dt_sec: float | None = None
    pose_odom_dt_sec: float | None = None
    tf_scan_dt_sec: float | None = None
    valid_scan_points: int = 0
    scan_points_in_map: int = 0
    scan_map_agreement: float | None = None
    scan_map_mean_residual_m: float | None = None
    scan_map_p95_residual_m: float | None = None
    global_translation_jump_m: float | None = None
    global_yaw_jump_rad: float | None = None
    correction_translation_jump_m: float | None = None
    correction_yaw_jump_rad: float | None = None
    correction_translation_m: float | None = None
    correction_yaw_rad: float | None = None
    correction_translation_rate_mps: float | None = None
    correction_yaw_rate_rps: float | None = None
    scan_rate_hz: float | None = None
    scan_nan_ratio: float | None = None
    scan_infinite_ratio: float | None = None
    scan_out_of_range_ratio: float | None = None
    odom_translation_step_m: float | None = None
    odom_yaw_step_rad: float | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class IntegrityThresholds:
    max_global_pose_age_sec: float = 0.5
    max_scan_age_sec: float = 0.5
    max_pose_scan_dt_sec: float = 0.15
    max_pose_odom_dt_sec: float = 0.05
    max_tf_scan_dt_sec: float = 0.05
    min_valid_scan_points: int = 30
    min_scan_map_agreement_warn: float = 0.45
    min_scan_map_agreement_reject: float = 0.20
    max_scan_map_mean_residual_warn_m: float = 0.30
    max_global_translation_jump_warn_m: float = 0.30
    max_global_translation_jump_reject_m: float = 0.75
    max_global_yaw_jump_warn_rad: float = 0.60
    max_global_yaw_jump_reject_rad: float = 1.20
    max_correction_translation_jump_warn_m: float = 0.30
    max_correction_translation_jump_reject_m: float = 0.75
    max_correction_yaw_jump_warn_rad: float = 0.60
    max_correction_yaw_jump_reject_rad: float = 1.20
    max_correction_translation_rate_warn_mps: float = 1.50
    max_correction_yaw_rate_warn_rps: float = 3.00


@dataclass(frozen=True)
class EvaluationResult:
    state: IntegrityState
    reasons: tuple[str, ...] = field(default_factory=tuple)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_delta(current: Pose2D, previous: Pose2D) -> tuple[float, float]:
    translation = math.hypot(current.x - previous.x, current.y - previous.y)
    yaw = abs(normalize_angle(current.yaw - previous.yaw))
    return translation, yaw


def compose_pose(left: Pose2D, right: Pose2D) -> Pose2D:
    cosine = math.cos(left.yaw)
    sine = math.sin(left.yaw)
    return Pose2D(
        x=left.x + cosine * right.x - sine * right.y,
        y=left.y + sine * right.x + cosine * right.y,
        yaw=normalize_angle(left.yaw + right.yaw),
    )


def inverse_pose(pose: Pose2D) -> Pose2D:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Pose2D(
        x=-cosine * pose.x - sine * pose.y,
        y=sine * pose.x - cosine * pose.y,
        yaw=normalize_angle(-pose.yaw),
    )


def correction_from_global_and_odom(map_to_base: Pose2D, odom_to_base: Pose2D) -> Pose2D:
    return compose_pose(map_to_base, inverse_pose(odom_to_base))


def quaternion_to_yaw(rotation_xyzw: Sequence[float]) -> float:
    x, y, z, w = _normalized_quaternion(rotation_xyzw)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _normalized_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError("quaternion must contain four values")
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("quaternion must be finite and non-zero")
    return x / norm, y / norm, z / norm, w / norm


def rotate_vector(
    rotation_xyzw: Sequence[float], point: Sequence[float]
) -> tuple[float, float, float]:
    x, y, z, w = _normalized_quaternion(rotation_xyzw)
    px, py, pz = (float(value) for value in point)
    # Quaternion-vector multiplication expressed without temporary quaternions.
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    )


def transform_point(transform: Transform3D, point: Sequence[float]) -> tuple[float, float, float]:
    rx, ry, rz = rotate_vector(transform.rotation_xyzw, point)
    tx, ty, tz = transform.translation
    return rx + tx, ry + ty, rz + tz


class OccupancyDistanceField:
    """Eight-connected distance field for a static OccupancyGrid."""

    _NEIGHBORS = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin: Pose2D,
        data: Sequence[int],
        occupied_threshold: int = 65,
    ) -> None:
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise ValueError("map dimensions and resolution must be positive")
        if len(data) != width * height:
            raise ValueError("map data length does not match dimensions")
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = origin
        self.occupied_threshold = int(occupied_threshold)
        self._distance_cells = [math.inf] * (self.width * self.height)
        queue: list[tuple[float, int]] = []
        for index, value in enumerate(data):
            if int(value) >= self.occupied_threshold:
                self._distance_cells[index] = 0.0
                heapq.heappush(queue, (0.0, index))
        if not queue:
            raise ValueError("map contains no occupied cells")
        self._build(queue)

    def _build(self, queue: list[tuple[float, int]]) -> None:
        while queue:
            distance, index = heapq.heappop(queue)
            if distance != self._distance_cells[index]:
                continue
            x = index % self.width
            y = index // self.width
            for dx, dy, cost in self._NEIGHBORS:
                nx = x + dx
                ny = y + dy
                if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                    continue
                neighbor = ny * self.width + nx
                candidate = distance + cost
                if candidate < self._distance_cells[neighbor]:
                    self._distance_cells[neighbor] = candidate
                    heapq.heappush(queue, (candidate, neighbor))

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        dx = x - self.origin.x
        dy = y - self.origin.y
        cosine = math.cos(self.origin.yaw)
        sine = math.sin(self.origin.yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        grid_x = math.floor(local_x / self.resolution)
        grid_y = math.floor(local_y / self.resolution)
        if grid_x < 0 or grid_x >= self.width or grid_y < 0 or grid_y >= self.height:
            return None
        return int(grid_x), int(grid_y)

    def distance_at_world(self, x: float, y: float) -> float | None:
        cell = self.world_to_grid(x, y)
        if cell is None:
            return None
        grid_x, grid_y = cell
        return self._distance_cells[grid_y * self.width + grid_x] * self.resolution


def scan_map_metrics(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    map_to_base: Transform3D,
    base_to_scan: Transform3D,
    distance_field: OccupancyDistanceField,
    agreement_distance_m: float = 0.20,
    residual_cap_m: float = 2.0,
    max_scan_points: int = 720,
) -> ScanMapMetrics:
    if angle_increment <= 0.0 or range_min < 0.0 or range_max <= range_min:
        raise ValueError("invalid scan geometry")
    if agreement_distance_m < 0.0 or residual_cap_m <= 0.0 or max_scan_points <= 0:
        raise ValueError("invalid scan-map metric limits")

    valid_indices = [
        index
        for index, value in enumerate(ranges)
        if math.isfinite(value) and range_min <= value <= range_max
    ]
    valid_count = len(valid_indices)
    if valid_count > max_scan_points:
        step = math.ceil(valid_count / max_scan_points)
        valid_indices = valid_indices[::step]

    residuals: list[float] = []
    supported = 0
    for index in valid_indices:
        distance = float(ranges[index])
        angle = angle_min + float(index) * angle_increment
        point_scan = (distance * math.cos(angle), distance * math.sin(angle), 0.0)
        point_base = transform_point(base_to_scan, point_scan)
        point_map = transform_point(map_to_base, point_base)
        residual = distance_field.distance_at_world(point_map[0], point_map[1])
        if residual is None:
            continue
        residuals.append(min(residual, residual_cap_m))
        if residual <= agreement_distance_m:
            supported += 1

    if not residuals:
        return ScanMapMetrics(valid_count, len(valid_indices), 0, 0, None, None, None)
    residuals.sort()
    percentile_index = max(0, math.ceil(0.95 * len(residuals)) - 1)
    return ScanMapMetrics(
        valid_point_count=valid_count,
        sampled_point_count=len(valid_indices),
        points_in_map=len(residuals),
        supported_point_count=supported,
        agreement=float(supported) / float(len(residuals)),
        mean_residual_m=sum(residuals) / float(len(residuals)),
        p95_residual_m=residuals[percentile_index],
    )


def evaluate_integrity(
    metrics: IntegrityMetrics, thresholds: IntegrityThresholds
) -> EvaluationResult:
    reasons: list[str] = []
    blocking_unknown = False

    def missing(condition: bool, reason: Reason) -> None:
        nonlocal blocking_unknown
        if condition:
            reasons.append(reason.value)
            blocking_unknown = True

    missing(not metrics.map_ready, Reason.MAP_UNAVAILABLE)
    missing(not metrics.global_pose_ready, Reason.GLOBAL_POSE_UNAVAILABLE)
    missing(not metrics.scan_ready, Reason.SCAN_UNAVAILABLE)
    missing(not metrics.odom_ready, Reason.ODOM_UNAVAILABLE)
    missing(
        metrics.global_pose_ready and metrics.scan_ready and not metrics.tf_ready,
        Reason.TF_LOOKUP_FAILED,
    )

    if metrics.global_pose_time_reset:
        reasons.append(Reason.GLOBAL_POSE_TIME_RESET.value)
    if metrics.scan_time_reset:
        reasons.append(Reason.SCAN_TIME_RESET.value)
    if metrics.odom_time_reset:
        reasons.append(Reason.ODOM_TIME_RESET.value)
    if _exceeds(metrics.global_pose_age_sec, thresholds.max_global_pose_age_sec):
        reasons.append(Reason.GLOBAL_POSE_STALE.value)
        blocking_unknown = True
    if _exceeds(metrics.scan_age_sec, thresholds.max_scan_age_sec):
        reasons.append(Reason.SCAN_STALE.value)
        blocking_unknown = True
    if _exceeds(metrics.pose_scan_dt_sec, thresholds.max_pose_scan_dt_sec):
        reasons.append(Reason.POSE_SCAN_TIME_MISMATCH.value)
        blocking_unknown = True
    if _exceeds(metrics.pose_odom_dt_sec, thresholds.max_pose_odom_dt_sec):
        reasons.append(Reason.ODOM_TIME_MISMATCH.value)
        blocking_unknown = True
    if _exceeds(metrics.tf_scan_dt_sec, thresholds.max_tf_scan_dt_sec):
        reasons.append(Reason.TF_TIME_MISMATCH.value)
        blocking_unknown = True
    if metrics.scan_ready and metrics.valid_scan_points < thresholds.min_valid_scan_points:
        reasons.append(Reason.INSUFFICIENT_SCAN_POINTS.value)
        blocking_unknown = True
    if (
        metrics.map_ready
        and metrics.global_pose_ready
        and metrics.scan_ready
        and metrics.tf_ready
        and not _exceeds(metrics.pose_scan_dt_sec, thresholds.max_pose_scan_dt_sec)
        and metrics.valid_scan_points >= thresholds.min_valid_scan_points
        and not metrics.scan_map_ready
    ):
        reasons.append(Reason.SCAN_MAP_EVIDENCE_UNAVAILABLE.value)
        blocking_unknown = True

    soft_jump = False
    hard_jump = False
    soft_jump |= _append_threshold_reason(
        reasons,
        metrics.global_translation_jump_m,
        thresholds.max_global_translation_jump_warn_m,
        Reason.GLOBAL_TRANSLATION_JUMP,
    )
    hard_jump |= _exceeds(
        metrics.global_translation_jump_m,
        thresholds.max_global_translation_jump_reject_m,
    )
    soft_jump |= _append_threshold_reason(
        reasons,
        metrics.global_yaw_jump_rad,
        thresholds.max_global_yaw_jump_warn_rad,
        Reason.GLOBAL_YAW_JUMP,
    )
    hard_jump |= _exceeds(
        metrics.global_yaw_jump_rad, thresholds.max_global_yaw_jump_reject_rad
    )
    soft_jump |= _append_threshold_reason(
        reasons,
        metrics.correction_translation_jump_m,
        thresholds.max_correction_translation_jump_warn_m,
        Reason.CORRECTION_TRANSLATION_JUMP,
    )
    hard_jump |= _exceeds(
        metrics.correction_translation_jump_m,
        thresholds.max_correction_translation_jump_reject_m,
    )
    soft_jump |= _append_threshold_reason(
        reasons,
        metrics.correction_yaw_jump_rad,
        thresholds.max_correction_yaw_jump_warn_rad,
        Reason.CORRECTION_YAW_JUMP,
    )
    hard_jump |= _exceeds(
        metrics.correction_yaw_jump_rad,
        thresholds.max_correction_yaw_jump_reject_rad,
    )
    _append_threshold_reason(
        reasons,
        metrics.correction_translation_rate_mps,
        thresholds.max_correction_translation_rate_warn_mps,
        Reason.CORRECTION_TRANSLATION_RATE,
    )
    _append_threshold_reason(
        reasons,
        metrics.correction_yaw_rate_rps,
        thresholds.max_correction_yaw_rate_warn_rps,
        Reason.CORRECTION_YAW_RATE,
    )

    poor_scan = False
    hard_scan = False
    if metrics.scan_map_agreement is not None:
        poor_scan = metrics.scan_map_agreement < thresholds.min_scan_map_agreement_warn
        hard_scan = metrics.scan_map_agreement < thresholds.min_scan_map_agreement_reject
        if poor_scan:
            reasons.append(Reason.SCAN_MAP_AGREEMENT_LOW.value)
    if _exceeds(
        metrics.scan_map_mean_residual_m,
        thresholds.max_scan_map_mean_residual_warn_m,
    ):
        reasons.append(Reason.SCAN_MAP_RESIDUAL_HIGH.value)
        poor_scan = True

    reasons = list(dict.fromkeys(reasons))
    if hard_jump and hard_scan and not blocking_unknown:
        state = IntegrityState.REJECT
    elif (
        soft_jump
        or poor_scan
        or metrics.global_pose_time_reset
        or metrics.scan_time_reset
        or metrics.odom_time_reset
    ):
        state = IntegrityState.SUSPECT
    elif blocking_unknown:
        state = IntegrityState.UNKNOWN
    else:
        state = IntegrityState.GOOD
    return EvaluationResult(state=state, reasons=tuple(reasons))


def metrics_record(
    metrics: IntegrityMetrics, result: EvaluationResult
) -> dict[str, object]:
    return {
        "schema_version": 1,
        "state": result.state.value,
        "reasons": list(result.reasons),
        "metrics": metrics.to_dict(),
    }


def _exceeds(value: float | None, limit: float) -> bool:
    return value is not None and math.isfinite(value) and value > limit


def _append_threshold_reason(
    reasons: list[str], value: float | None, limit: float, reason: Reason
) -> bool:
    if _exceeds(value, limit):
        reasons.append(reason.value)
        return True
    return False


def finite_values(values: Iterable[float | None]) -> list[float]:
    return [float(value) for value in values if value is not None and math.isfinite(value)]
