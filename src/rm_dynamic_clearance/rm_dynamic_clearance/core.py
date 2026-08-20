"""ROS-independent dynamic-clearance admission evaluation."""

from __future__ import annotations

import bisect
from dataclasses import dataclass
from enum import IntEnum
import math
import re
from typing import Sequence


DYNAMIC_CLEARANCE_REPORT_SCHEMA = "rm_dynamic_clearance_report/v1"
DYNAMIC_OBSTACLE_PREDICTIONS_SCHEMA = "rm_dynamic_obstacle_predictions/v1"
AUTHORITY_SHADOW_ONLY = "shadow_only"
ADMISSION_NONE = 0
ADMISSION_DYNAMIC_CLEARANCE = 1
TRACK_TENTATIVE = 1
TRACK_CONFIRMED = 2
TRACK_COASTING = 3
MAX_PATH_POINTS = 20_000
MAX_INTENT_SEGMENTS = 10_000
MAX_TRACKS = 256
MAX_PREDICTION_STEPS = 1_000
MAX_CLEARANCE_SAMPLES = 10_000
MAX_CLEARANCE_COMPARISONS = 1_000_000
STAMP_ROUNDING_TOLERANCE = 1.0e-6
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class ClearanceContractError(ValueError):
    """Raised when an input violates the bounded sidecar contract."""


class Decision(IntEnum):
    UNKNOWN = 0
    CLEAR = 1
    BLOCKED = 2


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class IntentSegment:
    start_distance: float
    end_distance: float
    region_ids: tuple[str, ...]
    blocked: bool
    max_linear_speed: float | None
    admission_policy: int
    traversal_policy: int


@dataclass(frozen=True)
class PredictedTrack:
    track_id: int
    state: int
    position: Point2D
    velocity: Point2D
    size_x: float
    size_y: float
    last_observation_stamp: float
    observation_count: int
    miss_count: int
    prediction: tuple[Point2D, ...]


@dataclass(frozen=True)
class PredictionFrame:
    frame_id: str
    source_stamp: float
    processing_stamp: float
    prediction_dt: float
    prediction_steps: int
    complete: bool
    total_track_count: int
    tracks: tuple[PredictedTrack, ...]
    schema: str = DYNAMIC_OBSTACLE_PREDICTIONS_SCHEMA
    authority: str = AUTHORITY_SHADOW_ONLY


@dataclass(frozen=True)
class ClearanceConfig:
    nominal_speed: float = 1.0
    sample_resolution: float = 0.10
    lookahead_distance: float = 6.0
    robot_radius: float = 0.35
    minimum_obstacle_radius: float = 0.15
    safety_margin: float = 0.15
    coasting_extra_margin: float = 0.15
    max_prediction_age: float = 0.40
    max_future_prediction: float = 0.05
    max_path_projection_distance: float = 1.0
    projection_ambiguity_distance: float = 0.03
    projection_ambiguity_progress: float = 0.50
    include_coasting: bool = True


@dataclass(frozen=True)
class ClearanceReport:
    decision: Decision
    reason: str
    path_revision: str
    region_set_sha256: str
    prediction_stamp: float
    region_ids: tuple[str, ...]
    traversal_policy: int
    route_progress: float
    start_distance: float
    end_distance: float
    entry_eta: float
    prediction_horizon: float
    sample_count: int
    minimum_clearance: float | None
    blocking_track_ids: tuple[int, ...]


@dataclass(frozen=True)
class _Polyline:
    points: tuple[Point2D, ...]
    cumulative: tuple[float, ...]
    total_length: float

    def sample(self, distance: float) -> Point2D:
        clamped = min(self.total_length, max(0.0, distance))
        index = min(
            len(self.points) - 2,
            max(0, bisect.bisect_right(self.cumulative, clamped) - 1),
        )
        begin = self.cumulative[index]
        end = self.cumulative[index + 1]
        fraction = min(1.0, max(0.0, (clamped - begin) / (end - begin)))
        first = self.points[index]
        second = self.points[index + 1]
        return Point2D(
            first.x + fraction * (second.x - first.x),
            first.y + fraction * (second.y - first.y),
        )

    def project(
        self,
        point: Point2D,
        *,
        max_distance: float,
        ambiguity_distance: float,
        ambiguity_progress: float,
    ) -> float:
        candidates: list[tuple[float, float]] = []
        for index in range(len(self.points) - 1):
            first = self.points[index]
            second = self.points[index + 1]
            dx = second.x - first.x
            dy = second.y - first.y
            squared_length = dx * dx + dy * dy
            fraction = min(
                1.0,
                max(
                    0.0,
                    ((point.x - first.x) * dx + (point.y - first.y) * dy)
                    / squared_length,
                ),
            )
            projected_x = first.x + fraction * dx
            projected_y = first.y + fraction * dy
            distance = math.hypot(point.x - projected_x, point.y - projected_y)
            progress = self.cumulative[index] + fraction * math.sqrt(squared_length)
            candidates.append((distance, progress))
        minimum_distance = min(item[0] for item in candidates)
        if minimum_distance > max_distance:
            raise ClearanceContractError("robot pose is too far from the active path")
        plausible = [
            progress
            for distance, progress in candidates
            if distance <= minimum_distance + ambiguity_distance
        ]
        if max(plausible) - min(plausible) > ambiguity_progress:
            raise ClearanceContractError("robot path projection is ambiguous")
        return min(candidates, key=lambda item: item[0])[1]


@dataclass(frozen=True)
class _SpeedPiece:
    begin: float
    end: float
    begin_time: float
    speed: float


@dataclass(frozen=True)
class _SpeedProfile:
    pieces: tuple[_SpeedPiece, ...]
    ends: tuple[float, ...]

    def arrival_time(self, distance: float) -> float:
        clamped = min(self.pieces[-1].end, max(0.0, distance))
        index = min(
            len(self.pieces) - 1,
            bisect.bisect_left(self.ends, clamped),
        )
        piece = self.pieces[index]
        return piece.begin_time + (clamped - piece.begin) / piece.speed


def _finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ClearanceContractError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ClearanceContractError(f"{name} must be finite")
    return result


def _positive(value: float, name: str, *, allow_zero: bool = False) -> float:
    result = _finite(value, name)
    if result < 0.0 or (not allow_zero and result == 0.0):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ClearanceContractError(f"{name} must be {qualifier}")
    return result


def validate_clearance_config(config: ClearanceConfig) -> None:
    """Validate bounded evaluator parameters before subscriptions start."""
    _positive(config.nominal_speed, "nominal_speed")
    _positive(config.sample_resolution, "sample_resolution")
    _positive(config.lookahead_distance, "lookahead_distance")
    _positive(config.robot_radius, "robot_radius", allow_zero=True)
    _positive(
        config.minimum_obstacle_radius,
        "minimum_obstacle_radius",
        allow_zero=True,
    )
    _positive(config.safety_margin, "safety_margin", allow_zero=True)
    _positive(
        config.coasting_extra_margin,
        "coasting_extra_margin",
        allow_zero=True,
    )
    _positive(config.max_prediction_age, "max_prediction_age", allow_zero=True)
    _positive(
        config.max_future_prediction,
        "max_future_prediction",
        allow_zero=True,
    )
    _positive(
        config.max_path_projection_distance,
        "max_path_projection_distance",
    )
    _positive(
        config.projection_ambiguity_distance,
        "projection_ambiguity_distance",
        allow_zero=True,
    )
    _positive(
        config.projection_ambiguity_progress,
        "projection_ambiguity_progress",
        allow_zero=True,
    )


def _build_polyline(points: Sequence[Point2D]) -> _Polyline:
    if not 2 <= len(points) <= MAX_PATH_POINTS:
        raise ClearanceContractError(
            f"path must contain 2..{MAX_PATH_POINTS} points"
        )
    checked: list[Point2D] = []
    cumulative = [0.0]
    for index, point in enumerate(points):
        x = _finite(point.x, f"path[{index}].x")
        y = _finite(point.y, f"path[{index}].y")
        candidate = Point2D(x, y)
        if not checked:
            checked.append(candidate)
            continue
        previous = checked[-1]
        length = math.hypot(x - previous.x, y - previous.y)
        if length <= 1.0e-12:
            continue
        checked.append(candidate)
        cumulative.append(cumulative[-1] + length)
    if cumulative[-1] <= 1.0e-9:
        raise ClearanceContractError("path length must be positive")
    return _Polyline(tuple(checked), tuple(cumulative), cumulative[-1])


def _validate_segments(
    segments: Sequence[IntentSegment], path_length: float
) -> tuple[IntentSegment, ...]:
    if len(segments) > MAX_INTENT_SEGMENTS:
        raise ClearanceContractError(
            f"annotation exceeds {MAX_INTENT_SEGMENTS} segments"
        )
    checked: list[IntentSegment] = []
    previous_end = 0.0
    for index, segment in enumerate(segments):
        start = _positive(
            segment.start_distance,
            f"segments[{index}].start_distance",
            allow_zero=True,
        )
        end = _positive(segment.end_distance, f"segments[{index}].end_distance")
        if end <= start or end > path_length + 1.0e-6:
            raise ClearanceContractError(f"segments[{index}] has invalid bounds")
        if index and start < previous_end - 1.0e-9:
            raise ClearanceContractError("annotated segments must not overlap")
        if not segment.region_ids or len(segment.region_ids) > 256:
            raise ClearanceContractError(f"segments[{index}] has invalid region_ids")
        if any(
            not isinstance(region_id, str)
            or not region_id
            or len(region_id.encode("utf-8")) > 128
            for region_id in segment.region_ids
        ):
            raise ClearanceContractError(f"segments[{index}] has invalid region_id")
        if segment.admission_policy not in (
            ADMISSION_NONE,
            ADMISSION_DYNAMIC_CLEARANCE,
        ):
            raise ClearanceContractError(
                f"segments[{index}] has unknown admission policy"
            )
        if segment.traversal_policy not in (0, 1):
            raise ClearanceContractError(
                f"segments[{index}] has unknown traversal policy"
            )
        speed = segment.max_linear_speed
        if speed is not None:
            speed = _positive(speed, f"segments[{index}].max_linear_speed")
        checked.append(
            IntentSegment(
                start,
                end,
                tuple(segment.region_ids),
                bool(segment.blocked),
                speed,
                segment.admission_policy,
                segment.traversal_policy,
            )
        )
        previous_end = end
    return tuple(checked)


def _validate_prediction(frame: PredictionFrame, expected_frame: str) -> None:
    if frame.schema != DYNAMIC_OBSTACLE_PREDICTIONS_SCHEMA:
        raise ClearanceContractError("unsupported prediction schema")
    if frame.authority != AUTHORITY_SHADOW_ONLY:
        raise ClearanceContractError("unexpected prediction authority")
    if frame.frame_id != expected_frame or not frame.frame_id:
        raise ClearanceContractError("prediction frame does not match path frame")
    source_stamp = _positive(frame.source_stamp, "prediction source_stamp")
    processing_stamp = _positive(
        frame.processing_stamp, "prediction processing_stamp"
    )
    if processing_stamp + STAMP_ROUNDING_TOLERANCE < source_stamp:
        raise ClearanceContractError("prediction processing stamp precedes source")
    _positive(frame.prediction_dt, "prediction_dt")
    if (
        isinstance(frame.prediction_steps, bool)
        or not isinstance(frame.prediction_steps, int)
        or not 1 <= frame.prediction_steps <= MAX_PREDICTION_STEPS
    ):
        raise ClearanceContractError(
            f"prediction_steps must be in 1..{MAX_PREDICTION_STEPS}"
        )
    if len(frame.tracks) > MAX_TRACKS:
        raise ClearanceContractError(f"prediction exceeds {MAX_TRACKS} tracks")
    if (
        isinstance(frame.total_track_count, bool)
        or not isinstance(frame.total_track_count, int)
        or frame.total_track_count < len(frame.tracks)
        or frame.total_track_count > 1_000_000
    ):
        raise ClearanceContractError("prediction total_track_count is invalid")
    if bool(frame.complete) != (frame.total_track_count == len(frame.tracks)):
        raise ClearanceContractError("prediction completeness fields disagree")
    track_ids: set[int] = set()
    for index, track in enumerate(frame.tracks):
        if (
            isinstance(track.track_id, bool)
            or not isinstance(track.track_id, int)
            or track.track_id <= 0
            or track.track_id in track_ids
        ):
            raise ClearanceContractError(f"track {index} has invalid/duplicate id")
        track_ids.add(track.track_id)
        if track.state not in (TRACK_TENTATIVE, TRACK_CONFIRMED, TRACK_COASTING):
            raise ClearanceContractError(f"track {track.track_id} has invalid state")
        for name, value in (
            ("position.x", track.position.x),
            ("position.y", track.position.y),
            ("velocity.x", track.velocity.x),
            ("velocity.y", track.velocity.y),
        ):
            _finite(value, f"track {track.track_id} {name}")
        _positive(track.size_x, f"track {track.track_id} size_x", allow_zero=True)
        _positive(track.size_y, f"track {track.track_id} size_y", allow_zero=True)
        last_stamp = _positive(
            track.last_observation_stamp,
            f"track {track.track_id} last_observation_stamp",
        )
        if last_stamp > frame.source_stamp + STAMP_ROUNDING_TOLERANCE:
            raise ClearanceContractError(
                f"track {track.track_id} observation is newer than source"
            )
        if track.observation_count <= 0 or track.miss_count < 0:
            raise ClearanceContractError(
                f"track {track.track_id} has invalid lifecycle counters"
            )
        if len(track.prediction) != frame.prediction_steps:
            raise ClearanceContractError(
                f"track {track.track_id} prediction length mismatch"
            )
        for point_index, point in enumerate(track.prediction):
            _finite(point.x, f"track {track.track_id} prediction[{point_index}].x")
            _finite(point.y, f"track {track.track_id} prediction[{point_index}].y")


def _build_speed_profile(
    path_length: float,
    segments: Sequence[IntentSegment],
    nominal_speed: float,
) -> _SpeedProfile:
    pieces: list[_SpeedPiece] = []
    cursor = 0.0
    elapsed = 0.0
    for segment in segments:
        if segment.start_distance > cursor:
            pieces.append(
                _SpeedPiece(cursor, segment.start_distance, elapsed, nominal_speed)
            )
            elapsed += (segment.start_distance - cursor) / nominal_speed
        speed = (
            min(nominal_speed, segment.max_linear_speed)
            if segment.max_linear_speed is not None
            else nominal_speed
        )
        pieces.append(
            _SpeedPiece(segment.start_distance, segment.end_distance, elapsed, speed)
        )
        elapsed += (segment.end_distance - segment.start_distance) / speed
        cursor = segment.end_distance
    if cursor < path_length:
        pieces.append(_SpeedPiece(cursor, path_length, elapsed, nominal_speed))
    if not pieces:
        pieces.append(_SpeedPiece(0.0, path_length, 0.0, nominal_speed))
    return _SpeedProfile(tuple(pieces), tuple(piece.end for piece in pieces))


def _track_position(track: PredictedTrack, time_from_source: float, dt: float) -> Point2D:
    if time_from_source <= 0.0:
        return track.position
    scaled = time_from_source / dt
    lower = int(math.floor(scaled))
    if lower >= len(track.prediction):
        return track.prediction[-1]
    fraction = scaled - lower
    first = track.position if lower == 0 else track.prediction[lower - 1]
    second = track.prediction[lower]
    return Point2D(
        first.x + fraction * (second.x - first.x),
        first.y + fraction * (second.y - first.y),
    )


def _base_report(
    decision: Decision,
    reason: str,
    path_revision: str,
    region_set_sha256: str,
    prediction: PredictionFrame,
    segment: IntentSegment | None,
    *,
    route_progress: float,
    entry_eta: float = 0.0,
    sample_count: int = 0,
    minimum_clearance: float | None = None,
    blocking_track_ids: tuple[int, ...] = (),
) -> ClearanceReport:
    return ClearanceReport(
        decision=decision,
        reason=reason,
        path_revision=path_revision,
        region_set_sha256=region_set_sha256,
        prediction_stamp=prediction.source_stamp,
        region_ids=segment.region_ids if segment else (),
        traversal_policy=segment.traversal_policy if segment else 0,
        route_progress=route_progress,
        start_distance=segment.start_distance if segment else 0.0,
        end_distance=segment.end_distance if segment else 0.0,
        entry_eta=entry_eta,
        prediction_horizon=prediction.prediction_dt * prediction.prediction_steps,
        sample_count=sample_count,
        minimum_clearance=minimum_clearance,
        blocking_track_ids=blocking_track_ids,
    )


def evaluate_dynamic_clearance(
    *,
    path_frame: str,
    path_points: Sequence[Point2D],
    robot_position: Point2D,
    path_revision: str,
    annotated_path_revision: str,
    annotated_path_length: float,
    region_set_sha256: str,
    segments: Sequence[IntentSegment],
    prediction: PredictionFrame,
    now: float,
    config: ClearanceConfig = ClearanceConfig(),
) -> ClearanceReport:
    """
    Evaluate the nearest upcoming dynamic-clearance interval.

    Robot position is projected onto the unchanged global plan. Remaining
    arrival time is derived from that progress, configured nominal speed and
    annotated speed caps. The result is shadow-only data and never grants
    motion authority by itself.
    """
    validate_clearance_config(config)
    if not isinstance(path_frame, str) or not path_frame:
        raise ClearanceContractError("path_frame must be non-empty")
    if not _SHA256.fullmatch(path_revision):
        raise ClearanceContractError("path_revision must be lowercase SHA256")
    if path_revision != annotated_path_revision:
        raise ClearanceContractError("annotated path revision does not match path")
    if not _SHA256.fullmatch(region_set_sha256):
        raise ClearanceContractError("region_set_sha256 must be lowercase SHA256")
    now = _positive(now, "now")
    polyline = _build_polyline(path_points)
    robot_point = Point2D(
        _finite(robot_position.x, "robot_position.x"),
        _finite(robot_position.y, "robot_position.y"),
    )
    route_progress = polyline.project(
        robot_point,
        max_distance=config.max_path_projection_distance,
        ambiguity_distance=config.projection_ambiguity_distance,
        ambiguity_progress=config.projection_ambiguity_progress,
    )
    annotated_length = _positive(annotated_path_length, "annotated_path_length")
    tolerance = max(1.0e-6, polyline.total_length * 1.0e-9)
    if abs(annotated_length - polyline.total_length) > tolerance:
        raise ClearanceContractError("annotated path length does not match path")
    checked_segments = _validate_segments(segments, polyline.total_length)
    _validate_prediction(prediction, path_frame)
    speed_profile = _build_speed_profile(
        polyline.total_length, checked_segments, config.nominal_speed
    )

    target = next(
        (
            segment
            for segment in checked_segments
            if segment.admission_policy == ADMISSION_DYNAMIC_CLEARANCE
            and segment.end_distance > route_progress + 1.0e-6
        ),
        None,
    )
    if target is None:
        return _base_report(
            Decision.CLEAR,
            "no_admission_required",
            path_revision,
            region_set_sha256,
            prediction,
            None,
            route_progress=route_progress,
        )
    entry_eta = max(
        0.0,
        speed_profile.arrival_time(target.start_distance)
        - speed_profile.arrival_time(route_progress),
    )
    if route_progress >= target.start_distance - 1.0e-6:
        return _base_report(
            Decision.UNKNOWN,
            "already_in_admission_region",
            path_revision,
            region_set_sha256,
            prediction,
            target,
            route_progress=route_progress,
            entry_eta=entry_eta,
        )
    if target.start_distance - route_progress > config.lookahead_distance:
        return _base_report(
            Decision.UNKNOWN,
            "admission_beyond_lookahead",
            path_revision,
            region_set_sha256,
            prediction,
            target,
            route_progress=route_progress,
            entry_eta=entry_eta,
        )
    if target.blocked:
        return _base_report(
            Decision.BLOCKED,
            "semantic_region_blocked",
            path_revision,
            region_set_sha256,
            prediction,
            target,
            route_progress=route_progress,
            entry_eta=entry_eta,
        )
    if not prediction.complete:
        return _base_report(
            Decision.UNKNOWN,
            "prediction_track_limit",
            path_revision,
            region_set_sha256,
            prediction,
            target,
            route_progress=route_progress,
            entry_eta=entry_eta,
        )

    source_age = now - prediction.source_stamp
    if source_age < -config.max_future_prediction:
        return _base_report(
            Decision.UNKNOWN,
            "prediction_from_future",
            path_revision,
            region_set_sha256,
            prediction,
            target,
            route_progress=route_progress,
            entry_eta=entry_eta,
        )
    if source_age > config.max_prediction_age:
        return _base_report(
            Decision.UNKNOWN,
            "prediction_stale",
            path_revision,
            region_set_sha256,
            prediction,
            target,
            route_progress=route_progress,
            entry_eta=entry_eta,
        )

    interval_length = target.end_distance - target.start_distance
    sample_count = max(2, int(math.ceil(interval_length / config.sample_resolution)) + 1)
    if sample_count > MAX_CLEARANCE_SAMPLES:
        raise ClearanceContractError(
            f"clearance sampling exceeds {MAX_CLEARANCE_SAMPLES} points"
        )
    distances = tuple(
        target.start_distance
        + interval_length * index / float(sample_count - 1)
        for index in range(sample_count)
    )
    arrival_times = tuple(
        speed_profile.arrival_time(distance)
        - speed_profile.arrival_time(route_progress)
        for distance in distances
    )
    required_horizon = max(0.0, source_age) + arrival_times[-1]
    prediction_horizon = prediction.prediction_dt * prediction.prediction_steps
    if required_horizon > prediction_horizon + 1.0e-9:
        return _base_report(
            Decision.UNKNOWN,
            "prediction_horizon_short",
            path_revision,
            region_set_sha256,
            prediction,
            target,
            route_progress=route_progress,
            entry_eta=entry_eta,
            sample_count=sample_count,
        )

    active_tracks = tuple(
        track
        for track in prediction.tracks
        if track.state in (TRACK_TENTATIVE, TRACK_CONFIRMED)
        or (config.include_coasting and track.state == TRACK_COASTING)
    )
    if sample_count * len(active_tracks) > MAX_CLEARANCE_COMPARISONS:
        raise ClearanceContractError(
            f"clearance evaluation exceeds {MAX_CLEARANCE_COMPARISONS} comparisons"
        )
    minimum_clearance: float | None = None
    blocking: set[int] = set()
    uncertain: set[int] = set()
    for distance, arrival_time in zip(distances, arrival_times):
        path_point = polyline.sample(distance)
        prediction_time = max(0.0, source_age + arrival_time)
        for track in active_tracks:
            obstacle_position = _track_position(
                track, prediction_time, prediction.prediction_dt
            )
            obstacle_radius = max(
                config.minimum_obstacle_radius,
                0.5 * math.hypot(track.size_x, track.size_y),
            )
            clearance = (
                math.hypot(
                    path_point.x - obstacle_position.x,
                    path_point.y - obstacle_position.y,
                )
                - config.robot_radius
                - obstacle_radius
            )
            if minimum_clearance is None or clearance < minimum_clearance:
                minimum_clearance = clearance
            margin = config.safety_margin
            if track.state == TRACK_COASTING:
                margin += config.coasting_extra_margin
            if clearance <= margin:
                if track.state == TRACK_TENTATIVE:
                    uncertain.add(track.track_id)
                else:
                    blocking.add(track.track_id)

    decision = (
        Decision.BLOCKED
        if blocking
        else Decision.UNKNOWN
        if uncertain
        else Decision.CLEAR
    )
    reason = (
        "predicted_overlap"
        if blocking
        else "tentative_overlap"
        if uncertain
        else "clear"
    )
    return _base_report(
        decision,
        reason,
        path_revision,
        region_set_sha256,
        prediction,
        target,
        route_progress=route_progress,
        entry_eta=entry_eta,
        sample_count=sample_count,
        minimum_clearance=minimum_clearance,
        blocking_track_ids=tuple(sorted(blocking | uncertain)),
    )
