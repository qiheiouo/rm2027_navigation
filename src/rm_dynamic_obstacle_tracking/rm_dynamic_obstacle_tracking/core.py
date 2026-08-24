"""ROS-independent dynamic obstacle extraction, clustering, and tracking."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Detection:
    centroid: Point2D
    size_x: float
    size_y: float
    point_count: int


class TrackState(str, Enum):
    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    COASTING = "coasting"
    LOST = "lost"


@dataclass(frozen=True)
class TrackSnapshot:
    track_id: int
    state: TrackState
    position: Point2D
    velocity: Point2D
    size_x: float
    size_y: float
    timestamp: float
    last_observation_timestamp: float
    age_sec: float
    observations: int
    misses: int
    prediction: tuple[Point2D, ...]


@dataclass(frozen=True)
class TrackerUpdate:
    tracks: tuple[TrackSnapshot, ...]
    created: int
    deleted: int
    time_reset: bool


def planar_rotation_matrix(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return the XY block of a normalized 3D quaternion rotation matrix."""
    qx, qy, qz, qw = quaternion
    norm_squared = qx * qx + qy * qy + qz * qz + qw * qw
    if not math.isfinite(norm_squared) or norm_squared <= 1.0e-12:
        raise ValueError("transform quaternion must be finite and non-zero")
    inverse_norm = 1.0 / math.sqrt(norm_squared)
    qx *= inverse_norm
    qy *= inverse_norm
    qz *= inverse_norm
    qw *= inverse_norm
    rotation_xx = 1.0 - 2.0 * (qy * qy + qz * qz)
    rotation_xy = 2.0 * (qx * qy - qz * qw)
    rotation_yx = 2.0 * (qx * qy + qz * qw)
    rotation_yy = 1.0 - 2.0 * (qx * qx + qz * qz)
    return rotation_xx, rotation_xy, rotation_yx, rotation_yy


class OccupancyMap:
    """Minimal occupancy map with planar origin support."""

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        data: Sequence[int],
        occupied_threshold: int = 65,
    ) -> None:
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise ValueError("map dimensions and resolution must be positive")
        if len(data) != width * height:
            raise ValueError("map data length does not match dimensions")
        if not all(
            math.isfinite(value)
            for value in (resolution, origin_x, origin_y, origin_yaw)
        ):
            raise ValueError("map geometry must be finite")
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_yaw = origin_yaw
        self.data = tuple(int(value) for value in data)
        self.occupied_threshold = int(occupied_threshold)
        self._origin_cosine = math.cos(self.origin_yaw)
        self._origin_sine = math.sin(self.origin_yaw)
        occupied_x_by_row: dict[int, list[int]] = {}
        for index, value in enumerate(self.data):
            if value >= self.occupied_threshold:
                occupied_x_by_row.setdefault(index // self.width, []).append(
                    index % self.width
                )
        self._occupied_x_by_row = {
            cell_y: tuple(cell_xs)
            for cell_y, cell_xs in occupied_x_by_row.items()
        }
        self._occupied_centers = tuple(
            (
                (cell_x + 0.5) * self.resolution,
                (cell_y + 0.5) * self.resolution,
            )
            for cell_y, cell_xs in self._occupied_x_by_row.items()
            for cell_x in cell_xs
        )
        self._occupied_bucket_cache: dict[
            float, dict[tuple[int, int], tuple[tuple[float, float], ...]]
        ] = {}

    def _world_to_local(self, point: Point2D) -> tuple[float, float]:
        dx = point.x - self.origin_x
        dy = point.y - self.origin_y
        return (
            self._origin_cosine * dx + self._origin_sine * dy,
            -self._origin_sine * dx + self._origin_cosine * dy,
        )

    def world_to_cell(self, point: Point2D) -> tuple[int, int] | None:
        local_x, local_y = self._world_to_local(point)
        cell_x = math.floor(local_x / self.resolution)
        cell_y = math.floor(local_y / self.resolution)
        if cell_x < 0 or cell_y < 0 or cell_x >= self.width or cell_y >= self.height:
            return None
        return cell_x, cell_y

    def value_at_world(self, point: Point2D) -> int | None:
        cell = self.world_to_cell(point)
        if cell is None:
            return None
        return self.data[cell[1] * self.width + cell[0]]

    def distance_to_occupied(self, point: Point2D, search_radius: float) -> float | None:
        cell = self.world_to_cell(point)
        if cell is None:
            return None
        local_x, local_y = self._world_to_local(point)
        if search_radius <= 0.0:
            occupied_xs = self._occupied_x_by_row.get(cell[1], ())
            if cell[0] not in occupied_xs:
                return None
            center_x = (cell[0] + 0.5) * self.resolution
            center_y = (cell[1] + 0.5) * self.resolution
            return math.hypot(local_x - center_x, local_y - center_y)

        buckets = self._occupied_bucket_cache.get(search_radius)
        if buckets is None:
            mutable_buckets: dict[
                tuple[int, int], list[tuple[float, float]]
            ] = {}
            for center_x, center_y in self._occupied_centers:
                key = (
                    math.floor(center_x / search_radius),
                    math.floor(center_y / search_radius),
                )
                mutable_buckets.setdefault(key, []).append((center_x, center_y))
            buckets = {
                key: tuple(centers) for key, centers in mutable_buckets.items()
            }
            self._occupied_bucket_cache[search_radius] = buckets

        bucket_x = math.floor(local_x / search_radius)
        bucket_y = math.floor(local_y / search_radius)
        best = math.inf
        for offset_y in (-1, 0, 1):
            for offset_x in (-1, 0, 1):
                for center_x, center_y in buckets.get(
                    (bucket_x + offset_x, bucket_y + offset_y), ()
                ):
                    best = min(
                        best, math.hypot(local_x - center_x, local_y - center_y)
                    )
        return None if math.isinf(best) else best


def dynamic_candidates(
    points: Iterable[Point2D],
    occupancy_map: OccupancyMap,
    static_distance_threshold: float,
    require_known_free: bool = True,
) -> list[Point2D]:
    """Return endpoints not explained by the static occupancy map."""
    if static_distance_threshold < 0.0:
        raise ValueError("static distance threshold must not be negative")
    result: list[Point2D] = []
    for point in points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            continue
        value = occupancy_map.value_at_world(point)
        if value is None or (require_known_free and value < 0):
            continue
        distance = occupancy_map.distance_to_occupied(
            point, static_distance_threshold
        )
        if distance is None or distance > static_distance_threshold:
            result.append(point)
    return result


def cluster_points(
    points: Sequence[Point2D],
    tolerance: float,
    min_points: int,
    max_extent: float,
) -> list[Detection]:
    """Euclidean connected clustering accelerated by metric buckets."""
    if tolerance <= 0.0 or min_points <= 0 or max_extent <= 0.0:
        raise ValueError("clustering parameters must be positive")
    buckets: dict[tuple[int, int], list[int]] = {}
    for index, point in enumerate(points):
        key = (math.floor(point.x / tolerance), math.floor(point.y / tolerance))
        buckets.setdefault(key, []).append(index)

    visited: set[int] = set()
    detections: list[Detection] = []
    tolerance_sq = tolerance * tolerance
    for seed in range(len(points)):
        if seed in visited:
            continue
        visited.add(seed)
        queue = [seed]
        component: list[int] = []
        while queue:
            current = queue.pop()
            component.append(current)
            point = points[current]
            bucket_x = math.floor(point.x / tolerance)
            bucket_y = math.floor(point.y / tolerance)
            for offset_y in (-1, 0, 1):
                for offset_x in (-1, 0, 1):
                    for candidate in buckets.get(
                        (bucket_x + offset_x, bucket_y + offset_y), ()
                    ):
                        if candidate in visited:
                            continue
                        other = points[candidate]
                        distance_sq = (point.x - other.x) ** 2 + (point.y - other.y) ** 2
                        if distance_sq <= tolerance_sq:
                            visited.add(candidate)
                            queue.append(candidate)
        if len(component) < min_points:
            continue
        xs = [points[index].x for index in component]
        ys = [points[index].y for index in component]
        size_x = max(xs) - min(xs)
        size_y = max(ys) - min(ys)
        if max(size_x, size_y) > max_extent:
            continue
        detections.append(
            Detection(
                centroid=Point2D(sum(xs) / len(xs), sum(ys) / len(ys)),
                size_x=max(size_x, tolerance),
                size_y=max(size_y, tolerance),
                point_count=len(component),
            )
        )
    return detections


def filter_detections_near_static(
    detections: Sequence[Detection],
    occupancy_map: OccupancyMap,
    static_distance_threshold: float,
) -> list[Detection]:
    """Reject residual clusters whose centroid is still close to mapped structure."""
    if static_distance_threshold < 0.0:
        raise ValueError("detection static distance threshold must not be negative")
    if static_distance_threshold == 0.0:
        return list(detections)
    result = []
    for detection in detections:
        distance = occupancy_map.distance_to_occupied(
            detection.centroid, static_distance_threshold
        )
        if distance is None or distance > static_distance_threshold:
            result.append(detection)
    return result


def _hungarian_min_cost(costs: Sequence[Sequence[float]]) -> list[int]:
    """Return the minimum-cost column for every row of a square matrix."""
    size = len(costs)
    if size == 0:
        return []
    if any(len(row) != size for row in costs):
        raise ValueError("Hungarian cost matrix must be square")
    row_potential = [0.0] * (size + 1)
    column_potential = [0.0] * (size + 1)
    matched_row = [0] * (size + 1)
    previous_column = [0] * (size + 1)
    for row in range(1, size + 1):
        matched_row[0] = row
        minimum = [math.inf] * (size + 1)
        used = [False] * (size + 1)
        column = 0
        while True:
            used[column] = True
            active_row = matched_row[column]
            delta = math.inf
            next_column = 0
            for candidate_column in range(1, size + 1):
                if used[candidate_column]:
                    continue
                reduced_cost = (
                    costs[active_row - 1][candidate_column - 1]
                    - row_potential[active_row]
                    - column_potential[candidate_column]
                )
                if reduced_cost < minimum[candidate_column]:
                    minimum[candidate_column] = reduced_cost
                    previous_column[candidate_column] = column
                if minimum[candidate_column] < delta:
                    delta = minimum[candidate_column]
                    next_column = candidate_column
            for candidate_column in range(size + 1):
                if used[candidate_column]:
                    row_potential[matched_row[candidate_column]] += delta
                    column_potential[candidate_column] -= delta
                else:
                    minimum[candidate_column] -= delta
            column = next_column
            if matched_row[column] == 0:
                break
        while True:
            previous = previous_column[column]
            matched_row[column] = matched_row[previous]
            column = previous
            if column == 0:
                break
    assignment = [-1] * size
    for column in range(1, size + 1):
        if matched_row[column] > 0:
            assignment[matched_row[column] - 1] = column - 1
    return assignment


def optimal_gated_assignment(
    track_positions: Sequence[Point2D],
    detection_positions: Sequence[Point2D],
    association_gate: float,
) -> dict[int, int]:
    """Globally minimize gated distances while allowing either side unmatched."""
    if association_gate <= 0.0:
        raise ValueError("association gate must be positive")
    track_count = len(track_positions)
    detection_count = len(detection_positions)
    if track_count == 0 or detection_count == 0:
        return {}
    size = track_count + detection_count
    unmatched_cost = association_gate + 1.0e-6
    forbidden_cost = unmatched_cost * (2.0 * size + 1.0)
    costs = [[0.0] * size for _ in range(size)]
    for track_index, track in enumerate(track_positions):
        for detection_index, detection in enumerate(detection_positions):
            distance = math.hypot(track.x - detection.x, track.y - detection.y)
            costs[track_index][detection_index] = (
                distance if distance <= association_gate else forbidden_cost
            )
        for dummy_column in range(detection_count, size):
            costs[track_index][dummy_column] = unmatched_cost
    for dummy_row in range(track_count, size):
        for detection_index in range(detection_count):
            costs[dummy_row][detection_index] = unmatched_cost
    assignment = _hungarian_min_cost(costs)
    result = {}
    for track_index in range(track_count):
        detection_index = assignment[track_index]
        if detection_index < 0 or detection_index >= detection_count:
            continue
        if costs[track_index][detection_index] <= association_gate:
            result[track_index] = detection_index
    return result


class _Kalman1D:
    def __init__(self, position: float, initial_variance: float) -> None:
        self.position = position
        self.velocity = 0.0
        self.p00 = initial_variance
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = initial_variance

    def predict(self, dt: float, process_noise: float) -> None:
        self.position += self.velocity * dt
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        p00 = self.p00 + dt * (self.p01 + self.p10) + dt2 * self.p11
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11
        self.p00 = p00 + process_noise * dt4 / 4.0
        self.p01 = p01 + process_noise * dt3 / 2.0
        self.p10 = p10 + process_noise * dt3 / 2.0
        self.p11 = p11 + process_noise * dt2

    def update(self, measurement: float, measurement_noise: float) -> None:
        innovation = measurement - self.position
        innovation_variance = self.p00 + measurement_noise
        if innovation_variance <= 1.0e-12:
            return
        gain_position = self.p00 / innovation_variance
        gain_velocity = self.p10 / innovation_variance
        old_p00 = self.p00
        old_p01 = self.p01
        old_p10 = self.p10
        old_p11 = self.p11
        self.position += gain_position * innovation
        self.velocity += gain_velocity * innovation
        self.p00 = (1.0 - gain_position) * old_p00
        self.p01 = (1.0 - gain_position) * old_p01
        self.p10 = old_p10 - gain_velocity * old_p00
        self.p11 = old_p11 - gain_velocity * old_p01
        symmetric = 0.5 * (self.p01 + self.p10)
        self.p01 = symmetric
        self.p10 = symmetric


@dataclass
class _Track:
    track_id: int
    x_filter: _Kalman1D
    y_filter: _Kalman1D
    created_at: float
    last_update: float
    size_x: float
    size_y: float
    initial_x: float
    initial_y: float
    observations: int = 1
    consecutive_hits: int = 1
    misses: int = 0
    state: TrackState = TrackState.TENTATIVE


class MultiObjectTracker:
    def __init__(
        self,
        association_gate: float = 0.6,
        process_noise: float = 3.0,
        measurement_noise: float = 0.08,
        initial_variance: float = 1.0,
        min_hits_to_confirm: int = 3,
        min_displacement_to_confirm: float = 0.0,
        use_global_assignment: bool = False,
        tentative_max_misses: int = 1,
        max_coast_time_sec: float = 0.6,
        prediction_steps: int = 15,
        prediction_dt: float = 0.1,
        velocity_decay_tau: float = 1.5,
        max_prediction_speed: float = 3.0,
        max_update_dt: float = 0.5,
    ) -> None:
        if association_gate <= 0.0 or process_noise < 0.0:
            raise ValueError("invalid tracker noise or association parameters")
        if measurement_noise <= 0.0 or initial_variance <= 0.0:
            raise ValueError("tracker variances must be positive")
        if (
            min_hits_to_confirm <= 0
            or not math.isfinite(min_displacement_to_confirm)
            or min_displacement_to_confirm < 0.0
            or tentative_max_misses < 0
            or max_coast_time_sec < 0.0
        ):
            raise ValueError("invalid track lifecycle parameters")
        if prediction_steps < 0 or prediction_dt <= 0.0 or max_update_dt <= 0.0:
            raise ValueError("invalid tracker timing parameters")
        self.association_gate = association_gate
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.initial_variance = initial_variance
        self.min_hits_to_confirm = min_hits_to_confirm
        self.min_displacement_to_confirm = min_displacement_to_confirm
        self.use_global_assignment = use_global_assignment
        self.tentative_max_misses = tentative_max_misses
        self.max_coast_time_sec = max_coast_time_sec
        self.prediction_steps = prediction_steps
        self.prediction_dt = prediction_dt
        self.velocity_decay_tau = velocity_decay_tau
        self.max_prediction_speed = max_prediction_speed
        self.max_update_dt = max_update_dt
        self._tracks: list[_Track] = []
        self._next_id = 1
        self._last_stamp: float | None = None

    def update(self, detections: Sequence[Detection], stamp: float) -> TrackerUpdate:
        if not math.isfinite(stamp):
            raise ValueError("tracker timestamp must be finite")
        time_reset = self._last_stamp is not None and stamp <= self._last_stamp
        if time_reset:
            deleted = len(self._tracks)
            self._tracks.clear()
            self._last_stamp = None
        else:
            deleted = 0
        dt = 0.1 if self._last_stamp is None else min(
            stamp - self._last_stamp, self.max_update_dt
        )
        self._last_stamp = stamp

        active_tracks: list[_Track] = []
        for track in self._tracks:
            if stamp - track.last_update > self.max_coast_time_sec:
                track.state = TrackState.LOST
                deleted += 1
            else:
                active_tracks.append(track)
        self._tracks = active_tracks

        for track in self._tracks:
            track.x_filter.predict(dt, self.process_noise)
            track.y_filter.predict(dt, self.process_noise)

        if self.use_global_assignment:
            assignments = optimal_gated_assignment(
                [
                    Point2D(track.x_filter.position, track.y_filter.position)
                    for track in self._tracks
                ],
                [detection.centroid for detection in detections],
                self.association_gate,
            )
        else:
            pairs: list[tuple[float, int, int]] = []
            for track_index, track in enumerate(self._tracks):
                for detection_index, detection in enumerate(detections):
                    distance = math.hypot(
                        track.x_filter.position - detection.centroid.x,
                        track.y_filter.position - detection.centroid.y,
                    )
                    if distance <= self.association_gate:
                        pairs.append((distance, track_index, detection_index))
            pairs.sort()
            matched_tracks: set[int] = set()
            matched_detections: set[int] = set()
            assignments = {}
            for _, track_index, detection_index in pairs:
                if (
                    track_index in matched_tracks
                    or detection_index in matched_detections
                ):
                    continue
                matched_tracks.add(track_index)
                matched_detections.add(detection_index)
                assignments[track_index] = detection_index
        matched_detections = set(assignments.values())

        for track_index, track in enumerate(self._tracks):
            if track_index in assignments:
                detection_index = assignments[track_index]
                detection = detections[detection_index]
                track.x_filter.update(detection.centroid.x, self.measurement_noise)
                track.y_filter.update(detection.centroid.y, self.measurement_noise)
                track.size_x = 0.7 * track.size_x + 0.3 * detection.size_x
                track.size_y = 0.7 * track.size_y + 0.3 * detection.size_y
                observed_displacement = math.hypot(
                    detection.centroid.x - track.initial_x,
                    detection.centroid.y - track.initial_y,
                )
                track.last_update = stamp
                track.observations += 1
                track.consecutive_hits += 1
                track.misses = 0
                if (
                    track.consecutive_hits >= self.min_hits_to_confirm
                    and observed_displacement >= self.min_displacement_to_confirm
                ):
                    track.state = TrackState.CONFIRMED
            else:
                track.misses += 1
                track.consecutive_hits = 0
                if track.state in (TrackState.CONFIRMED, TrackState.COASTING):
                    track.state = TrackState.COASTING

        survivors: list[_Track] = []
        for track in self._tracks:
            should_delete = (
                track.misses > self.tentative_max_misses
                if track.state == TrackState.TENTATIVE
                else stamp - track.last_update > self.max_coast_time_sec
            )
            if should_delete:
                track.state = TrackState.LOST
                deleted += 1
            else:
                survivors.append(track)
        self._tracks = survivors

        created = 0
        for detection_index, detection in enumerate(detections):
            if detection_index in matched_detections:
                continue
            track = _Track(
                track_id=self._next_id,
                x_filter=_Kalman1D(detection.centroid.x, self.initial_variance),
                y_filter=_Kalman1D(detection.centroid.y, self.initial_variance),
                created_at=stamp,
                last_update=stamp,
                size_x=detection.size_x,
                size_y=detection.size_y,
                initial_x=detection.centroid.x,
                initial_y=detection.centroid.y,
            )
            if (
                self.min_hits_to_confirm <= 1
                and self.min_displacement_to_confirm <= 0.0
            ):
                track.state = TrackState.CONFIRMED
            self._tracks.append(track)
            self._next_id += 1
            created += 1

        snapshots = tuple(self._snapshot(track, stamp) for track in self._tracks)
        return TrackerUpdate(snapshots, created, deleted, time_reset)

    def _snapshot(self, track: _Track, stamp: float) -> TrackSnapshot:
        velocity_x = track.x_filter.velocity
        velocity_y = track.y_filter.velocity
        speed = math.hypot(velocity_x, velocity_y)
        if self.max_prediction_speed > 0.0 and speed > self.max_prediction_speed:
            scale = self.max_prediction_speed / speed
            velocity_x *= scale
            velocity_y *= scale
        prediction: list[Point2D] = []
        for step in range(1, self.prediction_steps + 1):
            future = step * self.prediction_dt
            effective_time = (
                self.velocity_decay_tau
                * (1.0 - math.exp(-future / self.velocity_decay_tau))
                if self.velocity_decay_tau > 0.0
                else future
            )
            prediction.append(
                Point2D(
                    track.x_filter.position + velocity_x * effective_time,
                    track.y_filter.position + velocity_y * effective_time,
                )
            )
        return TrackSnapshot(
            track_id=track.track_id,
            state=track.state,
            position=Point2D(track.x_filter.position, track.y_filter.position),
            velocity=Point2D(track.x_filter.velocity, track.y_filter.velocity),
            size_x=track.size_x,
            size_y=track.size_y,
            timestamp=stamp,
            last_observation_timestamp=track.last_update,
            age_sec=max(0.0, stamp - track.created_at),
            observations=track.observations,
            misses=track.misses,
            prediction=tuple(prediction),
        )
