from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Sequence

from rm_path_annotations.core import (
    GEOMETRY_EPSILON,
    PathPose,
    RegionContractError,
    RegionSet,
    RegionType,
    annotate_path,
)


class GateState(str, Enum):
    """Observable pause-gate states."""

    WAITING_FOR_PATH = "waiting_for_path"
    ARMED = "armed"
    BRAKING = "braking"
    HOLDING = "holding"
    RELEASED = "released"
    COMMITTED = "committed"
    PASSED = "passed"
    INVALID_ENTRY = "invalid_entry"


@dataclass(frozen=True)
class GateSnapshot:
    state: GateState
    path_crosses_corridor: bool
    committed_corridor_ahead: bool
    inside_approach: bool
    inside_corridor: bool
    path_progress: float | None


def _point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> bool:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    px = point[0] - start[0]
    py = point[1] - start[1]
    cross = dx * py - dy * px
    if abs(cross) > GEOMETRY_EPSILON:
        return False
    dot = px * dx + py * dy
    return -GEOMETRY_EPSILON <= dot <= dx * dx + dy * dy + GEOMETRY_EPSILON


def point_in_polygon(
    point: tuple[float, float],
    polygon: Sequence[tuple[float, float]],
) -> bool:
    """Return true for an interior or boundary point."""
    inside = False
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        if _point_on_segment(point, start, end):
            return True
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing_x = (
                start[0]
                + (point[1] - start[1])
                * (end[0] - start[0])
                / (end[1] - start[1])
            )
            if crossing_x > point[0]:
                inside = not inside
    return inside


def project_path_distance(
    poses: Sequence[PathPose],
    x: float,
    y: float,
) -> float:
    """Project a map pose onto the closest path segment and return arc length."""
    if not poses:
        raise RegionContractError("path must contain at least one pose")
    if not math.isfinite(x) or not math.isfinite(y):
        raise RegionContractError("robot pose contains non-finite coordinates")
    if len(poses) == 1:
        return 0.0

    best_squared_distance = math.inf
    best_progress = 0.0
    distance = 0.0
    for first, second in zip(poses, poses[1:]):
        dx = second.x - first.x
        dy = second.y - first.y
        squared_length = dx * dx + dy * dy
        length = math.sqrt(squared_length)
        if squared_length <= GEOMETRY_EPSILON:
            continue
        parameter = ((x - first.x) * dx + (y - first.y) * dy) / squared_length
        parameter = min(1.0, max(0.0, parameter))
        projected_x = first.x + parameter * dx
        projected_y = first.y + parameter * dy
        squared_distance = (x - projected_x) ** 2 + (y - projected_y) ** 2
        progress = distance + parameter * length
        if (
            squared_distance < best_squared_distance - GEOMETRY_EPSILON
            or (
                abs(squared_distance - best_squared_distance) <= GEOMETRY_EPSILON
                and progress > best_progress
            )
        ):
            best_squared_distance = squared_distance
            best_progress = progress
        distance += length
    return best_progress


class DogHoleEntryPauseGate:
    """Pure state machine for a stop-before-entry, then committed traversal."""

    def __init__(
        self,
        region_set: RegionSet,
        *,
        brake_settle_sec: float = 0.5,
        hold_sec: float = 5.0,
        rearm_clear_sec: float = 1.0,
        progress_margin: float = 0.05,
    ) -> None:
        for name, value in (
            ("brake_settle_sec", brake_settle_sec),
            ("hold_sec", hold_sec),
            ("rearm_clear_sec", rearm_clear_sec),
            ("progress_margin", progress_margin),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise RegionContractError(f"{name} must be finite and non-negative")
        self._approaches = tuple(
            region
            for region in region_set.regions
            if region.region_type == RegionType.DOG_HOLE_APPROACH
        )
        self._corridors = tuple(
            region
            for region in region_set.regions
            if region.region_type == RegionType.COMMITTED_CORRIDOR
        )
        if not self._approaches:
            raise RegionContractError(
                "dog-hole pause gate requires at least one dog_hole_approach region"
            )
        if not self._corridors:
            raise RegionContractError(
                "dog-hole pause gate requires at least one committed_corridor region"
            )

        self._region_set = region_set
        self._brake_settle_sec = brake_settle_sec
        self._hold_sec = hold_sec
        self._rearm_clear_sec = rearm_clear_sec
        self._progress_margin = progress_margin
        self._poses: tuple[PathPose, ...] = ()
        self._committed_spans: tuple[tuple[float, float], ...] = ()
        self._state = GateState.WAITING_FOR_PATH
        self._state_since = 0.0
        self._inside_approach = False
        self._inside_corridor = False
        self._path_progress: float | None = None
        self._committed_ahead = False

    @property
    def state(self) -> GateState:
        return self._state

    @property
    def path_ready(self) -> bool:
        return bool(self._poses)

    @property
    def path_crosses_corridor(self) -> bool:
        return bool(self._committed_spans)

    @property
    def commit_latched(self) -> bool:
        return self._state in {
            GateState.RELEASED,
            GateState.COMMITTED,
            GateState.PASSED,
        }

    def reject_path(self, now: float) -> None:
        """Invalidate path authority without releasing an active pause."""
        self._poses = ()
        self._committed_spans = ()
        self._path_progress = None
        self._committed_ahead = False
        if self._state == GateState.ARMED:
            self._transition(GateState.WAITING_FOR_PATH, now)

    def set_path(self, poses: Sequence[PathPose], now: float) -> None:
        result = annotate_path(poses, self._region_set)
        committed_spans = tuple(
            (segment.start_distance, segment.end_distance)
            for segment in result.segments
            if RegionType.COMMITTED_CORRIDOR in segment.region_types
        )
        self._poses = tuple(poses)
        self._committed_spans = committed_spans
        self._path_progress = None
        self._committed_ahead = bool(committed_spans)
        if self._state == GateState.WAITING_FOR_PATH:
            self._transition(GateState.ARMED, now)

    def update_pose(self, x: float, y: float, now: float) -> GateSnapshot:
        if not math.isfinite(now):
            raise RegionContractError("time must be finite")
        point = (x, y)
        self._inside_approach = any(
            point_in_polygon(point, region.polygon) for region in self._approaches
        )
        self._inside_corridor = any(
            point_in_polygon(point, region.polygon) for region in self._corridors
        )
        if self._poses:
            self._path_progress = project_path_distance(self._poses, x, y)
            self._committed_ahead = any(
                end > self._path_progress + self._progress_margin
                for unused_start, end in self._committed_spans
            )

        if self._state in {GateState.WAITING_FOR_PATH, GateState.ARMED}:
            if self._inside_corridor:
                self._transition(GateState.INVALID_ENTRY, now)
            elif self._inside_approach and self._committed_ahead:
                self._transition(GateState.BRAKING, now)
        elif self._state == GateState.RELEASED and self._inside_corridor:
            self._transition(GateState.COMMITTED, now)
        elif (
            self._state == GateState.COMMITTED
            and not self._inside_corridor
            and not self._inside_approach
            and not self._committed_ahead
        ):
            self._transition(GateState.PASSED, now)

        self.tick(now)
        return self.snapshot()

    def tick(self, now: float) -> GateSnapshot:
        if not math.isfinite(now):
            raise RegionContractError("time must be finite")
        if self._state == GateState.BRAKING:
            if now - self._state_since >= self._brake_settle_sec:
                self._transition(GateState.HOLDING, now)
        elif self._state == GateState.HOLDING:
            if now - self._state_since >= self._hold_sec:
                self._transition(GateState.RELEASED, now)
        elif self._state == GateState.PASSED:
            if now - self._state_since >= self._rearm_clear_sec:
                self._transition(
                    GateState.ARMED if self._poses else GateState.WAITING_FOR_PATH,
                    now,
                )
        return self.snapshot()

    def must_stop(self, *, pose_fresh: bool) -> bool:
        if self._state in {
            GateState.BRAKING,
            GateState.HOLDING,
            GateState.INVALID_ENTRY,
        }:
            return True
        if self.commit_latched:
            return False
        if not self.path_ready:
            return True
        if self.path_crosses_corridor and not pose_fresh:
            return True
        return False

    def snapshot(self) -> GateSnapshot:
        return GateSnapshot(
            state=self._state,
            path_crosses_corridor=self.path_crosses_corridor,
            committed_corridor_ahead=self._committed_ahead,
            inside_approach=self._inside_approach,
            inside_corridor=self._inside_corridor,
            path_progress=self._path_progress,
        )

    def _transition(self, state: GateState, now: float) -> None:
        self._state = state
        self._state_since = now
