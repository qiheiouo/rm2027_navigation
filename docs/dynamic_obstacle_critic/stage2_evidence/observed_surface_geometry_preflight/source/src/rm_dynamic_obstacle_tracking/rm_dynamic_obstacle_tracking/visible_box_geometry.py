"""Conditional two-visible-face rectangle reconstruction (offline prototype).

An accepted fit is NOT a physical support certificate. Rectangle/visibility,
nearest-return and range-error assumptions cannot be verified by line residuals.
No ROS, truth, tracking state, TF or controller dependencies belong here.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .core import Point2D


@dataclass(frozen=True)
class BoxFitConfig:
    min_face_points: int = 4
    min_face_span: float = 0.08
    orthogonality_tolerance: float = math.radians(10.0)
    max_line_residual: float = 0.035
    range_error: float = 0.03
    min_ray_normal: float = 0.08
    max_dimension: float = 3.0
    assume_no_return_clear: bool = False

    def __post_init__(self) -> None:
        values = (self.min_face_span, self.orthogonality_tolerance,
                  self.max_line_residual, self.range_error, self.min_ray_normal,
                  self.max_dimension)
        if (not isinstance(self.min_face_points, int) or self.min_face_points < 3
                or not all(math.isfinite(v) and v > 0 for v in values)
                or self.orthogonality_tolerance >= math.pi / 4
                or self.min_ray_normal >= 1
                or not isinstance(self.assume_no_return_clear, bool)):
            raise ValueError('invalid box fit configuration')


@dataclass(frozen=True)
class VisibleBox:
    """Geometry measurement only; never a second public track record."""
    center: Point2D
    corners: tuple[Point2D, ...]
    enclosing_radius: float
    axis_angle: float
    edge_lengths: tuple[float, float]
    edge_intervals: tuple[tuple[float, float], tuple[float, float]]
    split: int
    max_line_residual: float
    rms_line_residual: float
    boundary_evidence: tuple[str, str]
    support_is_certified: bool = False


@dataclass(frozen=True)
class BoxFitResult:
    reason: str
    box: VisibleBox | None = None


def _dot(a: Point2D, b: Point2D) -> float:
    return a.x * b.x + a.y * b.y


def _covariance(prefix: Sequence[tuple[float, ...]], start: int, end: int):
    n = end - start
    sx, sy, xx, xy, yy = (b - a for a, b in zip(prefix[start], prefix[end]))
    return (Point2D(sx / n, sy / n), max(0., xx - sx * sx / n),
            xy - sx * sy / n, max(0., yy - sy * sy / n))


def _principal(xx: float, xy: float, yy: float) -> Point2D:
    angle = .5 * math.atan2(2 * xy, xx - yy)
    return Point2D(math.cos(angle), math.sin(angle))


def fit_visible_box(
    points: Sequence[Point2D],
    beam_indices: Sequence[int],
    *,
    ranges: Sequence[float],
    sensor: Point2D,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: BoxFitConfig = BoxFitConfig(),
) -> BoxFitResult:
    """Fit a source-ordered cluster; accept only conditional two-face evidence.

    Angles are in the same world frame as points. ``beam_indices`` must refer to
    the full original scan; dropping invalid beams before indexing is invalid.
    No-return clearance is disabled by default, including at scan seams.
    """
    finite = (sensor.x, sensor.y, angle_min, angle_increment, range_min, range_max)
    if (len(points) != len(beam_indices) or not all(math.isfinite(v) for v in finite)
            or angle_increment <= 0 or range_min < 0 or range_max <= range_min
            or not math.isfinite(angle_min + max(0, len(ranges) - 1) * angle_increment)
            or any(not math.isfinite(p.x) or not math.isfinite(p.y) for p in points)
            or any(not isinstance(i, int) or isinstance(i, bool)
                   or i < 0 or i >= len(ranges) for i in beam_indices)):
        return BoxFitResult('invalid_input')
    if len(set(beam_indices)) != len(beam_indices):
        return BoxFitResult('invalid_input')
    ordered = sorted(zip(beam_indices, points))
    indices = [i for i, _ in ordered]
    if len(indices) < 2 * config.min_face_points:
        return BoxFitResult('insufficient_points')
    if any(b != a + 1 for a, b in zip(indices, indices[1:])):
        return BoxFitResult('internal_beam_gap')
    # Keep coordinates near the sensor to limit large world-offset cancellation.
    local = [Point2D(p.x - sensor.x, p.y - sensor.y) for _, p in ordered]
    if any(not math.isfinite(p.x) or not math.isfinite(p.y) for p in local):
        return BoxFitResult('invalid_input')
    for i, p in zip(indices, local):
        measured = ranges[i]
        if not math.isfinite(measured) or not range_min <= measured <= range_max:
            return BoxFitResult('invalid_member_return')
        angle = angle_min + i * angle_increment
        if math.hypot(p.x - measured * math.cos(angle),
                      p.y - measured * math.sin(angle)) > 1e-6:
            return BoxFitResult('point_ray_mismatch')
    prefix = [(0., 0., 0., 0., 0.)]
    for p in local:
        prefix.append(tuple(a + b for a, b in zip(
            prefix[-1], (p.x, p.y, p.x * p.x, p.x * p.y, p.y * p.y))))
    if any(not math.isfinite(v) for v in prefix[-1]):
        return BoxFitResult('invalid_input')
    mean, xx, xy, yy = _covariance(prefix, 0, len(local))
    single = _principal(xx, xy, yy)
    # If a single face satisfies the SAME residual allowance, a two-face
    # interpretation cannot resolve hidden depth. Do not prefer it merely
    # because two fitted lines decrease least-squares error.
    if max(abs((p.x - mean.x) * single.y - (p.y - mean.y) * single.x)
           for p in local) <= config.max_line_residual:
        return BoxFitResult('single_face_not_excluded')
    best = None
    for split in range(config.min_face_points, len(local) - config.min_face_points + 1):
        m1, xx1, xy1, yy1 = _covariance(prefix, 0, split)
        m2, xx2, xy2, yy2 = _covariance(prefix, split, len(local))
        d1, d2 = _principal(xx1, xy1, yy1), _principal(xx2, xy2, yy2)
        if abs(_dot(d1, d2)) > math.sin(config.orthogonality_tolerance):
            continue
        # Joint orthogonal TLS: maximize u'(C1-C2)u, with v perpendicular.
        u = _principal(xx1 - xx2, xy1 - xy2, yy1 - yy2)
        v = Point2D(-u.y, u.x)
        corner = Point2D(u.x * _dot(m2, u) + v.x * _dot(m1, v),
                         u.y * _dot(m2, u) + v.y * _dot(m1, v))
        if _dot(Point2D(m1.x - corner.x, m1.y - corner.y), u) < 0:
            u = Point2D(-u.x, -u.y)
        if _dot(Point2D(m2.x - corner.x, m2.y - corner.y), v) < 0:
            v = Point2D(-v.x, -v.y)
        coordinates = [(p.x - corner.x, p.y - corner.y) for p in local]
        first = [x * u.x + y * u.y for x, y in coordinates[:split]]
        second = [x * v.x + y * v.y for x, y in coordinates[split:]]
        if (min(max(first) - min(first), max(second) - min(second)) < config.min_face_span
                or min(first + second) < -config.max_line_residual
                or max(first + second) > config.max_dimension
                or _dot(corner, u) <= 0 or _dot(corner, v) <= 0):
            continue
        residuals = ([abs(x * v.x + y * v.y) for x, y in coordinates[:split]]
                     + [abs(x * u.x + y * u.y) for x, y in coordinates[split:]])
        maximum = max(residuals)
        if maximum > config.max_line_residual:
            continue
        error = sum(r * r for r in residuals)
        # The silhouette endpoints must be the far ends of their fitted faces.
        if (first[0] + config.max_line_residual < max(first)
                or second[-1] + config.max_line_residual < max(second)):
            continue
        if best is None or error < best[0]:
            best = (error, split, corner, u, v, max(first), max(second), maximum)
    if best is None:
        return BoxFitResult('two_faces_not_observable')
    error, split, corner, u, v, length1, length2, maximum = best
    intervals, evidence = [], []
    for index, axis, normal, length in ((indices[0] - 1, u, v, length1),
                                        (indices[-1] + 1, v, u, length2)):
        if index < 0 or index >= len(ranges):
            return BoxFitResult('scan_boundary')
        measured = ranges[index]
        angle = angle_min + index * angle_increment
        direction = Point2D(math.cos(angle), math.sin(angle))
        denominator = _dot(direction, normal)
        if denominator <= config.min_ray_normal:
            return BoxFitResult('ill_conditioned_boundary')
        distance = _dot(corner, normal) / denominator
        projected = _dot(Point2D(distance * direction.x - corner.x,
                                distance * direction.y - corner.y), axis)
        if projected < length - config.max_line_residual or projected > config.max_dimension:
            return BoxFitResult('inconsistent_boundary')
        if measured == math.inf and config.assume_no_return_clear:
            label = 'no_return_assumed_clear'
            if distance >= range_max - config.range_error:
                return BoxFitResult('boundary_beyond_range')
        elif math.isfinite(measured) and range_min <= measured <= range_max:
            if measured <= distance + 2 * config.range_error:
                return BoxFitResult('boundary_occluded_or_no_contrast')
            label = 'finite_return_behind_face'
        else:
            return BoxFitResult('unknown_boundary_return')
        intervals.append((length, max(length, projected)))
        evidence.append(label)
    lengths = tuple((lo + hi) / 2 for lo, hi in intervals)
    center = Point2D(sensor.x + corner.x + .5 * (u.x * lengths[0] + v.x * lengths[1]),
                     sensor.y + corner.y + .5 * (u.y * lengths[0] + v.y * lengths[1]))
    corners = tuple(Point2D(sensor.x + corner.x + u.x * a + v.x * b,
                            sensor.y + corner.y + u.y * a + v.y * b)
                    for a, b in ((0., 0.), (lengths[0], 0.), lengths,
                                 (0., lengths[1])))
    # Empirical circle uses the upper silhouette bracket and a configured radial
    # allowance. This does not bound line-angle, corner or occlusion errors.
    radius = max(math.hypot(corner.x + u.x * a + v.x * b + sensor.x - center.x,
                            corner.y + u.y * a + v.y * b + sensor.y - center.y)
                 for a, b in ((0., 0.), (intervals[0][1], 0.),
                              (intervals[0][1], intervals[1][1]), (0., intervals[1][1])))
    return BoxFitResult('conditional_two_face_fit', VisibleBox(
        center, corners, radius + config.range_error, math.atan2(u.y, u.x),
        lengths, tuple(intervals), split, maximum, math.sqrt(error / len(local)),
        tuple(evidence)))
