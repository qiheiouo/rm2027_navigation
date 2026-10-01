"""Independent ray/box labels; geometry fits must not receive hidden dimensions."""
import math

import pytest

from rm_dynamic_obstacle_tracking.core import Point2D, cluster_point_indices, cluster_points
from rm_dynamic_obstacle_tracking.visible_box_geometry import BoxFitConfig, fit_visible_box


def _scan(sensor=Point2D(-3., -2.), center=Point2D(0., 0.),
          dimensions=(.4, .6), yaw=0., background=7.9, noise=0.):
    angle_min = math.atan2(center.y - sensor.y, center.x - sensor.x) - math.pi
    increment = math.radians(.5)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = sensor.x - center.x, sensor.y - center.y
    origin = (c * dx + s * dy, -s * dx + c * dy)
    ranges, points, indices = [], [], []
    for i in range(720):
        angle = angle_min + i * increment
        direction = (math.cos(angle - yaw), math.sin(angle - yaw))
        low, high = 0., math.inf
        for p, v, half in zip(origin, direction, (d / 2 for d in dimensions)):
            if abs(v) < 1e-12:
                if abs(p) > half:
                    high = -1.; break
            else:
                a, b = (-half - p) / v, (half - p) / v
                low, high = max(low, min(a, b)), min(high, max(a, b))
        hit = low if high >= low and .1 <= low <= 8. else None
        measured = background if hit is None else hit + noise * math.sin(i * 1.73)
        ranges.append(measured)
        if hit is not None:
            points.append(Point2D(sensor.x + measured * math.cos(angle),
                                  sensor.y + measured * math.sin(angle)))
            indices.append(i)
    return dict(points=points, beam_indices=indices, ranges=ranges, sensor=sensor,
                angle_min=angle_min, angle_increment=increment, range_min=.1, range_max=8.)


def test_cluster_members_keep_original_input_indices_and_legacy_aggregation():
    points = [Point2D(2., 0.), Point2D(.02, .03), Point2D(2.05, 0.),
              Point2D(0., 0.), Point2D(7., 0.)]
    components = cluster_point_indices(points, .1, 2, 1.)
    assert [set(c) for c in components] == [{0, 2}, {1, 3}]
    detections = cluster_points(points, .1, 2, 1.)
    assert [(d.centroid, d.point_count) for d in detections] == [
        (Point2D(2.025, 0.), 2), (Point2D(.01, .015), 2)]


def test_single_face_cannot_identify_hidden_depth_even_from_exact_scan():
    a = _scan(sensor=Point2D(-3., 0.), dimensions=(.4, .6))
    b = _scan(sensor=Point2D(-3., 0.), center=Point2D(.2, 0.), dimensions=(.8, .6))
    # Use the SAME beam grid for the hidden-depth witness.
    assert a['angle_min'] == b['angle_min']
    assert a['ranges'] == b['ranges']
    assert fit_visible_box(**a).reason == 'two_faces_not_observable'
    assert fit_visible_box(**b).box is None


@pytest.mark.parametrize('noise', [0., .005, .01])
def test_two_face_center_is_recovered_without_truth_in_fit(noise):
    result = fit_visible_box(**_scan(noise=noise))
    assert result.box is not None, result.reason
    box = result.box
    assert math.hypot(box.center.x, box.center.y) < .04
    assert box.enclosing_radius > math.hypot(.2, .3)
    assert not box.support_is_certified
    assert box.boundary_evidence == ('finite_return_behind_face',) * 2


def test_fit_is_invariant_to_source_order_and_rigid_world_transform():
    original = _scan()
    first = fit_visible_box(**original).box
    assert first is not None
    phi, shift = .91, Point2D(123., -21.)
    c, s = math.cos(phi), math.sin(phi)

    def transform(p):
        return Point2D(shift.x + c * p.x - s * p.y, shift.y + s * p.x + c * p.y)

    original['sensor'] = transform(original['sensor'])
    original['points'] = [transform(p) for p in reversed(original['points'])]
    original['beam_indices'].reverse()
    original['angle_min'] += phi
    second = fit_visible_box(**original).box
    assert second is not None
    expected = transform(first.center)
    assert math.hypot(second.center.x - expected.x, second.center.y - expected.y) < 1e-9
    assert math.isclose(first.enclosing_radius, second.enclosing_radius, abs_tol=1e-9)


def test_internal_missing_ray_is_not_silently_treated_as_a_surface():
    data = _scan()
    middle = len(data['points']) // 2
    data['ranges'][data['beam_indices'][middle]] = math.nan
    data['points'].pop(middle); data['beam_indices'].pop(middle)
    assert fit_visible_box(**data).reason == 'internal_beam_gap'


@pytest.mark.parametrize('missing', [math.inf, -math.inf, math.nan, 0.])
def test_unknown_outer_boundary_refuses_full_center(missing):
    data = _scan()
    data['ranges'][data['beam_indices'][0] - 1] = missing
    assert fit_visible_box(**data).reason == 'unknown_boundary_return'


def test_no_return_clearance_requires_explicit_model_assumption():
    data = _scan(background=math.inf)
    assert fit_visible_box(**data).box is None
    box = fit_visible_box(**data, config=BoxFitConfig(assume_no_return_clear=True)).box
    assert box is not None
    assert box.boundary_evidence == ('no_return_assumed_clear',) * 2
    assert not box.support_is_certified


def test_foreground_occlusion_at_boundary_is_rejected():
    data = _scan()
    data['ranges'][data['beam_indices'][-1] + 1] = .5
    assert fit_visible_box(**data).reason == 'boundary_occluded_or_no_contrast'


def test_points_must_use_the_original_source_ray_transform():
    data = _scan()
    data['angle_min'] += .02
    assert fit_visible_box(**data).reason == 'point_ray_mismatch'
    data['angle_min'] -= .02
    data['beam_indices'][0] = data['beam_indices'][1]
    assert fit_visible_box(**data).reason == 'invalid_input'


@pytest.mark.parametrize('settings', [{'min_face_points': 2}, {'range_error': math.nan},
                                   {'min_ray_normal': 1.}, {'assume_no_return_clear': 'true'}])
def test_invalid_geometry_configuration_is_rejected(settings):
    with pytest.raises(ValueError):
        BoxFitConfig(**settings)
