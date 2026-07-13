import math

from rm_pursuit.pursuit_math import PlanarPoint, compute_standoff_goal


def test_stops_before_stationary_target_and_faces_it():
    goal = compute_standoff_goal(
        PlanarPoint(0.0, 0.0),
        PlanarPoint(5.0, 0.0),
        PlanarPoint(0.0, 0.0),
        1.5,
        0.2,
        8.0,
    )
    assert goal is not None
    assert math.isclose(goal.x, 3.5)
    assert math.isclose(goal.y, 0.0)
    assert math.isclose(goal.yaw, 0.0)


def test_predicts_moving_target():
    goal = compute_standoff_goal(
        PlanarPoint(0.0, 0.0),
        PlanarPoint(3.0, 0.0),
        PlanarPoint(0.0, 1.0),
        1.0,
        1.0,
        8.0,
    )
    assert goal is not None
    assert goal.y > 0.0


def test_rejects_out_of_range_or_non_finite_target():
    assert compute_standoff_goal(
        PlanarPoint(0.0, 0.0),
        PlanarPoint(10.0, 0.0),
        PlanarPoint(0.0, 0.0),
        1.0,
        0.0,
        8.0,
    ) is None
    assert compute_standoff_goal(
        PlanarPoint(0.0, 0.0),
        PlanarPoint(float("nan"), 0.0),
        PlanarPoint(0.0, 0.0),
        1.0,
        0.0,
        8.0,
    ) is None
