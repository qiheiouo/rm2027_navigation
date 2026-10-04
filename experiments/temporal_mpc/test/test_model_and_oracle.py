import math
import numpy as np
import pytest
from temporal_mpc.contracts import Geometry
from temporal_mpc.dynamics import braking, rollout
from temporal_mpc.geometry import clearance
from temporal_mpc.oracle import polygon_distance, rectangle, step_and_audit


def test_omni_body_frame_lateral_and_rotated_forward_motion():
    states = rollout([0, 0, math.pi / 2, 1, .5, 0], np.zeros((10, 3)), .1)
    np.testing.assert_allclose(states[-1, :2], [-.5, 1.], atol=1e-12)
    states = rollout([0, 0, 0, 0, 0, 0], np.tile([0, 1, 0], (10, 1)), .1)
    np.testing.assert_allclose(states[-1, [0, 1, 4]], [0, .5, 1], atol=1e-12)


def test_acceleration_yaw_and_full_brake_tail():
    states = rollout(np.zeros(6), np.tile([0, 0, 2], (10, 1)), .1)
    assert states[-1, 2] == pytest.approx(1.)
    initial = [0, 0, 0, .8, -.5, 1.2]
    controls = braking(initial, 20, .05)
    assert np.max(np.abs(controls[:, 0])) <= 1
    assert np.max(np.abs(controls[:, 1])) <= 1
    assert np.max(np.abs(controls[:, 2])) <= 2
    states = rollout(initial, controls, .05)
    np.testing.assert_allclose(states[-1, 3:], 0, atol=1e-12)
    assert states[1, 3] == pytest.approx(.75)  # No instant physical stop.


def test_independent_oracle_tangency_containment_and_distance():
    a = rectangle(0, 0, 0, .325, .3)
    assert polygon_distance(a, rectangle(.55, 0, 0, .225, .275)) == pytest.approx(0.)
    assert polygon_distance(a, rectangle(0, 0, .3, 1., 1.)) == 0
    assert polygon_distance(a, rectangle(1., 0, 0, .225, .275)) == pytest.approx(.45)


def test_model_polygon_separation_never_exceeds_independent_exact_distance():
    rng = np.random.default_rng(7307)
    shape = Geometry("polygon", ((-.225, -.275), (.225, -.275), (.225, .275), (-.225, .275)), source="fixture")
    for _ in range(250):
        state = np.r_[rng.uniform(-1, 1, 2), rng.uniform(-math.pi, math.pi), 0, 0, 0]
        center = rng.uniform(-1, 1, 2)
        lower = clearance(state[None, :], center[None, :], shape, (.355, .33))[0]
        exact = polygon_distance(rectangle(*state[:3], .355, .33), rectangle(*center, 0, .225, .275))
        assert lower <= exact + 1e-12
        if lower > 0:
            assert exact > 0


def test_independent_dense_sweep_detects_crossing_between_control_ticks():
    truth = lambda t: ((0., -.8 + 16 * t), (.025, .025))
    _, lower, sampled, contact = step_and_audit([0, 0, 0, 0, 0, 0], [0, 0, 0], 0., .1,
                                                truth, (-2., 2., -2., 2.), 16., audit_dt=.001)
    assert contact and sampled == 0 and lower < 0
