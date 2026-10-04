import time
import numpy as np
import pytest
from r4_hws.follow import FollowMPC
from r4_hws.soft_field import TemporalSoftField
from r4_hws.observed_shape import ObservedShapeTracker
from r4_hws.tracker_core import Point2D


def test_configuration_space_gradient_and_plateau(make_cycle):
    public, shapes, _ = ObservedShapeTracker().update([Point2D(1., -.05), Point2D(1., 0.), Point2D(1., .05)], 1_000_000_000, 1)
    snap = make_cycle(public, shapes)
    field = TemporalSoftField(snap)
    p = np.array([.4, .02])
    sample = field.sample(p, 0)
    eps = 1e-6
    numerical = np.array([(field.sample(p+np.eye(2)[a]*eps, 0).residual-field.sample(p-np.eye(2)[a]*eps, 0).residual)/(2*eps) for a in range(2)])
    assert sample.clearance > 0 and np.allclose(sample.gradient, numerical, atol=1e-4)
    assert sample.gradient[0] > 0.
    plateau = field.sample((1., 0.), 0)
    assert plateau.residual == 8. and plateau.gradient == (0., 0.) and plateau.plateau
    assert field.sample((-1., 0.), 0).residual == 0.
    assert field.physical_half_extents == (.325, .300) and field.robot_padding == .03


def test_zoh_progress_static_bounds_without_terminal_stop(make_cycle):
    snap = make_cycle()
    result = FollowMPC().solve(snap)
    assert result.status == 'follow', result.reason
    states, controls = np.asarray(result.states), np.asarray(result.controls)
    expected = np.cumsum(np.repeat(controls[:, :2], 2, axis=0)*.05, axis=0)
    assert np.allclose(states[1:, :2], expected, atol=1e-12)
    assert states[-1, 3] > .1              # not the R3 terminal-zero requirement
    assert states[-1, 6] > .1 and result.hard_min >= -1e-5
    assert result.command[0] <= .05
    assert result.dynamic_long_horizon_vetoes == 0
    assert result.snapshot_digest == snap.digest


def test_late_snapshot_always_returns_bounded_stop(make_cycle):
    snap = make_cycle(state=(0., 0., 0., .4, 0., 0.), last=(.4, 0., 0.), acquired=time.perf_counter_ns()-50_000_000)
    result = FollowMPC().solve(snap)
    assert result.status == 'stop' and result.reason == 'cycle_deadline_before_assembly'
    assert result.command == pytest.approx((.35, 0., 0.))


def test_dynamic_future_plateau_is_diagnostic_not_entire_proposal_veto(make_cycle):
    public, shapes, _ = ObservedShapeTracker().update([Point2D(.9, -.05), Point2D(.9, 0.), Point2D(.9, .05)], 1_000_000_000, 1)
    result = FollowMPC().solve(make_cycle(public, shapes))
    assert result.status == 'follow', result.reason
    assert any(sample.plateau for sample in result.dynamic_samples)
    assert result.dynamic_long_horizon_vetoes == 0


def test_warm_shift_reuses_targets_not_old_trajectory_verdict(make_cycle):
    mpc = FollowMPC()
    first = mpc.solve(make_cycle())
    assert first.status == 'follow'
    second = mpc.solve(make_cycle(epoch=1_050_000_000, state=(.0025, 0., 0., .05, 0., 0.), last=first.command))
    assert second.status == 'follow'
    assert second.states[0][0] == .0025
    assert abs(second.command[0]-first.command[0]) <= .05+1e-9


def test_solver_exception_returns_stop_and_discards_warm(monkeypatch, make_cycle):
    import r4_hws.follow as module
    class BrokenSolver:
        def setup(self, **kwargs): raise RuntimeError('injected solver failure')
    monkeypatch.setattr(module.osqp, 'OSQP', BrokenSolver)
    mpc = FollowMPC()
    snap = make_cycle(state=(0., 0., 0., .2, 0., 0.), last=(.2, 0., 0.))
    result = mpc.solve(snap)
    assert result.status == 'stop' and result.reason == 'solver_exception'
    assert result.command == pytest.approx((.15, 0., 0.))
    assert mpc.previous is None


def test_solver_overrun_cannot_publish_late_solution(monkeypatch, make_cycle):
    import r4_hws.follow as module
    real_solver = module.osqp.OSQP
    class SlowSolver:
        def __init__(self): self.solver = real_solver()
        def setup(self, **kwargs): self.solver.setup(**kwargs)
        def warm_start(self, **kwargs): self.solver.warm_start(**kwargs)
        def solve(self, **kwargs):
            time.sleep(.045)
            return self.solver.solve(**kwargs)
    monkeypatch.setattr(module.osqp, 'OSQP', SlowSolver)
    result = FollowMPC().solve(make_cycle())
    assert result.status == 'stop' and result.reason == 'cycle_or_solver_deadline'


def test_static_infeasibility_not_dynamic_failure(make_cycle):
    # Initial centre passes the preparation gate, but a moving command cannot
    # decelerate within a nearby certified static boundary in this horizon.
    result = FollowMPC().solve(make_cycle(state=(5.76, 0., 0., .8, 0., 0.), last=(.8, 0., 0.)))
    assert result.status == 'stop'
    assert result.reason in ('solver_not_solved', 'hard_static_or_bounds')
    assert result.command[0] == pytest.approx(.75)
    assert result.dynamic_long_horizon_vetoes == 0
