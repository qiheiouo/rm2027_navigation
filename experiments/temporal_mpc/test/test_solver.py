from types import SimpleNamespace as NS
import numpy as np
import pytest
from temporal_mpc.contracts import ContractError, Geometry, Snapshot, Track
from temporal_mpc.fixtures import SCENARIOS, reference
from temporal_mpc.solver import Config, TemporalMPC


def problem(cfg, initial=None):
    initial = np.zeros(6) if initial is None else np.asarray(initial, float)
    times = np.arange(cfg.steps * cfg.substeps + 1) * cfg.collision_dt
    return initial, reference(initial, (3., 0., 0.), times)


def test_free_motion_has_rate_limited_command_and_stopped_terminal():
    cfg = Config(deadline_s=3.)
    initial, path = problem(cfg)
    result = TemporalMPC(cfg).solve(initial, 0, 0, Snapshot(0, ()), path)
    assert result.model_feasible
    assert 0 < result.command[0] <= .1 + cfg.tolerance
    np.testing.assert_allclose(result.states[-1, 3:], 0, atol=cfg.tolerance)
    assert np.all(np.max(np.abs(result.controls), axis=0) <= np.asarray(cfg.acceleration) + cfg.tolerance)
    assert np.max(result.states[:, 3]) <= .8 + cfg.tolerance


def test_invalid_source_cannot_reuse_a_successful_plan_or_instantly_stop():
    cfg = Config(deadline_s=3.)
    mpc = TemporalMPC(cfg)
    initial, path = problem(cfg)
    assert mpc.solve(initial, 0, 0, Snapshot(0, ()), path).model_feasible
    initial, path = problem(cfg, [0, 0, 0, .6, 0, 0])
    result = mpc.solve(initial, 1_000_000_000, 1_000_000_000, Snapshot(0, ()), path)
    assert result.status == "invalid_input_brake" and not result.model_feasible and mpc.warm is None
    assert result.command[0] == pytest.approx(.5)


def test_start_inside_obstacle_does_not_return_a_safety_certificate():
    cfg = Config(deadline_s=3., iterations=3)
    initial, path = problem(cfg)
    shape = Geometry("circle", radius=.5, source="fixture")
    snapshot = Snapshot(0, (Track(1, (0, 0), (0, 0), 0, "confirmed", shape),))
    result = TemporalMPC(cfg).solve(initial, 0, 0, snapshot, path)
    assert result.status == "unsafe_brake" and not result.model_feasible and result.constraint_min < 0


def test_deadline_rejects_output_and_preserves_measured_momentum():
    cfg = Config(deadline_s=1e-9)
    initial, path = problem(cfg, [0, 0, 0, .6, 0, 0])
    result = TemporalMPC(cfg).solve(initial, 0, 0, Snapshot(0, ()), path)
    assert result.reason == "deadline" and result.deadline_miss
    assert result.command[0] == pytest.approx(.5)


def test_solver_success_with_invalid_controls_is_rejected(monkeypatch):
    def dishonest(*args, **kwargs):
        return NS(success=True, x=np.full(15, 20.))
    monkeypatch.setattr("temporal_mpc.solver.minimize", dishonest)
    cfg = Config(deadline_s=3.)
    initial, path = problem(cfg)
    result = TemporalMPC(cfg).solve(initial, 0, 0, Snapshot(0, ()), path)
    assert result.status == "brake" and result.command.tolist() == [0, 0, 0]


def test_nan_solver_output_is_rejected(monkeypatch):
    monkeypatch.setattr("temporal_mpc.solver.minimize", lambda *a, **k: NS(success=True, x=np.full(15, float("nan"))))
    cfg = Config(deadline_s=3.)
    initial, path = problem(cfg)
    result = TemporalMPC(cfg).solve(initial, 0, 0, Snapshot(0, ()), path)
    assert result.status == "brake" and np.isfinite(result.command).all()


@pytest.mark.parametrize("kwargs", [{"physical_half_extents": (.3, .2)}, {"padding": 0.},
    {"margin": 0.}, {"horizon": 2.1}, {"dt": .13}, {"collision_dt": .2},
    {"acceleration": (1., 0., 2.)}, {"deadline_s": float("nan")}])
def test_configuration_rejects_shrunk_geometry_or_bad_time_grid(kwargs):
    with pytest.raises(ValueError):
        Config(**kwargs)


def test_robot_and_prediction_must_share_evaluation_epoch_and_world_frame():
    cfg = Config()
    initial, path = problem(cfg)
    for state_epoch, frame in ((1, "map"), (0, "odom")):
        with pytest.raises(ContractError):
            TemporalMPC(cfg).solve(initial, state_epoch, 0, Snapshot(0, ()), path, frame=frame)


def test_static_whole_footprint_and_environment_boundary_are_hard_constraints():
    cfg = Config(deadline_s=3., iterations=2)
    initial, path = problem(cfg)
    # Cell-sized obstacle at the footprint corner; the robot center is free.
    shape = Geometry("polygon", ((-.02, -.02), (.02, -.02), (.02, .02), (-.02, .02)), source="occupied_cell_fixture")
    result = TemporalMPC(cfg).solve(initial, 0, 0, Snapshot(0, ()), path, static=(((.34, .32), shape),))
    assert not result.model_feasible and result.status == "unsafe_brake"
    result = TemporalMPC(cfg).solve(initial, 0, 0, Snapshot(0, ()), path, domain=(-.3, 7, -1.2, 1.2))
    assert not result.model_feasible


def test_live_tracker_can_feed_existing_public_contract_without_private_fields():
    # Exercise the current repository tracker through its normal snapshots;
    # converting producer snapshots to wire-shaped objects is the test harness.
    import sys
    from pathlib import Path
    import importlib.util
    path = Path(__file__).resolve().parents[1] / "reference_inputs/core.py"
    spec = importlib.util.spec_from_file_location("frozen_project_tracker", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    MultiObjectTracker, Detection, Point2D = module.MultiObjectTracker, module.Detection, module.Point2D
    from test_contracts import message, stamp
    from temporal_mpc.contracts import PublicAdapter, V1
    tracker = MultiObjectTracker(min_hits_to_confirm=3, min_displacement_to_confirm=.15,
                                 prediction_steps=15, prediction_dt=.1)
    for k in range(8):
        update = tracker.update([Detection(Point2D(2 + .05 * k, 0), .45, .55, 8)], 1 + .1 * k)
    track = update.tracks[0]
    assert track.state.value == "confirmed"
    msg = message()
    msg.schema = V1
    msg.header.stamp = stamp(1, 700_000_000)
    msg.tracks[0].last_observation_stamp = msg.header.stamp
    msg.tracks[0].position.x = track.position.x
    msg.tracks[0].velocity.x = track.velocity.x
    result = PublicAdapter(schema=V1).consume(msg, 1_700_000_000)
    assert result.tracks[0].velocity[0] > 0


def test_experiment_streams_valid_json_evidence(tmp_path):
    import json
    from dataclasses import replace
    from run_experiment import simulate
    report = simulate(replace(SCENARIOS[1], duration=.1), "observed_polygon", "temporal", Config(), tmp_path)
    json.dumps(report, allow_nan=False)
    lines = next(tmp_path.glob("*.jsonl")).read_text().splitlines()
    assert len(lines) == 1
    assert type(json.loads(lines[0])["model_feasible"]) is bool


def test_small_velocity_brake_matches_a_full_held_command_without_reversal():
    cfg = Config(deadline_s=1e-9)
    initial, path = problem(cfg, [0, 0, 0, .02, -.03, .01])
    result = TemporalMPC(cfg).solve(initial, 0, 0, Snapshot(0, ()), path)
    np.testing.assert_allclose(result.acceleration, [-.2, .3, -.1], atol=1e-12)
    np.testing.assert_allclose(result.command, [0, 0, 0], atol=1e-12)
    np.testing.assert_allclose(initial[3:] + cfg.dt * result.acceleration, result.command, atol=1e-12)
    np.testing.assert_allclose(result.controls[:cfg.substeps], np.tile(result.acceleration, (cfg.substeps, 1)))
    assert result.states[cfg.substeps, 3] >= -1e-12
    assert result.states[cfg.substeps, 4] <= 1e-12
