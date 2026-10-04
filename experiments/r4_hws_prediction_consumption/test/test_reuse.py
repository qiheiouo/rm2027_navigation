from dataclasses import asdict
import os
from pathlib import Path
import subprocess
import sys
import types
import numpy as np
import pytest
from r4_hws import tracker_core
from r4_hws.frontend import prepare_route
from r4_hws.contracts import PreparedRoute, ContractError


def test_tracker_algorithm_matches_frozen_source_except_association_output():
    root = Path(__file__).resolve().parents[3]
    content = subprocess.run(['git', 'show', '04291a410f193c009e043af88e014cf420e1f68b:experiments/temporal_mpc/ros2/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py'],
                             cwd=root, text=True, capture_output=True, check=True).stdout
    original = types.ModuleType('r4_test_frozen_tracker')
    sys.modules[original.__name__] = original
    exec(compile(content, 'frozen_R3_tracker', 'exec'), original.__dict__)
    try:
        kwargs = dict(public_anchor_mode='last_observation_cv', velocity_decay_tau=0., max_prediction_speed=0., min_displacement_to_confirm=.15)
        frozen, r4 = original.MultiObjectTracker(**kwargs), tracker_core.MultiObjectTracker(**kwargs)
        for k in range(30):
            positions = [(1.+.04*k, .02), (3.-.03*k, -.1)] if k not in (7, 8, 19) else []
            if k % 2: positions.reverse()
            original_d = [original.Detection(original.Point2D(x, y), .2, .3, 3) for x, y in positions]
            r4_d = [tracker_core.Detection(tracker_core.Point2D(x, y), .2, .3, 3) for x, y in positions]
            epoch = 1_000_000_000+k*100_000_000
            a, b = frozen.update(original_d, epoch/1e9, source_stamp_ns=epoch), r4.update(r4_d, epoch/1e9, source_stamp_ns=epoch)
            b_values = asdict(b); b_values.pop('associations')
            assert asdict(a) == b_values
        # Backwards clock retains the original reset/lifecycle behavior too.
        a, b = frozen.update([], 1.), r4.update([], 1.)
        b_values = asdict(b); b_values.pop('associations')
        assert asdict(a) == b_values
    finally:
        del sys.modules[original.__name__]


def test_actual_frozen_TDT_bridge_and_raw_static_certificate():
    executable = os.environ.get('R4_TDT_BRIDGE')
    if not executable: pytest.skip('set R4_TDT_BRIDGE to test explicit frozen frontend executable')
    grid = np.zeros((60, 100), np.uint8)
    route = prepare_route(executable, grid, .1, (-2., -3.), (0., 0.), (5., 0.), supplied_path=[(0., 0.), (5., 0.)])
    prepared = PreparedRoute.from_frontend(route, 'raw-static-integration', 1)
    s, bounds = prepared.local((0., 0.))
    assert s == pytest.approx(0.) and bounds[0] < 0 < bounds[1]
    point, tangent = prepared.sample(np.array([0., 1., 5.]))
    assert np.allclose(point, [[0., 0.], [1., 0.], [5., 0.]])
    assert np.allclose(tangent, [[1., 0.]]*3)
    # An externally supplied path through a raw-static wall cannot bypass the
    # bridge/certificate. This checks the reused boundary, not a fake corridor.
    grid[:, 45] = 254
    with pytest.raises(ContractError):
        prepare_route(executable, grid, .1, (-2., -3.), (0., 0.), (5., 0.), supplied_path=[(0., 0.), (5., 0.)])
