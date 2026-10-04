"""Small deterministic numerical probes, explicitly NOT physical benchmarks."""
import time
import numpy as np
from .contracts import PreparedRoute, freeze_cycle
from .follow import FollowMPC
from .frontend import StaticRoute, prepare_route
from .observed_shape import ObservedShapeTracker
from .tracker_core import Point2D


def prepared_open_route(executable=None):
    grid = np.zeros((60, 100), np.uint8)
    if executable:
        frontend = prepare_route(executable, grid, .1, (-2., -3.), (0., 0.), (5., 0.), supplied_path=[(0., 0.), (5., 0.)])
    else:
        anchors = [[float(x), 0.] for x in np.arange(0., 5.1, .5)]
        document = dict(path=[anchors[0], anchors[-1]], anchors=[dict(position=p, centre_bounds=[-.8, 5.8, -1.8, 1.8]) for p in anchors])
        frontend = StaticRoute(document, grid, .1, (-2., -3.))
    return PreparedRoute.from_frontend(frontend, 'numerical-raw-static', 1)


def wait_release_probe(route):
    """Ideal measured wall, then ideal measurements as it moves away.

    There is no Gazebo sensor, complete-body oracle or STVL/MPPI comparison.
    All resulting failures are retained; weights/geometry never change here.
    """
    mpc, tracker = FollowMPC(), ObservedShapeTracker()
    state, last = (0., 0., 0., 0., 0., 0.), (0., 0., 0.)
    rows = []
    for k in range(100):
        epoch = 1_000_000_000+k*50_000_000
        # Latest measured endpoints only. Depart after t=2s at 1m/s.
        y = max(0., (k-40)*.05)
        points = [Point2D(.75, y+offset) for offset in np.arange(-.4, .401, .1)]
        public, shapes, update = tracker.update(points, epoch, k)
        acquired = time.perf_counter_ns()
        snapshot = freeze_cycle(cycle_id=k, epoch_ns=epoch, acquired_steady_ns=acquired,
                                state_ns=epoch, tf_ns=epoch, state=state, last_command=last,
                                last_command_ns=epoch, public=public, shapes=shapes, route=route)
        result = mpc.solve(snapshot)
        command = result.command
        rows.append(dict(cycle=k, epoch_ns=epoch, status=result.status, reason=result.reason,
                         state=state, command=command, progress=snapshot.progress,
                         solver_status=result.solver_status, solver_s=result.solver_s,
                         elapsed_s=result.elapsed_s, digest=snapshot.digest,
                         plateau_stages=sum(s.plateau for s in result.dynamic_samples),
                         dynamic_cost=result.solved_dynamic_cost,
                         dynamic_long_horizon_vetoes=result.dynamic_long_horizon_vetoes))
        # Toy fixed-yaw ZOH model only; this is not measured plant execution.
        state = (float(state[0]+.05*command[0]), float(state[1]+.05*command[1]), 0., *command[:2], 0.)
        last = command
    waiting = [r for r in rows[20:40] if np.linalg.norm(r['command'][:2]) < .03]
    resumed = [r for r in rows[60:] if r['command'][0] >= .1]
    return dict(kind='ideal_endpoint_numerical_probe', cycles=len(rows), wait_cycles=len(waiting),
                resumes_after_release=bool(resumed), final_x=state[0],
                solver_failures=sum(r['status'] != 'follow' for r in rows),
                max_cycle_s=max(r['elapsed_s'] for r in rows), rows=rows,
                physical_acceptance='NOT_EVALUATED', B0_comparison='NOT_RUN')
