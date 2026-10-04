import time
import numpy as np
import pytest
from r4_hws.contracts import PreparedRoute, freeze_cycle
from r4_hws.frontend import StaticRoute
from r4_hws.observed_shape import ObservedShapeTracker


@pytest.fixture
def route():
    grid = np.zeros((60, 100), np.uint8)
    anchors = [[float(x), 0.] for x in np.arange(0., 5.1, .5)]
    document = dict(path=[anchors[0], anchors[-1]],
                    anchors=[dict(position=p, centre_bounds=[-.8, 5.8, -1.8, 1.8]) for p in anchors])
    return PreparedRoute.from_frontend(StaticRoute(document, grid, .1, (-2., -3.)), 'raw-static-fixture', 1)


@pytest.fixture
def make_cycle(route):
    def make(public=None, shapes=None, *, epoch=1_000_000_000, state=(0., 0., 0., 0., 0., 0.),
             last=(0., 0., 0.), acquired=None, **extra):
        if public is None:
            public, shapes, _ = ObservedShapeTracker().update([], epoch, epoch)
        values = dict(cycle_id=epoch, epoch_ns=epoch, acquired_steady_ns=time.perf_counter_ns() if acquired is None else acquired,
                      state_ns=epoch, tf_ns=epoch, state=state, last_command=last, last_command_ns=epoch,
                      public=public, shapes=shapes, route=route)
        values.update(extra)
        return freeze_cycle(**values)
    return make
