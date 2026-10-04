"""Bounded common execution gate for the experimental command chain.

This is neither a planner nor an optimizer. It may only pass a slew-limited
requested velocity or reduce the previous output toward zero. Public geometry
and the authored static map are its inputs; physical truth is never an input.
A model certificate is not a continuous actuator/detection safety certificate.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy.spatial import cKDTree
from .contracts import ContractError, predict
from .dynamics import braking, rollout_zoh
from .geometry import clearance


@dataclass
class GuardResult:
    command: np.ndarray
    status: str
    reason: str
    model_certified: bool
    constraint_min: float | None
    elapsed_s: float


class StaticCells:
    def __init__(self, costs, resolution, origin):
        grid = np.asarray(costs)
        if (grid.ndim != 2 or not 3 <= min(grid.shape) or max(grid.shape) > 1000
                or grid.size > 100000 or not np.issubdtype(grid.dtype, np.integer)
                or np.any((grid < 0) | (grid > 255)) or not .01 <= resolution <= .2
                or np.shape(origin) != (2,) or not np.isfinite(origin).all()):
            raise ContractError('invalid guard static grid')
        blocked = grid >= 253
        blocked[[0, -1], :] = True
        blocked[:, [0, -1]] = True
        iy, ix = np.where(blocked)
        self.left = origin[0] + ix * resolution
        self.bottom = origin[1] + iy * resolution
        self.resolution = resolution
        self.centers = np.c_[self.left+resolution/2,self.bottom+resolution/2]
        self.index = cKDTree(self.centers)
        self.bounds = (origin[0], origin[0] + grid.shape[1]*resolution,
                       origin[1], origin[1] + grid.shape[0]*resolution)

    def minimum(self, states, reserve):
        positions = np.asarray(states)[:,:2]
        _, nearest = self.index.query(positions,k=1)
        half = self.resolution/2
        candidate_distance = np.linalg.norm(np.maximum(np.abs(positions-self.centers[nearest])-half,0.),axis=1)
        # Any cell improving this upper bound has its center within d+half
        # diagonal. Query ALL such cells, then evaluate exact square distance.
        # The tree is built outside the execution cycle; no nearest-center
        # approximation or occupancy culling is used for acceptance.
        neighbors = self.index.query_ball_point(positions,candidate_distance+np.sqrt(2)*half+1e-12)
        counts=np.array([len(n) for n in neighbors])
        indices=np.concatenate(neighbors).astype(int)
        point_indices=np.repeat(np.arange(len(positions)),counts)
        delta=np.maximum(np.abs(positions[point_indices]-self.centers[indices])-half,0.)
        reach = np.hypot(.355, .330) + .02 + reserve
        b = self.bounds
        x,y=positions.T
        return float(min(np.min(np.linalg.norm(delta,axis=1))-reach,
                         np.min(x)-b[0]-reach,b[1]-np.max(x)-reach,
                         np.min(y)-b[2]-reach,b[3]-np.max(y)-reach))


class ExecutionGuard:
    period = .05
    budget = .010
    times = np.arange(31)*.05

    def __init__(self):
        self.previous = np.zeros(3)

    def step(self, initial, requested, epoch_ns, snapshot, cells, state_age_s=0.):
        started = time.perf_counter()
        lower, upper = np.array([-.5, -.5, 0.]), np.array([.8, .5, 0.])
        # Always preserve a finite bounded output, including malformed state.
        brake = np.sign(self.previous)*np.maximum(np.abs(self.previous)-[.05,.05,0.],0.)
        x, desired = np.asarray(initial, float), np.asarray(requested, float)
        valid = (x.shape == (6,) and np.isfinite(x).all() and abs(x[5]) <= 1e-8
                 and -.5 <= x[3] <= .8 and abs(x[4]) <= .5
                 and type(epoch_ns) is int and epoch_ns >= 0
                 and np.isfinite(state_age_s) and 0 <= state_age_s <= .15)
        timeline = None
        def deadline():
            if time.perf_counter()-started > self.budget:
                raise TimeoutError('guard deadline')
        if valid:
            x = x.copy(); x[5] = 0.
            try:
                timeline = predict(snapshot, epoch_ns, self.times)
            except (ContractError, AttributeError, TypeError):
                pass

        def check(command):
            if not valid or timeline is None or not isinstance(cells, StaticCells):
                return False, None, 'input degraded/stale'
            first = (command-x[3:])/.05
            if np.max(np.abs(first[:2])) > 1.+1e-6 or first[2] != 0.:
                return False, None, 'measured/target slew disagreement'
            after = x.copy(); after[3:] = command
            controls = np.vstack((first,braking(after,29,.05)))
            states = rollout_zoh(x,controls,.05)
            deadline()
            reserve = .025*np.hypot(.8,.5) + state_age_s*np.hypot(.8,.5)
            minima = [(cells.minimum(states,reserve),'static braking sweep')]
            deadline()
            for centers, shape, speed in zip(timeline.centers,timeline.geometries,timeline.speeds):
                deadline()
                slack = clearance(states,centers,shape,(.355,.330))-.02-reserve-.025*speed
                minima.append((float(np.min(slack)),'dynamic braking sweep'))
            minimum, reason = min(minima)
            return minimum >= 0. and np.max(np.abs(states[-1,3:])) <= 1e-5, minimum, reason

        candidate_valid = (desired.shape == (3,) and np.isfinite(desired).all() and desired[2] == 0.)
        if candidate_valid:
            command = np.clip(desired,np.maximum(lower,self.previous-[.05,.05,0.]),
                              np.minimum(upper,self.previous+[.05,.05,0.]))
            try: ok, minimum, reason = check(command)
            except TimeoutError: ok, minimum, reason = False,None,'guard deadline'
        else:
            ok, minimum, reason = False, None, 'invalid requested velocity'
        if ok:
            status = 'pass'
        else:
            candidate_reason = reason
            command = brake
            if candidate_reason == 'guard deadline':
                ok, minimum, reason = False,None,'guard deadline'
            else:
                try: ok, minimum, reason = check(command)
                except TimeoutError: ok, minimum, reason = False,None,'guard deadline'
            status = 'certified_brake' if ok else 'uncertified_brake'
            reason = candidate_reason + '; ' + reason
        elapsed = time.perf_counter()-started
        if elapsed > self.budget:
            command, ok, status, reason = brake, False, 'uncertified_brake', 'guard deadline'
        self.previous = command.copy()
        return GuardResult(command,status,reason,bool(ok),minimum,elapsed)
