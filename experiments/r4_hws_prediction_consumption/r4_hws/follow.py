"""One bounded OSQP local approximation of the R4 kinematic Follow objective.

Fixed yaw, body vx/vy command targets, free path progress. Dynamic predictions
appear only as stage residuals. The terminal has static/bounds constraints but
no dynamic/Follow cost and no required zero command. No candidate portfolio.
"""
from dataclasses import dataclass
import math
import time
import numpy as np
from scipy import sparse
import osqp
from .contracts import ContractError, CycleSnapshot
from .soft_field import TemporalSoftField


@dataclass(frozen=True)
class FollowConfig:
    period: float = .05
    decision_dt: float = .1
    nodes: int = 15
    cycle_budget: float = .04
    solver_budget: float = .015
    max_iterations: int = 400
    velocity_lower: tuple = (-.5, -.5)
    velocity_upper: tuple = (.8, .5)
    command_rate: tuple = (1., 1.)
    cruise: float = .6
    progress_upper: float = .8
    contour_weight: float = 20.
    lag_weight: float = 8.
    projection_weight: float = 2.
    speed_weight: float = 2.
    rate_weight: float = .10
    jerk_weight: float = .02
    progress_reward: float = .10

    def __post_init__(self):
        # The first slice is a fixed preregistered formulation, not a tuning API.
        expected = (.05, .1, 15, .04, .015, 400, (-.5, -.5), (.8, .5), (1., 1.),
                    .6, .8, 20., 8., 2., 2., .10, .02, .10)
        if tuple(self.__dict__.values()) != expected or type(self.nodes) is not int or type(self.max_iterations) is not int:
            raise ContractError("R4 first-slice parameters are frozen")


@dataclass(frozen=True)
class SolveResult:
    snapshot_digest: str
    status: str
    reason: str
    command: tuple
    controls: tuple
    states: tuple
    solver_status: str
    iterations: int
    solver_s: float
    elapsed_s: float
    hard_min: float | None
    dynamic_samples: tuple
    nominal_dynamic_cost: float
    solved_dynamic_cost: float
    dynamic_long_horizon_vetoes: int = 0


def bounded_brake(last_command, period=.05):
    """Command-rate-limited output, with no physical braking certificate."""
    return (float(np.clip(last_command[0], -.5, .8)-np.clip(last_command[0], -period, period)),
            float(np.clip(last_command[1], -.5, .5)-np.clip(last_command[1], -period, period)), 0.)


class FollowMPC:
    def __init__(self, config=FollowConfig()):
        if osqp.__version__ != '1.0.5':
            raise ContractError("only the registered OSQP 1.0.5 is accepted")
        self.config = config
        self.previous = self.previous_epoch = self.previous_plan = None
        self.times = np.arange(31)*config.period
        self.integration = np.clip(self.times[:, None]-np.arange(15)[None, :]*.1, 0., .1)
        self.command_index = np.minimum(np.arange(31)//2, 14)

    def reset(self):
        self.previous = self.previous_epoch = self.previous_plan = None

    def _maps(self, snapshot):
        yaw = snapshot.state[2]
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        xy = np.zeros((31, 2, 45))
        xy[:, 0, 0::3] = self.integration*rotation[0, 0]
        xy[:, 0, 1::3] = self.integration*rotation[0, 1]
        xy[:, 1, 0::3] = self.integration*rotation[1, 0]
        xy[:, 1, 1::3] = self.integration*rotation[1, 1]
        progress = np.zeros((31, 45))
        progress[:, 2::3] = self.integration
        return xy, progress, rotation

    def _seed(self, snapshot):
        cfg = self.config
        if (self.previous is not None and self.previous_plan == (snapshot.route.plan_id, snapshot.route.map_revision, snapshot.route.generation)
                and self.previous_epoch is not None):
            age = (snapshot.epoch_ns-self.previous_epoch)*1e-9
            if 0 < age <= .15:
                # Piecewise constant target overlap; retain terminal target.
                left = np.arange(15)*.1
                right = left+.1
                overlap = np.maximum(0., np.minimum(right[:, None]+age, right[None, :])
                                     - np.maximum(left[:, None]+age, left[None, :]))
                seed = overlap @ np.asarray(self.previous)/.1
                seed += np.maximum(0., right+age-1.5)[:, None].clip(0., .1)/.1*np.asarray(self.previous[-1])
                return seed.ravel()
        z = np.zeros((15, 3))
        velocity = np.asarray(snapshot.last_command[:2])
        s = snapshot.progress
        yaw = snapshot.state[2]
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        for k in range(15):
            _, tangent = snapshot.route.sample(np.array(s))
            cruise = min(cfg.cruise, math.sqrt(2*max(0., snapshot.route.arcs[-1]-s)))
            target = rotation.T @ tangent*cruise
            limit = cfg.period if k == 0 else cfg.decision_dt
            velocity = np.clip(velocity+np.clip(target-velocity, -limit, limit), cfg.velocity_lower, cfg.velocity_upper)
            speed = min(cfg.progress_upper, max(0., float(tangent @ rotation @ velocity)))
            speed = min(speed, max(0., (snapshot.route.arcs[-1]-s)/cfg.decision_dt))
            z[k] = (*velocity, speed)
            s += cfg.decision_dt*speed
        return z.ravel()

    def _rollout(self, snapshot, z, xy_map, s_map):
        controls = z.reshape(15, 3)
        xy = np.asarray(snapshot.state[:2]) + np.einsum('kij,j->ki', xy_map, z)
        progress = snapshot.progress+s_map@z
        velocities = controls[self.command_index, :2].copy()
        velocities[0] = snapshot.state[3:5]  # measured state at t=0; subsequent ZOH targets
        return np.c_[xy, np.full(31, snapshot.state[2]), velocities, np.zeros(31), progress]

    def solve(self, snapshot):
        if not isinstance(snapshot, CycleSnapshot):
            raise ContractError("fixed cycle snapshot required")
        cfg = self.config
        elapsed = lambda: (time.perf_counter_ns()-snapshot.acquired_steady_ns)*1e-9
        brake = bounded_brake(snapshot.last_command)
        solver_s, status, iterations = 0., 'not_run', 0
        minimum = None
        field = TemporalSoftField(snapshot)
        xy_map, s_map, rotation = self._maps(snapshot)
        zbar = self._seed(snapshot)
        nominal = self._rollout(snapshot, zbar, xy_map, s_map)
        nominal_samples = field.rollout_samples(nominal[:, :2])
        nominal_cost = .5*cfg.period*sum(s.residual**2 for s in nominal_samples[:-1])

        def failure(reason):
            self.reset()
            return SolveResult(snapshot.digest, 'stop', reason, brake, (), (), status,
                               iterations, solver_s, elapsed(), minimum, nominal_samples,
                               nominal_cost, nominal_cost)

        if elapsed() < 0 or elapsed() >= cfg.cycle_budget:
            return failure('cycle_deadline_before_assembly')
        rows, constants = [], []
        def residual(jacobian, at_nominal, weight):
            scale = math.sqrt(weight)
            rows.append(scale*jacobian)
            constants.append(scale*(at_nominal-jacobian@zbar))

        reference, tangent = snapshot.route.sample(nominal[:, 6])
        for k in range(30):                    # Follow terminal cost = 0
            dt = cfg.period
            normal = np.array([-tangent[k, 1], tangent[k, 0]])
            delta = nominal[k, :2]-reference[k]
            residual(normal@xy_map[k], normal@delta, dt*cfg.contour_weight)
            residual(tangent[k]@xy_map[k]-s_map[k], tangent[k]@delta, dt*cfg.lag_weight)
            idx = self.command_index[k]
            command_row = np.zeros(45)
            command_row[3*idx:3*idx+2] = tangent[k]@rotation
            progress_row = np.zeros(45)
            progress_row[3*idx+2] = 1.
            residual(command_row-progress_row, (command_row-progress_row)@zbar, dt*cfg.projection_weight)
            cruise = min(cfg.cruise, math.sqrt(2*max(0., snapshot.route.arcs[-1]-nominal[k, 6])))
            residual(progress_row, progress_row@zbar-cruise, dt*cfg.speed_weight)
            sample = nominal_samples[k]
            residual(np.asarray(sample.gradient)@xy_map[k], sample.residual, dt)
        difference = np.zeros((30, 45))
        previous = np.zeros(30)
        for k in range(15):
            for axis in range(2):
                row = 2*k+axis
                difference[row, 3*k+axis] = 1.
                if k:
                    difference[row, 3*(k-1)+axis] = -1.
                else:
                    previous[row] = snapshot.last_command[axis]
                interval = cfg.period if k == 0 else cfg.decision_dt
                residual(difference[row]/interval, (difference[row]@zbar-previous[row])/interval,
                         cfg.decision_dt*cfg.rate_weight)
                if k:
                    jerk = difference[row]/interval-difference[row-2]/(cfg.period if k == 1 else cfg.decision_dt)
                    residual(jerk, jerk@zbar+previous[row-2]/(cfg.period if k == 1 else cfg.decision_dt),
                             cfg.jerk_weight)
        J, c = np.asarray(rows), np.asarray(constants)
        P, q = J.T@J + np.eye(45)*1e-8, J.T@c
        q[2::3] -= cfg.progress_reward*cfg.decision_dt
        constraints = [np.eye(45), difference]
        lower = [np.tile((*cfg.velocity_lower, 0.), 15)]
        upper = [np.tile((*cfg.velocity_upper, cfg.progress_upper), 15)]
        rate_limit = np.tile(np.asarray(cfg.command_rate)*cfg.decision_dt, 15)
        rate_limit[:2] = np.asarray(cfg.command_rate)*cfg.period
        lower.append(previous-rate_limit)
        upper.append(previous+rate_limit)
        reserve = .5*cfg.period*math.hypot(.8, .5)
        b = snapshot.centre_bounds
        for axis in range(2):
            constraints.append(xy_map[:, axis])
            lower.append(np.full(31, b[2*axis]+reserve-snapshot.state[axis]))
            upper.append(np.full(31, b[2*axis+1]-reserve-snapshot.state[axis]))
        constraints.append(s_map)
        lower.append(np.full(31, -snapshot.progress))
        upper.append(np.full(31, snapshot.route.arcs[-1]-snapshot.progress))
        A, lo, hi = np.vstack(constraints), np.concatenate(lower), np.concatenate(upper)
        # Exact variable change: rates instead of absolute velocity targets.
        # This preserves objective/feasible set but avoids 1/dt rate Jacobians
        # dominating OSQP's numerical scale. s_dot remains a direct variable.
        transform = np.zeros((45, 45))
        offset = np.tile((*snapshot.last_command[:2], 0.), 15)
        intervals = np.r_[cfg.period, np.full(14, cfg.decision_dt)]
        for k in range(15):
            transform[3*k+2, 3*k+2] = 1.
            for j in range(k+1):
                transform[3*k, 3*j] = transform[3*k+1, 3*j+1] = intervals[j]
        rate_P = transform.T@P@transform
        rate_q = transform.T@(P@offset+q)
        rate_A = A@transform
        rate_lo, rate_hi = lo-A@offset, hi-A@offset
        targets = zbar.reshape(15, 3)
        warm = targets.copy()
        warm[:, :2] = np.diff(np.vstack([snapshot.last_command[:2], targets[:, :2]]), axis=0)/intervals[:, None]
        remaining = cfg.cycle_budget-elapsed()
        if remaining <= 0:
            return failure('cycle_deadline_after_assembly')
        solver_start = time.perf_counter_ns()
        try:
            solver = osqp.OSQP()
            solver.setup(P=sparse.triu(sparse.csc_matrix(rate_P), format='csc'), q=rate_q,
                         A=sparse.csc_matrix(rate_A), l=rate_lo, u=rate_hi, verbose=False,
                         eps_abs=1e-6, eps_rel=1e-6, max_iter=cfg.max_iterations,
                         time_limit=min(cfg.solver_budget, remaining), polishing=True, check_termination=10)
            solver.warm_start(x=warm.ravel())
            solution = solver.solve(raise_error=False)
            status, iterations = solution.info.status, solution.info.iter
            solver_s = (time.perf_counter_ns()-solver_start)*1e-9
        except (ValueError, RuntimeError, osqp.OSQPException) as error:
            status = type(error).__name__
            solver_s = (time.perf_counter_ns()-solver_start)*1e-9
            return failure('solver_exception')
        if elapsed() >= cfg.cycle_budget or solver_s > cfg.solver_budget:
            return failure('cycle_or_solver_deadline')
        if status != 'solved' or solution.x is None or not np.isfinite(solution.x).all():
            return failure('solver_not_solved')
        z = offset+transform@solution.x
        # Tiny numerical targets are projected to exact bounds/rate limits.
        # The independent complete static/bounds check follows the projection.
        controls = z.reshape(15, 3)
        last = np.asarray(snapshot.last_command[:2])
        for k in range(15):
            interval = cfg.period if k == 0 else cfg.decision_dt
            last = np.clip(controls[k, :2], np.maximum(cfg.velocity_lower, last-interval),
                           np.minimum(cfg.velocity_upper, last+interval))
            controls[k, :2] = last
            controls[k, 2] = np.clip(controls[k, 2], 0., cfg.progress_upper)
        values = A@z
        minimum = float(min(np.min(values-lo), np.min(hi-values)))
        if minimum < -1e-5:
            return failure('hard_static_or_bounds')
        states = self._rollout(snapshot, z, xy_map, s_map)
        samples = field.rollout_samples(states[:, :2])
        cost = .5*cfg.period*sum(s.residual**2 for s in samples[:-1])
        if elapsed() >= cfg.cycle_budget:
            return failure('cycle_deadline_after_validation')
        self.previous = tuple(map(tuple, controls))
        self.previous_epoch = snapshot.epoch_ns
        self.previous_plan = (snapshot.route.plan_id, snapshot.route.map_revision, snapshot.route.generation)
        command = (float(controls[0, 0]), float(controls[0, 1]), 0.)
        return SolveResult(snapshot.digest, 'follow', 'solved_soft_dynamic', command,
                           tuple(map(tuple, controls)), tuple(map(tuple, states)), status, iterations,
                           solver_s, elapsed(), minimum, samples, nominal_cost, cost)
