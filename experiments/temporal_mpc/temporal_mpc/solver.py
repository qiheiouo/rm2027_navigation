"""Direct shooting MPC with hard constraints and checked, bounded brake proposals."""
from dataclasses import dataclass
import time
import numpy as np
from scipy.optimize import minimize

from .contracts import ContractError, Timeline, predict
from .dynamics import braking, rollout
from .geometry import boundary_clearance, clearance


@dataclass(frozen=True)
class Config:
    dt: float = 0.1
    horizon: float = 1.5
    collision_dt: float = 0.05
    blocks: int = 5
    velocity_lower: tuple = (-0.5, -0.5, -1.2)
    velocity_upper: tuple = (0.8, 0.5, 1.2)
    acceleration: tuple = (1.0, 1.0, 2.0)
    physical_half_extents: tuple = (0.325, 0.300)
    padding: float = 0.03
    margin: float = 0.02
    deadline_s: float = 0.5  # Offline feasibility budget, NOT the 20 Hz acceptance target.
    iterations: int = 45
    tolerance: float = 1e-5

    def __post_init__(self):
        scalar = [self.dt, self.horizon, self.collision_dt, self.padding, self.margin,
                  self.deadline_s, self.tolerance]
        if (not np.isfinite(scalar).all() or self.dt <= 0 or not 0.5 <= self.horizon <= 2
                or not 0 < self.collision_dt <= self.dt or self.deadline_s <= 0
                or type(self.blocks) is not int or not 1 <= self.blocks <= 20
                or type(self.iterations) is not int or not 1 <= self.iterations <= 200
                or self.physical_half_extents != (0.325, 0.300) or self.padding != 0.03
                or self.margin < 0.02 or not 0 < self.tolerance <= 1e-5):
            raise ValueError("invalid MPC configuration / fixed safety geometry")
        for ratio in (self.horizon / self.dt, self.dt / self.collision_dt):
            if abs(ratio - round(ratio)) > 1e-9:
                raise ValueError("MPC and collision grids must be integral")
        if self.blocks > self.steps:
            raise ValueError("more control blocks than model steps")
        for values in (self.velocity_lower, self.velocity_upper, self.acceleration):
            if len(values) != 3 or not np.isfinite(values).all():
                raise ValueError("invalid motion limits")
        if np.any(np.asarray(self.velocity_lower) >= self.velocity_upper) or np.any(np.asarray(self.acceleration) <= 0):
            raise ValueError("invalid motion bounds")

    @property
    def steps(self):
        return round(self.horizon / self.dt)

    @property
    def substeps(self):
        return round(self.dt / self.collision_dt)

    @property
    def half_extents(self):
        return tuple(h + self.padding for h in self.physical_half_extents)


@dataclass
class Result:
    status: str
    reason: str
    command: np.ndarray
    acceleration: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    elapsed_s: float
    cpu_s: float
    model_feasible: bool
    deadline_miss: bool
    optimizer_failures: int
    constraint_min: float | None
    objective: float | None


class Deadline(Exception):
    pass


class TemporalMPC:
    def __init__(self, config=Config(), consumption="temporal"):
        if consumption not in ("temporal", "current_only", "future_union"):
            raise ValueError("unknown consumption mechanism")
        self.config, self.consumption = config, consumption
        self.warm = None

    def reset(self):
        self.warm = None

    def solve(self, initial, state_epoch_ns, evaluation_ns, snapshot, reference,
              domain=(-1.0, 7.0, -1.2, 1.2), static=(), frame="map"):
        cfg = self.config
        start, cpu_start = time.perf_counter(), time.process_time()
        initial = np.asarray(initial, float)
        reference = np.asarray(reference, float)
        samples = cfg.steps * cfg.substeps
        times = np.arange(samples + 1) * cfg.collision_dt
        if (initial.shape != (6,) or not np.isfinite(initial).all() or frame != "map"
                or type(state_epoch_ns) is not int or state_epoch_ns != evaluation_ns
                or reference.shape != (samples + 1, 3) or not np.isfinite(reference).all()
                or len(domain) != 4 or not np.isfinite(domain).all()
                or domain[0] >= domain[1] or domain[2] >= domain[3]
                or len(static) > 64):
            self.reset()
            raise ContractError("state/reference/domain not synchronized in the evaluation world frame")
        # Execution holds the first acceleration for a full control tick. A
        # brake must not reach zero on a substep then assume a different input
        # during the remainder of that same tick.
        brake = np.repeat(braking(initial, cfg.steps, cfg.dt, cfg.acceleration), cfg.substeps, axis=0)
        brake_states = rollout(initial, brake, cfg.collision_dt)

        def finish(status, reason, controls, states, feasible, failures, value=None, cost=None):
            elapsed = time.perf_counter() - start
            if status in ("optimized", "feasible_iterate") and elapsed > cfg.deadline_s:
                self.reset()
                status, reason = ("brake" if brake_ok else "unsafe_brake"), "deadline"
                controls, states, feasible, value, cost = brake, brake_states, brake_ok, brake_margin, None
            return Result(status, reason, states[cfg.substeps, 3:].copy(), controls[0].copy(),
                          states, controls, elapsed, time.process_time() - cpu_start,
                          bool(feasible), elapsed > cfg.deadline_s, failures, value, cost)

        try:
            timeline = predict(snapshot, evaluation_ns, times)
        except ContractError as error:
            self.reset()
            return finish("invalid_input_brake", str(error), brake, brake_states, False, 0)
        if self.consumption == "current_only":
            timeline = Timeline(evaluation_ns, times,
                                tuple(np.broadcast_to(p[0], p.shape) for p in timeline.centers),
                                timeline.geometries, tuple(0.0 for _ in timeline.speeds), timeline.track_ids)
        # Static entries are explicit known occupied polygons, never future union.
        obstacles = list(zip(timeline.centers, timeline.geometries, timeline.speeds))
        for center, geometry in static:
            center = np.asarray(center, float)
            if center.shape != (2,) or not np.isfinite(center).all():
                raise ContractError("invalid static center")
            obstacles.append((np.broadcast_to(center, (samples + 1, 2)), geometry, 0.0))
        block_indices = np.minimum(np.arange(cfg.steps) * cfg.blocks // cfg.steps, cfg.blocks - 1)

        def expand(z):
            return np.repeat(np.asarray(z).reshape(cfg.blocks, 3)[block_indices], cfg.substeps, axis=0)

        def clearance_values(states):
            robot_speed = np.max(np.linalg.norm(states[:, 3:5], axis=1))
            point_speed = robot_speed + np.hypot(*cfg.half_extents) * np.max(np.abs(states[:, 5]))
            values = [boundary_clearance(states, domain, cfg.half_extents).ravel()
                      - 0.5 * cfg.collision_dt * point_speed]
            for centers, shape, speed in obstacles:
                if self.consumption == "future_union" and speed > 0:
                    # Only a diagnostic ablation. Each robot state sees every
                    # future location, deliberately destroying time semantics.
                    gaps = np.min([clearance(states, np.broadcast_to(p, centers.shape), shape, cfg.half_extents)
                                   for p in centers], axis=0)
                    reserve = 0.5 * cfg.collision_dt * point_speed
                else:
                    gaps = clearance(states, centers, shape, cfg.half_extents)
                    reserve = 0.5 * cfg.collision_dt * (point_speed + speed)
                values.append(gaps - cfg.margin - reserve)
            return np.concatenate(values)

        def hard_values(controls, states):
            return np.r_[clearance_values(states),
                         (states[:, 3:] - cfg.velocity_lower).ravel(),
                         (np.asarray(cfg.velocity_upper) - states[:, 3:]).ravel(),
                         (np.asarray(cfg.acceleration) - np.abs(controls)).ravel()]

        def feasible(controls, states):
            if not np.isfinite(controls).all() or not np.isfinite(states).all():
                return False, None
            values = hard_values(controls, states)
            value = float(values.min())
            return bool(value >= -cfg.tolerance and np.max(np.abs(states[-1, 3:])) <= cfg.tolerance), value

        brake_ok, brake_margin = feasible(brake, brake_states)
        if (np.any(initial[3:] < cfg.velocity_lower) or np.any(initial[3:] > cfg.velocity_upper)):
            self.reset()
            return finish("unsafe_brake", "measured velocity outside model bounds", brake, brake_states, False, 0, brake_margin)
        current = [float(boundary_clearance(initial[None, :], domain, cfg.half_extents).min())]
        for centers, shape, _ in obstacles:
            current.append(float(clearance(initial[None, :], centers[:1], shape, cfg.half_extents)[0] - cfg.margin))
        if min(current) < -cfg.tolerance:
            self.reset()
            return finish("unsafe_brake", "current footprint already violates model margin", brake, brake_states, False, 0, brake_margin)

        def check_time():
            if time.perf_counter() - start > cfg.deadline_s:
                raise Deadline()

        # Reuse the rollout at the same finite-difference point across callbacks.
        cache_z, cache_states, cache_controls = None, None, None

        def evaluate(z):
            nonlocal cache_z, cache_states, cache_controls
            check_time()
            if cache_z is None or not np.array_equal(z, cache_z):
                cache_z = np.array(z, copy=True)
                cache_controls = expand(z)
                cache_states = rollout(initial, cache_controls, cfg.collision_dt)
            return cache_controls, cache_states

        def objective(z):
            controls, states = evaluate(z)
            error = states[:, :2] - reference[:, :2]
            yaw_error = np.arctan2(np.sin(states[:, 2] - reference[:, 2]), np.cos(states[:, 2] - reference[:, 2]))
            path = 4.0 * np.sum(error**2) + np.sum(yaw_error**2)
            terminal = 10.0 * np.sum(error[-1]**2) + 3.0 * yaw_error[-1]**2
            velocity = 0.05 * np.sum(states[:, 3:]**2)
            acceleration = 0.04 * np.sum(controls**2)
            smoothness = 0.02 * np.sum(np.diff(controls, axis=0)**2)
            soft = 0.3 * np.sum(np.maximum(0.3 - clearance_values(states), 0)**2)
            return float(cfg.collision_dt * (path + velocity + acceleration + smoothness + soft) + terminal)

        # Goal-directed acceleration then a full stop is a seed, not a bypass.
        c, s = np.cos(initial[2]), np.sin(initial[2])
        delta = reference[-1, :2] - initial[:2]
        direction = np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1],
                              np.arctan2(np.sin(reference[-1, 2] - initial[2]), np.cos(reference[-1, 2] - initial[2]))])
        seed = np.clip(direction * 2 / cfg.horizon**2, -np.asarray(cfg.acceleration), cfg.acceleration)
        tracking_seed = np.tile(seed, (cfg.blocks, 1))
        tracking_seed[cfg.blocks // 2:] *= -1
        seeds = [tracking_seed.ravel(), np.zeros(cfg.blocks * 3)]
        if self.warm is not None:
            shifted = np.vstack((self.warm[1:], self.warm[-1:]))
            seeds.insert(0, np.asarray([shifted[block_indices == b].mean(axis=0)
                                      for b in range(cfg.blocks)]).ravel())
        bounds = [(-limit, limit) for _ in range(cfg.blocks) for limit in cfg.acceleration]
        candidates, failures, timed_out = [], 0, False
        try:
            for seed in seeds[:2]:
                check_time()
                result = minimize(objective, seed, method="SLSQP", bounds=bounds,
                                  constraints=[{"type": "ineq", "fun": lambda z: hard_values(*evaluate(z))},
                                               {"type": "eq", "fun": lambda z: evaluate(z)[1][-1, 3:]}],
                                  options={"maxiter": cfg.iterations, "ftol": 1e-6})
                if not result.success:
                    failures += 1
                if np.isfinite(result.x).all():
                    controls = expand(result.x)
                    states = rollout(initial, controls, cfg.collision_dt)
                    ok, constraint_min = feasible(controls, states)
                    if ok:
                        candidates.append((objective(result.x), controls, states, constraint_min, bool(result.success)))
                    elif result.success:
                        failures += 1
                elif result.success:
                    failures += 1
        except Deadline:
            timed_out = True
        except (FloatingPointError, ValueError):
            failures += 1
        if time.perf_counter() - start > cfg.deadline_s:
            timed_out = True
        if candidates and not timed_out:
            cost, controls, states, value, converged = min(candidates, key=lambda item: item[0])
            self.warm = controls[::cfg.substeps].copy()
            return finish("optimized" if converged else "feasible_iterate", "checked same-time trajectory",
                          controls, states, True, failures, value, cost)
        self.reset()
        return finish("brake" if brake_ok else "unsafe_brake",
                      "deadline" if timed_out else "no feasible optimizer result", brake, brake_states,
                      brake_ok, failures, brake_margin)
