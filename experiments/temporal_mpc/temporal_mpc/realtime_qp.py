"""One bounded convex QP, with independent whole-trajectory acceptance.

Phase-one domain is fixed yaw, zero wz. Body ax/ay controls still permit
omnidirectional translation. Path topology and static occupancy belong to the
certified frontend Window. No global/static search is present in this controller.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy import sparse
import osqp
from .contracts import ContractError, predict
from .dynamics import braking, rollout
from .frontend import Window
from .geometry import clearance


@dataclass(frozen=True)
class QPConfig:
    period: float = .05             # 20 Hz execution
    node_dt: float = .1             # 15 model nodes, 30 decision variables
    horizon: float = 1.5
    cycle_budget: float = .04       # includes prediction, assembly and validation
    solver_budget: float = .015
    max_iterations: int = 400
    max_active_obstacles: int = 4
    physical_half_extents: tuple = (.325, .300)
    padding: float = .03
    margin: float = .02
    velocity_lower: tuple = (-.5, -.5)
    velocity_upper: tuple = (.8, .5)
    acceleration: tuple = (1., 1.)

    def __post_init__(self):
        if (self.period != .05 or self.node_dt != .1 or not 1 <= self.horizon <= 2
                or abs(self.horizon/self.node_dt-round(self.horizon/self.node_dt)) > 1e-9
                or not 0 < self.solver_budget <= .02 or not self.solver_budget < self.cycle_budget <= .045
                or type(self.max_iterations) is not int or not 1 <= self.max_iterations <= 500
                or type(self.max_active_obstacles) is not int or not 1 <= self.max_active_obstacles <= 4
                or self.physical_half_extents != (.325, .300) or self.padding != .03
                or self.margin != .02 or self.velocity_lower != (-.5, -.5)
                or self.velocity_upper != (.8, .5) or self.acceleration != (1., 1.)):
            raise ValueError("invalid bounded realtime configuration")

    @property
    def nodes(self):
        return round(self.horizon/self.node_dt)

    @property
    def times(self):
        return np.arange(2*self.nodes+1)*self.period


@dataclass
class QPResult:
    status: str
    reason: str
    command: np.ndarray
    acceleration: np.ndarray
    states: np.ndarray
    controls: np.ndarray
    model_feasible: bool
    fallback_id: str | None
    elapsed_s: float
    cpu_s: float
    solver_status: str
    iterations: int
    selected_ids: tuple
    constraint_min: float | None


class RealtimeMPC:
    def __init__(self, config=QPConfig()):
        self.config = config
        self.previous = None
        self.previous_epoch = None
        self.previous_plan = None
        self.times = config.times
        # Condensed exact fixed-yaw body acceleration integration matrices.
        t, left = self.times[:, None], np.arange(config.nodes)[None, :]*config.node_dt
        active = np.clip(t-left, 0., config.node_dt)
        self.vmap = active
        self.pmap = active*(t-left-.5*active)
        self.eye = np.eye(2*config.nodes)

    def reset(self):
        self.previous = self.previous_epoch = self.previous_plan = None

    def _expand(self, z):
        return np.c_[np.repeat(np.asarray(z).reshape(-1, 2), 2, axis=0),
                     np.zeros(2*self.config.nodes)]

    def solve(self, initial, epoch_ns, snapshot, window):
        cfg = self.config
        started, cpu = time.perf_counter(), time.process_time()
        x = np.asarray(initial, float)
        if x.shape != (6,) or not np.isfinite(x).all():
            self.reset()
            raise ContractError("invalid measured state; boundary watchdog must stop")
        brake = braking(x, 2*cfg.nodes, cfg.period, (*cfg.acceleration, 2.))
        brake_states = rollout(x, brake, cfg.period)
        timeline, selected_ids = None, ()
        status, iterations, minimum = "not_run", 0, None
        valid_window = (isinstance(window, Window) and type(epoch_ns) is int and epoch_ns >= 0
                        and window.epoch_ns == epoch_ns and window.frame == "map"
                        and bool(window.plan_id) and np.shape(window.reference) == (len(self.times), 3)
                        and np.isfinite(window.reference).all() and len(window.centre_bounds) == 4
                        and np.isfinite(window.centre_bounds).all()
                        and window.centre_bounds[0] < window.centre_bounds[1]
                        and window.centre_bounds[2] < window.centre_bounds[3])

        def check(controls, states):
            if timeline is None or not valid_window or abs(x[5]) > 1e-8:
                return False, None
            b = window.centre_bounds
            reserve = .5*cfg.period*np.max(np.linalg.norm(states[:, 3:5], axis=1))
            vals = [states[:, 0]-b[0]-reserve, b[1]-states[:, 0]-reserve,
                    states[:, 1]-b[2]-reserve, b[3]-states[:, 1]-reserve,
                    (states[:, 3:5]-cfg.velocity_lower).ravel(),
                    (np.asarray(cfg.velocity_upper)-states[:, 3:5]).ravel(),
                    (np.asarray(cfg.acceleration)-np.abs(controls[:, :2])).ravel()]
            # ALL tracks are revalidated, including those excluded from the QP.
            for centers, shape, speed in zip(timeline.centers, timeline.geometries, timeline.speeds):
                vals.append(clearance(states, centers, shape, (.355, .330))-cfg.margin
                            -.5*cfg.period*(np.max(np.linalg.norm(states[:, 3:5], axis=1))+speed))
            m = float(np.min(np.concatenate(vals)))
            terminal = float(np.max(np.abs(states[-1, 3:])))
            return bool(np.isfinite(states).all() and np.isfinite(controls).all()
                        and m >= -1e-6 and terminal <= 1e-5), m

        def finish(label, reason, controls, states, feasible, margin, request=None):
            elapsed = time.perf_counter()-started
            return QPResult(label, reason, states[1, 3:].copy(), controls[0].copy(),
                            states, controls, feasible, request, elapsed,
                            time.process_time()-cpu, status, iterations, selected_ids, margin)

        def fallback(reason):
            nonlocal minimum
            # Re-anchor shifted inputs to NEW measured state and NEW prediction.
            # No old safety verdict survives a state, source, or plan change.
            if (timeline is not None and valid_window and self.previous is not None
                    and self.previous_plan == window.plan_id and self.previous_epoch == epoch_ns-round(cfg.period*1e9)):
                shifted = np.vstack([self.previous[1:], np.zeros((1, 3))])
                shifted_states = rollout(x, shifted, cfg.period)
                ok, minimum = check(shifted, shifted_states)
                if ok and time.perf_counter()-started < cfg.cycle_budget:
                    self.previous, self.previous_epoch = shifted.copy(), epoch_ns
                    return finish("previous_feasible", reason, shifted, shifted_states, True, minimum, "FollowPathMPPI")
            self.reset()
            ok, minimum = check(brake, brake_states)
            return finish("brake" if ok else "uncertified_brake", reason, brake, brake_states,
                          ok, minimum, "FollowPathMPPI")

        if not valid_window:
            return fallback("invalid static reference/corridor")
        try:
            timeline = predict(snapshot, epoch_ns, self.times)
        except ContractError as error:
            return fallback(str(error))
        if (abs(x[5]) > 1e-8 or np.max(np.abs(np.arctan2(np.sin(window.reference[:, 2]-x[2]),
                np.cos(window.reference[:, 2]-x[2])))) > 1e-8):
            self.reset()
            return fallback("fixed-yaw phase-one domain violated")
        if time.perf_counter()-started >= cfg.cycle_budget:
            return fallback("assembly deadline")

        rot = np.array([[np.cos(x[2]), -np.sin(x[2])], [np.sin(x[2]), np.cos(x[2])]])
        pm = np.kron(self.pmap, rot)
        vm = np.kron(self.vmap, np.eye(2))
        p0 = x[:2]+self.times[:, None]*(rot@x[3:5])
        v0 = np.tile(x[3:5], len(self.times))
        weights = np.ones(len(self.times))*4*cfg.period
        weights[-1] += 10.
        weights = np.repeat(weights, 2)
        diff = np.diff(self.eye.reshape(cfg.nodes, 2, -1), axis=0).reshape(-1, 2*cfg.nodes)
        hessian = 2*(pm.T@(weights[:, None]*pm)+.05*vm.T@vm+.08*self.eye+.2*diff.T@diff)
        linear = 2*pm.T@(weights*(p0-window.reference[:, :2]).ravel())+.1*vm.T@v0
        rows = [self.eye, vm, vm[-2:]]
        lower = [-np.tile(cfg.acceleration, cfg.nodes), np.tile(cfg.velocity_lower,len(self.times))-v0, -x[3:5]]
        upper = [np.tile(cfg.acceleration,cfg.nodes), np.tile(cfg.velocity_upper,len(self.times))-v0, -x[3:5]]
        vmax = np.hypot(.8, .5)
        reserve = .5*cfg.period*vmax
        b = window.centre_bounds
        rows.append(pm)
        lower.append(np.tile([b[0]+reserve,b[2]+reserve],len(self.times))-p0.ravel())
        upper.append(np.tile([b[1]-reserve,b[3]-reserve],len(self.times))-p0.ravel())

        warm = None
        if (self.previous is not None and self.previous_plan == window.plan_id
                and self.previous_epoch == epoch_ns-round(cfg.period*1e9)):
            shifted = np.vstack([self.previous[1:],np.zeros((1,3))])
            warm = shifted[:, :2].reshape(cfg.nodes, 2, 2).mean(axis=1).ravel()
        seed = rollout(x, self._expand(warm), cfg.period) if warm is not None else brake_states
        relevant = []
        for i, (centers, shape) in enumerate(zip(timeline.centers, timeline.geometries)):
            radius = shape.radius if shape.kind == "circle" else np.max(np.linalg.norm(shape.offsets, axis=1))
            # Both reference and braking/warm trajectories participate in culling.
            distance = min(np.min(np.linalg.norm(centers-window.reference[:, :2],axis=1)),
                           np.min(np.linalg.norm(centers-seed[:, :2],axis=1))) - radius
            if distance < np.hypot(.355,.330)+.3:
                relevant.append((float(distance), i))
        if len(relevant) > cfg.max_active_obstacles:
            return fallback("active obstacle budget exceeded")
        selected = [i for _,i in sorted(relevant)]
        selected_ids = tuple(timeline.track_ids[i] for i in selected)
        for i in selected:
            centers, shape, speed = timeline.centers[i], timeline.geometries[i], timeline.speeds[i]
            for k, center in enumerate(centers):
                direction = seed[k,:2]-center
                if shape.kind == "polygon":
                    points = np.asarray(shape.offsets)
                    edges = np.roll(points,-1,axis=0)-points
                    normals = np.c_[-edges[:,1],edges[:,0]]
                    normals /= np.linalg.norm(normals,axis=1)[:,None]
                    axes = np.vstack([rot.T, normals])
                else:
                    axes = rot.T if np.linalg.norm(direction) < 1e-10 else np.vstack([rot.T, direction/np.linalg.norm(direction)])
                axes = np.vstack([axes,-axes])
                robot_support = np.abs(axes@rot)@np.array([.355,.330])
                obstacle_support = (np.full(len(axes),shape.radius) if shape.kind=="circle"
                                    else np.max(axes@np.asarray(shape.offsets).T,axis=1))
                gap = axes@direction-robot_support-obstacle_support
                best = int(np.argmax(gap)); n = axes[best]
                rhs = n@center+robot_support[best]+obstacle_support[best]+cfg.margin+.5*cfg.period*(vmax+speed)
                rows.append((n@pm[2*k:2*k+2])[None,:])
                lower.append(np.array([rhs-n@p0[k]]));upper.append(np.array([np.inf]))
        if time.perf_counter()-started >= cfg.cycle_budget-cfg.solver_budget-.003:
            return fallback("assembly deadline")
        try:
            qp = osqp.OSQP()
            qp.setup(P=sparse.csc_matrix(np.triu(hessian)),q=linear,A=sparse.csc_matrix(np.vstack(rows)),
                     l=np.concatenate(lower),u=np.concatenate(upper),verbose=False,
                     max_iter=cfg.max_iterations,time_limit=cfg.solver_budget,
                     eps_abs=1e-6,eps_rel=1e-6,polishing=False,warm_starting=True,
                     check_termination=10)
            if warm is not None:
                qp.warm_start(x=warm)
            answer = qp.solve(raise_error=False)
            status, iterations = answer.info.status, int(answer.info.iter)
            if answer.x is not None and np.isfinite(answer.x).all() and status in (
                    "solved", "solved inaccurate", "maximum iterations reached"):
                controls = self._expand(answer.x)
                states = rollout(x,controls,cfg.period)
                ok, minimum = check(controls,states)
                if ok and time.perf_counter()-started < cfg.cycle_budget:
                    self.previous,self.previous_epoch,self.previous_plan=controls.copy(),epoch_ns,window.plan_id
                    return finish("optimized" if status=="solved" else "feasible_iterate",
                                  "revalidated temporal QP",controls,states,True,minimum)
                if not ok:
                    status += "; trajectory rejected by hard checks"
        except (ValueError,RuntimeError,osqp.OSQPException) as error:
            status = type(error).__name__
        return fallback("cycle deadline" if time.perf_counter()-started >= cfg.cycle_budget else status)
