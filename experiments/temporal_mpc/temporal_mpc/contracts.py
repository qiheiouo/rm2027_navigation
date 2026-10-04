"""Consumer values for the existing public prediction boundary, not a new ROS API."""
from dataclasses import dataclass
import math
import numpy as np

V1 = "rm_dynamic_obstacle_predictions/v1"
V2 = "rm_dynamic_obstacle_predictions/v2_observation_anchor"
NOMINAL_DIAMETER = 1.6970562748477143


class ContractError(ValueError):
    pass


def stamp_ns(stamp):
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ContractError("invalid integer ROS stamp")
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def polygon(points):
    """Require an ordered, strictly convex polygon, preserving its reference point."""
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or not 3 <= len(p) <= 128 or not np.isfinite(p).all():
        raise ContractError("invalid polygon")
    edges = np.roll(p, -1, axis=0) - p
    turns = np.cross(edges, np.roll(edges, -1, axis=0))
    if not (np.all(turns > 1e-10) or np.all(turns < -1e-10)):
        raise ContractError("polygon must be ordered and strictly convex")
    # Consistent turns alone do not exclude star/self-intersecting orderings.
    cross = np.cross(edges[:, None, :], p[None, :, :] - p[:, None, :])
    if not (np.all(cross >= -1e-10) or np.all(cross <= 1e-10)):
        raise ContractError("polygon is not convex")
    p = p.copy()
    p.setflags(write=False)
    return p


@dataclass(frozen=True)
class Geometry:
    kind: str
    offsets: tuple = ()
    radius: float = 0.0
    source: str = ""

    def __post_init__(self):
        if not self.source:
            raise ContractError("geometry needs explicit provenance")
        if self.kind == "circle":
            if self.offsets or not math.isfinite(self.radius) or not 0 < self.radius <= 3:
                raise ContractError("invalid circle")
        elif self.kind == "polygon":
            p = polygon(self.offsets)
            if self.radius != 0 or np.max(np.abs(p)) > 3:
                raise ContractError("invalid polygon extent")
            object.__setattr__(self, "offsets", tuple(map(tuple, p)))
        else:
            raise ContractError("unsupported geometry")


@dataclass(frozen=True)
class Track:
    track_id: int
    position: tuple
    velocity: tuple
    observation_ns: int
    state: str
    geometry: Geometry


@dataclass(frozen=True)
class Snapshot:
    source_ns: int
    tracks: tuple
    frame: str = "map"
    schema: str = V2
    complete: bool = True


@dataclass(frozen=True)
class Timeline:
    evaluation_ns: int
    relative_times: np.ndarray
    centers: tuple
    geometries: tuple
    speeds: tuple
    track_ids: tuple
    frame: str = "map"


def causal_snapshot(snapshots, evaluation_ns):
    """Bounded DDS receipt history; never backdate a newer scan to a request."""
    if type(evaluation_ns) is not int or evaluation_ns < 0 or len(snapshots) > 4:
        raise ContractError("invalid causal snapshot history")
    previous = -1
    selected = None
    for snapshot in snapshots:
        if (not isinstance(snapshot, Snapshot) or type(snapshot.source_ns) is not int
                or snapshot.source_ns <= previous):
            raise ContractError("unordered causal snapshot history")
        previous = snapshot.source_ns
        if snapshot.source_ns <= evaluation_ns:
            selected = snapshot
    return selected


def predict(snapshot, evaluation_ns, times, max_age=0.4):
    times = np.asarray(times, dtype=float)
    if (not isinstance(snapshot, Snapshot) or type(evaluation_ns) is not int or type(snapshot.source_ns) is not int
            or min(evaluation_ns, snapshot.source_ns) < 0 or snapshot.frame != "map"
            or snapshot.schema not in (V1, V2) or snapshot.complete is not True):
        raise ContractError("invalid snapshot epoch/frame/schema/completeness")
    if (times.ndim != 1 or len(times) < 2 or len(times) > 401
            or not np.isfinite(times).all() or times[0] != 0 or np.any(np.diff(times) <= 0)
            or times[-1] > 2.0 + 1e-9):
        raise ContractError("invalid relative time grid")
    age = (evaluation_ns - snapshot.source_ns) * 1e-9
    if not 0 <= age <= max_age:
        raise ContractError("source stale or future")
    if len(snapshot.tracks) > 64:
        raise ContractError("track budget exceeded")
    ids, centers, shapes, speeds = [], [], [], []
    for track in snapshot.tracks:
        if (not isinstance(track, Track) or not isinstance(track.geometry, Geometry)
                or type(track.track_id) is not int or not 0 <= track.track_id < 2**64
                or track.track_id in ids or track.state not in ("tentative", "confirmed", "coasting")
                or type(track.observation_ns) is not int or track.observation_ns < 0):
            raise ContractError("invalid track identity/state/observation")
        observation_age = (evaluation_ns - track.observation_ns) * 1e-9
        if not 0 <= observation_age <= max_age or track.observation_ns > snapshot.source_ns:
            raise ContractError("observation stale or future")
        if snapshot.schema == V2 and ((track.state != "coasting" and track.observation_ns != snapshot.source_ns)
                or (track.state == "coasting" and track.observation_ns >= snapshot.source_ns)):
            raise ContractError("v2 observation/scan epoch relation")
        p, v = np.asarray(track.position, float), np.asarray(track.velocity, float)
        if p.shape != (2,) or v.shape != (2,) or not np.isfinite([p, v]).all() or np.linalg.norm(v) > 3:
            raise ContractError("invalid position/velocity")
        # Current-only is an explicit conservative occupancy hypothesis, not a
        # reliability estimate. The public message has no confidence field.
        if track.state == "tentative":
            v = np.zeros(2)
        centers.append(p + (age + times[:, None]) * v)
        shapes.append(track.geometry)
        speeds.append(float(np.linalg.norm(v)))
        ids.append(track.track_id)
    frozen_times = times.copy()
    frozen_times.setflags(write=False)
    for center in centers:
        center.setflags(write=False)
    return Timeline(evaluation_ns, frozen_times, tuple(centers), tuple(shapes), tuple(speeds), tuple(ids))


class PublicAdapter:
    """Stateless geometry conversion with source-order/jump checks across receipts.

    Failed receipts clear the usable cache. Actual observed polygons cannot be
    reconstructed from size: requests for that mode are rejected.
    """
    def __init__(self, dt=0.1, steps=15, geometry_mode="visible_extent_proxy",
                 schema=V2, nominal_diameter=NOMINAL_DIAMETER):
        if (not math.isfinite(dt) or dt <= 0 or type(steps) is not int or not 1 <= steps <= 20
                or dt * steps > 2.0 + 1e-9 or schema not in (V1, V2)
                or geometry_mode not in ("visible_extent_proxy", "nominal_diameter")
                or not math.isfinite(nominal_diameter) or not 0 < nominal_diameter <= 3):
            raise ContractError("invalid adapter configuration")
        self.dt, self.steps, self.geometry_mode, self.schema = dt, steps, geometry_mode, schema
        self.nominal_diameter = nominal_diameter
        self.last_source_ns = None
        self.last_positions = {}
        self.usable = None

    def consume(self, message, evaluation_ns):
        self.usable = None
        try:
            source = stamp_ns(message.header.stamp)
            if (message.schema != self.schema or message.authority != "shadow_only"
                    or message.header.frame_id != "map" or message.complete is not True
                    or type(message.total_track_count) is not int
                    or message.total_track_count != len(message.tracks) or len(message.tracks) > 64
                    or message.prediction_steps != self.steps
                    or not math.isclose(message.prediction_dt, self.dt, abs_tol=1e-12, rel_tol=0)):
                raise ContractError("public schema/authority/frame/count/grid")
            if self.last_source_ns is not None and source <= self.last_source_ns:
                raise ContractError("non-increasing source epoch")
            tracks = []
            states = {1: "tentative", 2: "confirmed", 3: "coasting"}
            for t in message.tracks:
                fields = [t.position.x, t.position.y, t.position.z, t.velocity.x,
                          t.velocity.y, t.velocity.z, t.size.x, t.size.y, t.size.z]
                if (not all(math.isfinite(f) for f in fields)
                        or min(t.size.x, t.size.y) <= 0 or max(t.size.x, t.size.y) > 3
                        or t.size.z < 0 or abs(t.position.z) > 1e-6 or abs(t.velocity.z) > 1e-6
                        or t.state not in states or len(t.prediction) != self.steps
                        or any(not np.isfinite([p.x, p.y, p.z]).all() for p in t.prediction)):
                    raise ContractError("public track fields")
                previous = self.last_positions.get(t.track_id)
                if previous is not None:
                    elapsed = (source - self.last_source_ns) * 1e-9
                    if math.hypot(t.position.x - previous[0], t.position.y - previous[1]) > 0.5 + 3 * elapsed:
                        raise ContractError("track position jump")
                radius = (max(0.36, 0.5 * math.hypot(t.size.x, t.size.y))
                          if self.geometry_mode == "visible_extent_proxy" else self.nominal_diameter)
                geometry = Geometry("circle", radius=radius, source=self.geometry_mode + ":uncertified")
                tracks.append(Track(t.track_id, (t.position.x, t.position.y),
                                    (t.velocity.x, t.velocity.y), stamp_ns(t.last_observation_stamp),
                                    states[t.state], geometry))
            snapshot = Snapshot(source, tuple(tracks), schema=self.schema)
            predict(snapshot, evaluation_ns, np.arange(self.steps + 1) * self.dt)
            self.last_source_ns = source
            self.last_positions = {t.track_id: t.position for t in tracks}
            self.usable = snapshot
            return snapshot
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise ContractError(str(error)) from error
