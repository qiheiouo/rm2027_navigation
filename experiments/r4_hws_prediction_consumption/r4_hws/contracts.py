"""Strict, deeply immutable boundary for an offline R4 control cycle.

Public v2 anchors are already advanced to source_ns. Shape cells are relative
to that anchor; only (stage_epoch - source_epoch) is added by the renderer.
The sidecar is observed evidence, never a certified hidden physical body.
"""
from dataclasses import dataclass, asdict
import hashlib
import json
import math
import numpy as np

V2 = "rm_dynamic_obstacle_predictions/v2_observation_anchor"
SHAPE_V1 = "r4_observed_raster/v1"


class ContractError(ValueError):
    pass


def integer(value, name, lower=0, upper=2**63-1):
    if type(value) is not int or not lower <= value <= upper:
        raise ContractError(name)
    return value


def vector(value, length, name):
    try:
        if len(value) != length or any(not isinstance(x, (int, float, np.integer, np.floating))
                                       or isinstance(x, (bool, np.bool_)) for x in value):
            raise ContractError(name)
        out = tuple(float(x) for x in value)
    except TypeError as error:
        raise ContractError(name) from error
    if not all(math.isfinite(x) for x in out):
        raise ContractError(name)
    return out


@dataclass(frozen=True)
class ObservedTrack:
    track_id: int
    state: str
    source_ns: int
    observation_ns: int
    anchor: tuple
    velocity: tuple
    size: tuple                 # public metadata, NOT a shape generator
    resolution: float
    origin: tuple               # local lower-left grid corner relative to anchor
    cells: tuple
    association_sequence: int
    detection_index: int
    member_ids: tuple           # source candidate/ray IDs, not indices after reorder
    provenance: str


@dataclass(frozen=True)
class PreparedRoute:
    """Value copy of the reused raw-static certificate; no time reference."""
    points: tuple
    arcs: tuple
    regions: tuple
    plan_id: str
    map_revision: str
    generation: int

    def __post_init__(self):
        points = tuple(vector(p, 2, 'route point') for p in self.points)
        regions = tuple(vector(b, 4, 'route region') for b in self.regions)
        arcs = tuple(float(s) for s in self.arcs)
        if (not 2 <= len(points) <= 4096 or len(regions) != len(points) or len(arcs) != len(points)
                or not all(math.isfinite(s) for s in arcs) or arcs[0] != 0.
                or any(arcs[i+1] <= arcs[i] for i in range(len(arcs)-1))
                or not self.plan_id or not self.map_revision):
            raise ContractError('prepared route values')
        for b in regions:
            if b[0] >= b[1] or b[2] >= b[3]:
                raise ContractError('prepared route region')
        object.__setattr__(self, 'points', points)
        object.__setattr__(self, 'arcs', arcs)
        object.__setattr__(self, 'regions', regions)
        integer(self.generation, 'prepared generation')

    @classmethod
    def from_frontend(cls, route, map_revision, generation):
        # Only this adapter accepts a mutable frontend. The cycle copies values.
        from .frontend import StaticRoute
        if not isinstance(route, StaticRoute) or not map_revision:
            raise ContractError("a certified static frontend is required")
        return cls(tuple(tuple(map(float, p)) for p in route.anchors), tuple(map(float, route.arc)),
                   tuple(tuple(map(float, b)) for b in route.bounds), route.plan_id, str(map_revision),
                   integer(generation, "path generation"))

    def local(self, position):
        p = np.asarray(position)
        points, regions = np.asarray(self.points), np.asarray(self.regions)
        indices = np.flatnonzero((regions[:, 0] < p[0]) & (p[0] < regions[:, 1])
                                & (regions[:, 2] < p[1]) & (p[1] < regions[:, 3]))
        if not len(indices):
            raise ContractError("state outside certified static corridor")
        index = indices[np.argmin(np.linalg.norm(points[indices] - p, axis=1))]
        edges = np.diff(points, axis=0)
        frac = np.clip(np.einsum('ij,ij->i', p-points[:-1], edges)
                       / np.einsum('ij,ij->i', edges, edges), 0., 1.)
        projected = points[:-1] + frac[:, None]*edges
        segment = int(np.argmin(np.linalg.norm(projected-p, axis=1)))
        s = self.arcs[segment] + frac[segment]*(self.arcs[segment+1]-self.arcs[segment])
        return float(s), tuple(self.regions[index])

    def sample(self, s):
        points, arcs = np.asarray(self.points), np.asarray(self.arcs)
        s = np.clip(np.asarray(s), 0., arcs[-1])
        segments = np.clip(np.searchsorted(arcs, s, side='right')-1, 0, len(arcs)-2)
        tangent = (points[segments+1]-points[segments])/(arcs[segments+1]-arcs[segments])[..., None]
        return points[segments] + (s-arcs[segments])[..., None]*tangent, tangent


@dataclass(frozen=True)
class CycleSnapshot:
    cycle_id: int
    epoch_ns: int
    acquired_steady_ns: int
    state_ns: int
    tf_ns: int
    state: tuple
    last_command: tuple
    last_command_ns: int
    source_sequence: int
    source_ns: int
    tracks: tuple
    route: PreparedRoute
    progress: float
    centre_bounds: tuple
    digest: str

    @property
    def stage_epochs(self):
        return tuple(self.epoch_ns + k*50_000_000 for k in range(31))


def _freeze_cycle(*, cycle_id, epoch_ns, acquired_steady_ns, state_ns, tf_ns,
                 state, last_command, last_command_ns, public, shapes, route):
    """Copy all IO to tuples once. No fallback cache after an invalid receipt."""
    integer(cycle_id, "cycle ID")
    for value in (epoch_ns, acquired_steady_ns, state_ns, tf_ns, last_command_ns):
        integer(value, "cycle/source epoch")
    if not 0 <= epoch_ns-state_ns <= 150_000_000 or tf_ns != state_ns:
        raise ContractError("state TTL or source-time TF")
    if not 0 <= epoch_ns-last_command_ns <= 150_000_000:
        raise ContractError("last actually sent command TTL")
    x, cmd = vector(state, 6, "state"), vector(last_command, 3, "last command")
    if (abs(x[5]) > 1e-8 or abs(cmd[2]) > 1e-8
            or not (-.5 <= x[3] <= .8 and -.5 <= x[4] <= .5)
            or not (-.5 <= cmd[0] <= .8 and -.5 <= cmd[1] <= .5)):
        raise ContractError("fixed yaw / measured and sent velocity bounds")
    if not isinstance(route, PreparedRoute):
        raise ContractError("prepared static route required")
    source = integer(public["source_ns"], "prediction source")
    sequence = integer(public["sequence"], "prediction sequence")
    if (public["schema"] != V2 or public["frame"] != "map"
            or public["authority"] != "shadow_only" or public["complete"] is not True
            or type(public["prediction_steps"]) is not int or public["prediction_steps"] != 15
            or public["prediction_dt"] != .1
            or type(public["total_track_count"]) is not int
            or public["total_track_count"] != len(public["tracks"])
            or len(public["tracks"]) > 4 or not 0 <= epoch_ns-source <= 400_000_000):
        raise ContractError("public v2 schema/count/TTL/grid/track budget")
    if (shapes["schema"] != SHAPE_V1 or shapes["source_ns"] != source
            or shapes["sequence"] != sequence or shapes["frame"] != "map"
            or shapes["complete"] is not True or len(shapes["tracks"]) != len(public["tracks"])):
        raise ContractError("shape/public epoch identity or completeness")
    integer(shapes['source_ns'], 'sidecar source')
    integer(shapes['sequence'], 'sidecar sequence')
    shape_by_id = {}
    for shape in shapes["tracks"]:
        sid = integer(shape["track_id"], "shape track ID", upper=2**64-1)
        if sid in shape_by_id:
            raise ContractError("duplicate shape ID")
        shape_by_id[sid] = shape
    tracks = []
    ids = set()
    for track in public["tracks"]:
        tid = integer(track["track_id"], "track ID", upper=2**64-1)
        observation = integer(track["observation_ns"], "observation epoch")
        state_name = track["state"]
        if (tid in ids or tid not in shape_by_id or state_name not in ('tentative', 'confirmed', 'coasting')
                or not 0 <= epoch_ns-observation <= 400_000_000 or observation > source
                or (state_name == 'confirmed' and observation != source)
                or (state_name == 'coasting' and observation >= source)):
            raise ContractError("track identity/lifecycle/observation TTL")
        ids.add(tid)
        p = vector(track["position"], 2, "public anchor")
        v = vector(track["velocity"], 2, "public CV velocity")
        size = vector(track["size"], 2, "public visible extent metadata")
        if math.hypot(*v) > 3 or min(size) <= 0 or max(size) > 3:
            raise ContractError("public velocity/extent bound")
        display = tuple(vector(a, 2, "display anchor") for a in track["prediction"])
        if len(display) != 15 or any(math.dist(a, (p[0]+(i+1)*.1*v[0], p[1]+(i+1)*.1*v[1])) > 1e-6
                                     for i, a in enumerate(display)):
            raise ContractError("v2 display must be source-aligned unclipped CV")
        shape = shape_by_id[tid]
        integer(shape['observation_ns'], 'shape observation epoch')
        resolution = shape["resolution"]
        if (shape["observation_ns"] != observation or vector(shape["anchor"], 2, "shape anchor") != p
                or resolution != .05 or shape["provenance"] != 'associated_observed_endpoints:uncertified'
                or shape["association_sequence"] > sequence):
            raise ContractError("shape origin/observation/association provenance")
        association_sequence = integer(shape["association_sequence"], "association sequence")
        detection_index = integer(shape["detection_index"], "detection index", upper=4095)
        origin = vector(shape["origin"], 2, "local raster origin")
        cells = tuple(tuple(integer(i, "cell index", upper=120) for i in cell) for cell in shape["cells"])
        members = tuple(integer(i, "member ID", upper=2**32-1) for i in shape["member_ids"])
        if (not 1 <= len(cells) <= 512 or any(len(c) != 2 for c in cells)
                or len(set(cells)) != len(cells) or not 1 <= len(members) <= 4096
                or len(set(members)) != len(members)
                or any(abs(origin[a]+(c[a]+.5)*resolution) > 3 for c in cells for a in range(2))):
            raise ContractError("observed raster/member budget or extent")
        # Tentative geometry remains present; motion is explicitly current-only.
        tracks.append(ObservedTrack(tid, state_name, source, observation, p,
                                    (0., 0.) if state_name == 'tentative' else v, size,
                                    resolution, origin, cells, association_sequence,
                                    detection_index, members, shape["provenance"]))
    s, bounds = route.local(x[:2])
    snap = CycleSnapshot(cycle_id, epoch_ns, acquired_steady_ns, state_ns, tf_ns, x, cmd,
                         last_command_ns, sequence, source, tuple(tracks), route, s, bounds, '')
    payload = json.dumps(asdict(snap), sort_keys=True, allow_nan=False, separators=(',', ':')).encode()
    from dataclasses import replace
    return replace(snap, digest=hashlib.sha256(payload).hexdigest())


def freeze_cycle(**values):
    try:
        return _freeze_cycle(**values)
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ContractError(str(error)) from error


class ReceiptOrder:
    """Fail closed after duplicate, backwards or discontinuous public receipt."""
    def __init__(self):
        self.last_source = self.last_sequence = None
        self.positions = {}
        self.usable = None

    def accept(self, snapshot):
        self.usable = None
        if self.last_source is not None:
            if snapshot.source_ns <= self.last_source or snapshot.source_sequence <= self.last_sequence:
                raise ContractError("non-increasing prediction receipt")
            dt = (snapshot.source_ns-self.last_source)*1e-9
            for track in snapshot.tracks:
                if track.track_id in self.positions and math.dist(track.anchor, self.positions[track.track_id]) > .5+3*dt:
                    raise ContractError("public track position jump")
        self.last_source, self.last_sequence = snapshot.source_ns, snapshot.source_sequence
        self.positions = {t.track_id: t.anchor for t in snapshot.tracks}
        self.usable = snapshot
        return snapshot
