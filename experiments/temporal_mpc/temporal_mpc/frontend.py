"""Consume a T-DT static path/corridor outside the control loop.

The bridge reuses the frozen migrated YAstar and SfcSquare. This module only
resamples that route and checks its static certificate; it never searches.
"""
from dataclasses import dataclass
import hashlib
import json
import subprocess
import numpy as np
from .contracts import ContractError


@dataclass(frozen=True)
class Window:
    epoch_ns: int
    reference: np.ndarray
    centre_bounds: tuple
    plan_id: str
    frame: str = "map"


class StaticRoute:
    def __init__(self, document, costs, resolution, origin):
        self.document = document
        self.path = np.asarray(document["path"], float)
        self.anchors = np.asarray([a["position"] for a in document["anchors"]], float)
        self.bounds = np.asarray([a["centre_bounds"] for a in document["anchors"]], float)
        if (self.path.ndim != 2 or self.path.shape[1] != 2 or not 2 <= len(self.path) <= 256
                or not 2 <= len(self.anchors) <= 4096 or self.bounds.shape != (len(self.anchors), 4)
                or not np.isfinite(self.path).all() or not np.isfinite(self.anchors).all()
                or not np.isfinite(self.bounds).all()):
            raise ContractError("invalid static route")
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.anchors, axis=0), axis=1))]
        if np.any(np.diff(self.arc) <= 1e-8):
            raise ContractError("degenerate static route")
        # Independent continuous rectangle-vs-raw-cell certification. Unknown,
        # inscribed and lethal cells stay blocked. No reliance on planner success.
        grid = np.asarray(costs)
        if grid.ndim != 2 or not np.isfinite(grid).all() or np.any((grid < 0) | (grid > 255)):
            raise ContractError("invalid raw static grid")
        blocked = grid >= 253
        blocked[[0, -1], :] = True
        blocked[:, [0, -1]] = True
        iy, ix = np.where(blocked)
        left, bottom = origin[0] + ix * resolution, origin[1] + iy * resolution
        reach = np.hypot(.355, .330) + .02

        def certified(b):
            dx = np.maximum.reduce([left - b[1], b[0] - left - resolution, np.zeros(len(left))])
            dy = np.maximum.reduce([bottom - b[3], b[2] - bottom - resolution, np.zeros(len(left))])
            return (np.min(np.hypot(dx, dy)) >= reach - 1e-6
                    and b[0] >= origin[0] and b[1] <= origin[0] + grid.shape[1]*resolution
                    and b[2] >= origin[1] and b[3] <= origin[1] + grid.shape[0]*resolution)

        for a, b in zip(self.anchors, self.bounds):
            if not (b[0] < a[0] < b[1] and b[2] < a[1] < b[3]):
                raise ContractError("anchor outside centre corridor")
            if not certified(b):
                raise ContractError("corridor does not certify complete padded footprint against raw static grid")
        # SfcSquare deliberately emits local squares. Compress consecutive
        # squares into a rectangle ONLY when its entire hull independently
        # certifies against the original static map. This is frontend work,
        # performed once; no topology search or raw map scan runs in MPC.
        merged = self.bounds.copy()
        for i in range(len(merged)):
            b = merged[i]
            for j in range(i+1, len(self.bounds)):
                c = self.bounds[j]
                if c[0] > b[1] or b[0] > c[1] or c[2] > b[3] or b[2] > c[3]:
                    break
                hull = np.array([min(b[0],c[0]),max(b[1],c[1]),min(b[2],c[2]),max(b[3],c[3])])
                if not certified(hull):
                    break
                b = hull
            merged[i] = b
        self.bounds = merged
        payload = json.dumps(document, sort_keys=True).encode() + grid.tobytes()
        self.plan_id = hashlib.sha256(payload).hexdigest()

    def window(self, initial, epoch_ns, times, cruise=.6):
        p = np.asarray(initial[:2])
        # Choose a containing certified region near the geometric route, then
        # cap progression within it. Waiting never requires reaching a new
        # corridor merely because wall time advances.
        candidates = np.flatnonzero((self.bounds[:, 0] < p[0]) & (p[0] < self.bounds[:, 1])
                                   & (self.bounds[:, 2] < p[1]) & (p[1] < self.bounds[:, 3]))
        if not len(candidates):
            raise ContractError("measured centre outside all static corridors")
        index = candidates[np.argmin(np.linalg.norm(self.anchors[candidates] - p, axis=1))]
        arc = np.minimum(self.arc[index] + np.asarray(times)*cruise, self.arc[-1])
        xy = np.c_[np.interp(arc, self.arc, self.anchors[:, 0]),
                   np.interp(arc, self.arc, self.anchors[:, 1])]
        b = self.bounds[index]
        xy = np.clip(xy, [b[0]+.04, b[2]+.04], [b[1]-.04, b[3]-.04])
        return Window(epoch_ns, np.c_[xy, np.full(len(times), initial[2])], tuple(b), self.plan_id)


def prepare_route(executable, costs, resolution, origin, start, goal, supplied_path=None):
    raw = np.asarray(costs)
    if (raw.ndim != 2 or min(raw.shape) < 3 or max(raw.shape) > 1000 or raw.size > 100000
            or not np.issubdtype(raw.dtype, np.integer) or np.any(raw < 0) or np.any(raw > 255)):
        raise ContractError("raw static costs must be bounded integer Nav2 bytes; no wrapping/truncation")
    grid = raw.astype(np.uint8)
    header = [grid.shape[1], grid.shape[0], resolution, *origin, *start[:2], *goal[:2]]
    wire = " ".join(map(str, header)) + "\n" + " ".join(map(str, grid.ravel())) + "\n"
    arguments = [str(executable)]
    if supplied_path is not None:
        path = np.asarray(supplied_path, float)
        if (path.ndim != 2 or path.shape[1] != 2 or not 2 <= len(path) <= 256
                or not np.isfinite(path).all() or np.any(np.linalg.norm(np.diff(path, axis=0),axis=1) < 1e-7)
                or not np.allclose(path[0], start[:2]) or not np.allclose(path[-1], goal[:2])):
            raise ContractError("invalid externally planned path")
        wire += str(len(path)) + "\n" + " ".join(map(str, path.ravel())) + "\n"
        arguments.append("--path")
    result = subprocess.run(arguments, input=wire, text=True, capture_output=True, timeout=2.)
    if result.returncode:
        raise ContractError(f"T-DT frontend failed ({result.returncode}): {result.stderr[:200]}")
    return StaticRoute(json.loads(result.stdout), grid, resolution, origin)
