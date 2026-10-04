"""Stage-aligned translated raster, with explicit configuration-space support.

The conservative world AABB of the padded, fixed-yaw robot is Minkowski-added
to EACH observed cell. Geometry quantization, robot padding/margin and optional
motion error are separate quantities. None certifies unseen object surfaces.
"""
from dataclasses import dataclass
import math
import numpy as np
from .contracts import ContractError


@dataclass(frozen=True)
class FieldSample:
    residual: float
    gradient: tuple
    clearance: float | None
    track_id: int | None
    plateau: bool


class TemporalSoftField:
    physical_half_extents = (.325, .300)
    robot_padding = .03
    geometric_margin = .02
    halo = .40
    slope = 16.
    residual_scale = 8.

    def __init__(self, snapshot, motion_error_speed=0.):
        if not math.isfinite(motion_error_speed) or not 0 <= motion_error_speed <= 3:
            raise ContractError("motion-error speed must be separately explicit")
        self.snapshot = snapshot
        self.motion_error_speed = motion_error_speed  # hypothesis, not covariance or certificate
        yaw = snapshot.state[2]
        hx, hy = np.asarray(self.physical_half_extents) + self.robot_padding
        self.robot_world_half = np.array([abs(math.cos(yaw))*hx+abs(math.sin(yaw))*hy,
                                          abs(math.sin(yaw))*hx+abs(math.cos(yaw))*hy])
        self.local_centres = tuple(np.asarray(t.origin)+(np.asarray(t.cells)+.5)*t.resolution
                                   for t in snapshot.tracks)
        for array in self.local_centres:
            array.setflags(write=False)

    def translated_cells(self, index, stage):
        if type(stage) is not int or not 0 <= stage <= 30:
            raise ContractError("stage grid")
        track = self.snapshot.tracks[index]
        dt = (self.snapshot.stage_epochs[stage]-track.source_ns)*1e-9
        # Anchor already coasting-advanced to source; do not add observation age.
        return self.local_centres[index] + np.asarray(track.anchor) + dt*np.asarray(track.velocity)

    def sample(self, xy, stage):
        p = np.asarray(xy, float)
        if p.shape != (2,) or not np.isfinite(p).all():
            raise ContractError("soft field query")
        best = FieldSample(0., (0., 0.), None, None, False)
        for index, track in enumerate(self.snapshot.tracks):
            centres = self.translated_cells(index, stage)
            motion_age = (self.snapshot.stage_epochs[stage]-track.observation_ns)*1e-9
            half = (self.robot_world_half + track.resolution/2 + self.geometric_margin
                    + self.motion_error_speed*motion_age)
            offset = p-centres
            d = np.abs(offset)-half
            outside = np.maximum(d, 0.)
            distances = np.linalg.norm(outside, axis=1) + np.minimum(np.max(d, axis=1), 0.)
            cell = int(np.argmin(distances))
            distance = float(distances[cell])
            if distance >= self.halo:
                continue
            residual = self.residual_scale*math.exp(-self.slope*max(distance, 0.))
            gradient = np.zeros(2)
            if distance > 0.:
                normal = outside[cell]/np.linalg.norm(outside[cell])*np.sign(offset[cell])
                gradient = -self.slope*residual*normal
            # max fused cost; all tracks remain in diagnostics, no future veto.
            if residual > best.residual:
                best = FieldSample(residual, tuple(gradient), distance, track.track_id, distance <= 0.)
        return best

    def rollout_samples(self, positions):
        if np.shape(positions) != (31, 2):
            raise ContractError("rollout soft diagnostics grid")
        return tuple(self.sample(position, k) for k, position in enumerate(positions))
