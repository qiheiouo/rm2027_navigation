"""Single-output lease/STOP policy and current-grid first-interval protection.

This module has NO future-prediction input. An independent publisher process
must own tick(); compute may only offer a leased command. MPPI can use the same
offer/guard boundary. The offline probe uses simulated occupancy, not ROS.
"""
from dataclasses import dataclass
import math
import numpy as np
from .contracts import ContractError, integer, vector
from .follow import bounded_brake


@dataclass(frozen=True)
class GuardVerdict:
    clear: bool
    reason: str
    checked_ns: int
    map_revision: str


class CurrentGrid:
    def __init__(self, costs, resolution, origin, source_ns, revision):
        grid = np.asarray(costs)
        if (grid.ndim != 2 or min(grid.shape) < 3 or grid.size > 100000
                or not np.issubdtype(grid.dtype, np.integer) or np.any((grid < 0) | (grid > 255))
                or not math.isfinite(resolution) or resolution <= 0 or not revision):
            raise ContractError("current raw Nav2 grid")
        self.grid = grid.copy()
        self.grid.setflags(write=False)
        self.resolution, self.origin = resolution, vector(origin, 2, 'current grid origin')
        self.source_ns, self.revision = integer(source_ns, 'current grid source'), str(revision)
        iy, ix = np.where(self.grid >= 253)  # unknown, inscribed and lethal
        self.blocked_centres = np.c_[self.origin[0]+(ix+.5)*resolution,
                                     self.origin[1]+(iy+.5)*resolution]

    def check(self, pose, pose_ns, command, epoch_ns):
        """Continuous translation sweep of padded fixed-yaw footprint, 50ms only."""
        pose = vector(pose, 6, 'current measured pose')
        command = vector(command, 3, 'current guard command')
        integer(epoch_ns, 'guard epoch')
        integer(pose_ns, 'guard pose epoch')
        def verdict(clear, reason):
            return GuardVerdict(clear, reason, epoch_ns, self.revision)
        if not 0 <= epoch_ns-pose_ns <= 150_000_000 or not 0 <= epoch_ns-self.source_ns <= 400_000_000:
            return verdict(False, 'current_input_TTL')
        if abs(pose[5]) > 1e-8 or abs(command[2]) > 1e-8:
            return verdict(False, 'fixed_yaw_gate')
        if not (-.5 <= command[0] <= .8 and -.5 <= command[1] <= .5):
            return verdict(False, 'command_bounds')
        yaw = pose[2]
        ex, ey = np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])
        displacement = .05*(command[0]*ex+command[1]*ey)
        half = np.array([.355, .330])  # original mechanical model + .03 padding
        world_half = np.abs(ex)*half[0]+np.abs(ey)*half[1]
        left = np.asarray(pose[:2])+np.minimum(displacement, 0.)-world_half
        right = np.asarray(pose[:2])+np.maximum(displacement, 0.)+world_half
        if (np.any(left < self.origin) or np.any(right > np.asarray(self.origin)+np.array(self.grid.shape[::-1])*self.resolution)):
            return verdict(False, 'current_map_boundary')
        axes = [np.array([1., 0.]), np.array([0., 1.]), ex, ey]
        if np.linalg.norm(displacement) > 1e-12:
            axes.append(np.array([-displacement[1], displacement[0]])/np.linalg.norm(displacement))
        collision = np.ones(len(self.blocked_centres), bool)
        delta = self.blocked_centres-np.asarray(pose[:2])
        for axis in axes:
            robot_support = abs(axis@ex)*half[0]+abs(axis@ey)*half[1]
            cell_support = self.resolution/2*np.abs(axis).sum()
            end = axis@displacement
            projection = delta@axis
            collision &= ((projection+cell_support >= min(0., end)-robot_support)
                          & (projection-cell_support <= max(0., end)+robot_support))
        return verdict(not bool(np.any(collision)), 'current_occupied_sweep' if np.any(collision) else 'current_clear')


@dataclass(frozen=True)
class CommandOffer:
    command: tuple
    acquired_steady_ns: int
    valid_until_steady_ns: int
    path_generation: int
    map_revision: str
    snapshot_digest: str
    source: str


@dataclass(frozen=True)
class Output:
    command: tuple
    mode: str
    reason: str
    source: str
    brake_certified: bool = False
    dynamic_long_horizon_vetoes: int = 0


class OutputArbiter:
    """Pure tick policy; caller records sent() only AFTER successful publication.

    Generations and current occupancy may cancel an offer. New prediction
    messages cannot replace the immutable solve or veto its 1.5s rollout.
    """
    def __init__(self, last_command=(0., 0., 0.)):
        self.last_command = vector(last_command, 3, 'last sent command')
        if not (-.5 <= last_command[0] <= .8 and -.5 <= last_command[1] <= .5) or abs(last_command[2]) > 1e-8:
            raise ContractError('last command bounds')
        self.offer = None
        self.last_offer_ns = None
        self.last_tick_ns = None

    def submit(self, offer):
        if not isinstance(offer, CommandOffer) or offer.source not in ('r4', 'mppi'):
            raise ContractError('single output source')
        command = vector(offer.command, 3, 'offered command')
        if not (-.5 <= command[0] <= .8 and -.5 <= command[1] <= .5) or abs(command[2]) > 1e-8:
            raise ContractError('offer bounds')
        for value in (offer.acquired_steady_ns, offer.valid_until_steady_ns, offer.path_generation):
            integer(value, 'offer epoch/generation')
        if not 0 < offer.valid_until_steady_ns-offer.acquired_steady_ns <= 75_000_000 or not offer.map_revision or not offer.snapshot_digest:
            raise ContractError('offer lease/provenance')
        if self.last_offer_ns is not None and offer.acquired_steady_ns <= self.last_offer_ns:
            raise ContractError('late or duplicate command offer')
        from dataclasses import replace
        self.offer = replace(offer, command=command)
        self.last_offer_ns = offer.acquired_steady_ns

    def offer_result(self, snapshot, result):
        if result.snapshot_digest != snapshot.digest:
            self.offer = None
            raise ContractError('result belongs to another cycle')
        if result.status != 'follow':
            self.offer = None
            return
        self.submit(CommandOffer(result.command, snapshot.acquired_steady_ns,
                                 snapshot.acquired_steady_ns+75_000_000,
                                 snapshot.route.generation, snapshot.route.map_revision,
                                 snapshot.digest, 'r4'))

    def tick(self, steady_ns, epoch_ns, path_generation, map_revision, current_revision, current_guard):
        integer(steady_ns, 'publisher steady epoch')
        integer(epoch_ns, 'publisher source epoch')
        if self.last_tick_ns is not None and steady_ns <= self.last_tick_ns:
            raise ContractError('publisher clock order')
        interval = .05 if self.last_tick_ns is None else min(.05, (steady_ns-self.last_tick_ns)*1e-9)
        self.last_tick_ns = steady_ns
        offer = self.offer
        reason = 'no_offer'
        if offer is not None:
            if not offer.acquired_steady_ns <= steady_ns < offer.valid_until_steady_ns:
                reason = 'command_lease_expired_or_future'
            elif offer.path_generation != path_generation or offer.map_revision != map_revision:
                reason = 'static_generation_changed'
            else:
                command = np.asarray(self.last_command[:2])+np.clip(np.asarray(offer.command[:2])-self.last_command[:2], -interval, interval)
                command = (*map(float, command), 0.)
                guard = current_guard(command)
                if (isinstance(guard, GuardVerdict) and guard.clear is True
                        and guard.map_revision == current_revision and guard.checked_ns == epoch_ns):
                    speed = math.hypot(*command[:2])
                    return Output(command, 'wait' if speed < .03 else 'follow', 'current_guard_clear', offer.source)
                reason = guard.reason if isinstance(guard, GuardVerdict) else 'invalid_current_guard'
                if isinstance(guard, GuardVerdict) and guard.checked_ns != epoch_ns:
                    reason = 'cached_current_guard'
        command = bounded_brake(self.last_command, interval)
        guard = current_guard(command)
        # Bounded brake is still published when occupancy is unsafe. It is not
        # labelled certified and does not pretend the robot can stop instantly.
        if (not isinstance(guard, GuardVerdict) or not guard.clear or guard.checked_ns != epoch_ns
                or guard.map_revision != current_revision):
            reason += ':brake_current_uncertified'
        return Output(command, 'stop', reason, 'brake')

    def sent(self, output):
        self.last_command = vector(output.command, 3, 'actually sent command')
