"""Runtime controller selection policy; never publishes chassis commands."""
from dataclasses import dataclass
import math

MPPI = "FollowPathMPPI"
MPC = "FollowPathTemporalMPC"


@dataclass
class Selection:
    selected: str = MPPI
    health_time: float | None = None
    ready: bool = False
    reason: str = "startup baseline"

    def health(self, now, ready, fallback=False):
        if not math.isfinite(now) or now < 0 or type(ready) is not bool or type(fallback) is not bool:
            raise ValueError("invalid health")
        if self.health_time is not None and now < self.health_time:
            self.selected, self.ready, self.reason = MPPI, False, "health clock regression"
            return self.selected
        self.health_time = now
        self.ready = ready and not fallback
        if not self.ready:
            self.selected, self.reason = MPPI, "MPC degraded/fallback requested"
        return self.selected

    def tick(self, now):
        if (not math.isfinite(now) or self.health_time is None or now < self.health_time
                or now-self.health_time > .1):
            self.selected, self.ready, self.reason = MPPI, False, "MPC health stale"
        return self.selected

    def request(self, controller_id, now):
        self.tick(now)
        if controller_id not in (MPPI, MPC):
            raise ValueError("unknown controller_id")
        if controller_id == MPC and not self.ready:
            self.reason = "MPC request refused: not ready"
            return self.selected
        self.selected, self.reason = controller_id, "explicit runtime request"
        return self.selected
