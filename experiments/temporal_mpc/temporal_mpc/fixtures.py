"""Fixed fixtures. Present measurement and future truth audit are separate calls."""
from dataclasses import dataclass
import math
import numpy as np
from .contracts import Geometry, NOMINAL_DIAMETER, Snapshot, Track


def box(hx, hy):
    return ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))


@dataclass(frozen=True)
class Scenario:
    name: str
    trajectory: str
    start: tuple = (0., 0., 0., 0., 0., 0.)
    goal: tuple = (5.6, 0., 0.)
    domain: tuple = (-1., 7., -1.2, 1.2)
    duration: float = 12.0
    geometry: str = "full_current_polygon"
    speed_bound: float = 0.8

    def current_motion(self, t):
        if self.trajectory == "static":
            return (2.8, 0.0), (0.0, 0.0)
        if self.trajectory == "low":
            return (2.0, -0.9 + 0.25 * t), (0., 0.25)
        if self.trajectory == "high":
            return (2.0, -1.4 + 0.8 * t), (0., 0.8)
        if self.trajectory == "head_on":
            return (4.8 - 0.4 * t, 0.), (-0.4, 0.)
        if self.trajectory == "stop":
            return (2.0, -0.9 + 0.4 * min(t, 2.0)), (0., 0.4 if t < 2.0 else 0.)
        if self.trajectory == "reverse":
            return (2.0, -0.9 + 0.4 * (t if t < 2 else 4 - t)), (0., 0.4 if t < 2 else -0.4)
        if self.trajectory == "sine":
            phase = 2.0 + t
            return (2.8, 0.9 * math.sin(2 * math.pi * phase / 8)), (0., 0.9 * 2 * math.pi / 8 * math.cos(2 * math.pi * phase / 8))
        if self.trajectory == "wait":
            return (1.3, -0.7 + 0.3 * t), (0., 0.3)
        if self.trajectory == "reopen":
            return (1.3, 0.3 * max(0., t - 2.0)), (0., 0. if t < 2 else 0.3)
        raise ValueError("unknown fixture trajectory")

    def truth(self, t):
        return self.current_motion(t)[0], (0.225, 0.275)

    def observe(self, t, mode):
        source_time = t
        state, observation_time = "confirmed", t
        # A 0.3s source gap tests stale rejection: the snapshot remains at its
        # original observation epoch instead of masquerading as a fresh coast.
        if self.name == "short_occlusion" and 2.0 < t < 2.6:
            source_time = observation_time = 2.0
        center, velocity = self.current_motion(source_time)
        if mode == "observed_polygon":
            if self.geometry == "near_face_only":
                # Fixed measured face strip; no full hidden object completion.
                anchor = (center[0] - 0.225, center[1])
                geometry = Geometry("polygon", box(0.01, 0.275), source="synthetic_visible_face_strip:uncertified")
            else:
                anchor = center
                geometry = Geometry("polygon", box(0.225, 0.275), source="controlled_full_current_polygon:known_fixture")
        elif mode == "nominal_diameter":
            anchor = (center[0] - 0.225, center[1])
            geometry = Geometry("circle", radius=NOMINAL_DIAMETER, source="2026_conditional_nominal_D:uncertified")
        else:
            raise ValueError("unknown geometry mode")
        return Snapshot(round(source_time * 1e9),
                        (Track(1, anchor, velocity, round(observation_time * 1e9), state, geometry),))


SCENARIOS = (
    Scenario("stationary", "static", speed_bound=0.),
    Scenario("low_crossing", "low", speed_bound=0.25),
    Scenario("high_crossing", "high"),
    Scenario("head_on", "head_on", speed_bound=0.4),
    Scenario("sudden_stop", "stop", speed_bound=0.4),
    Scenario("sudden_reverse", "reverse", speed_bound=0.4),
    Scenario("short_occlusion", "low", speed_bound=0.25),
    Scenario("narrow_crossing", "low", domain=(-1., 7., -0.8, 0.8), speed_bound=0.25),
    Scenario("wait_then_pass", "wait", goal=(4.2, 0., 0.), domain=(-1., 6., -0.7, 0.7), speed_bound=0.3),
    Scenario("road_reopens", "reopen", goal=(4.2, 0., 0.), domain=(-1., 6., -0.7, 0.7), speed_bound=0.3),
    # An additional geometry-risk case and an analytic scene regression. This
    # sine is NOT the PD-driven Gazebo actor or its historical measured trace.
    Scenario("near_face_geometry", "low", geometry="near_face_only", speed_bound=0.25),
    Scenario("analytic_course_sine", "sine", speed_bound=0.9 * 2 * math.pi / 8),
)


def reference(initial, goal, times, cruise=0.6):
    """Straight global reference with re-anchored progression, allowing waiting."""
    position, target = np.asarray(initial[:2]), np.asarray(goal[:2])
    delta = target - position
    distance = np.linalg.norm(delta)
    unit = delta / distance if distance > 1e-12 else np.zeros(2)
    positions = position + np.minimum(times * cruise, distance)[:, None] * unit
    return np.c_[positions, np.full(len(times), goal[2])]
