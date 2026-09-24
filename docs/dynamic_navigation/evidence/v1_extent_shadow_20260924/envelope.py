"""Reference V1 fixture occupancy from a visible scan cluster and CV state.

This is an experiment model, not a certified bound on arbitrary moving objects.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AxisBox:
    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def contains(self, center, size, tolerance=1e-9):
        return (self.min_x <= center[0] - size[0] / 2 + tolerance and
                self.max_x >= center[0] + size[0] / 2 - tolerance and
                self.min_y <= center[1] - size[1] / 2 + tolerance and
                self.max_y >= center[1] + size[1] / 2 - tolerance)

    def polygon(self):
        return ((self.min_x, self.min_y), (self.max_x, self.min_y),
                (self.max_x, self.max_y), (self.min_x, self.max_y))


def predicted_box(center, velocity, visible_size, object_extent, age, horizon,
                  reference_acceleration):
    """Dilate a visible cluster by the *full* object dimensions on each side.

    At source time a visible return can lie anywhere on/in the object. A full
    known extent, rather than half an extent around the visible centroid, is
    required to include its hidden side. The optional CV mismatch term uses
    the fixture's sinusoidal target acceleration; it is not a hard actuator
    acceleration limit. No Gazebo truth pose is an input to this function.
    """
    values = (*center, *velocity, *visible_size, *object_extent, age, horizon,
              reference_acceleration)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite prediction input')
    if any(value < 0 for value in (*visible_size, age, horizon,
                                   reference_acceleration)) or any(
            value <= 0 for value in object_extent):
        raise ValueError('invalid extent or time')
    duration = age + horizon
    mismatch = .5 * reference_acceleration * duration * duration
    predicted = [center[i] + velocity[i] * duration for i in range(2)]
    half = [visible_size[i] / 2 + object_extent[i] + mismatch for i in range(2)]
    return AxisBox(predicted[0] - half[0], predicted[1] - half[1],
                   predicted[0] + half[0], predicted[1] + half[1])


def fixture_target_acceleration(amplitude=.9, period=8.0):
    if not math.isfinite(amplitude) or not math.isfinite(period) or amplitude < 0 or period <= 0:
        raise ValueError('invalid sinusoidal target')
    return amplitude * (2 * math.pi / period) ** 2
