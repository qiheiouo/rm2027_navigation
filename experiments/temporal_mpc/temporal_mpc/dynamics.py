"""Four-omni body-velocity model integrated on the collision-check grid."""
import numpy as np


def rollout(initial, controls, dt):
    initial = np.asarray(initial, float)
    controls = np.asarray(controls, float)
    if (initial.shape != (6,) or controls.ndim != 2 or controls.shape[1] != 3
            or not np.isfinite(initial).all() or not np.isfinite(controls).all()
            or not np.isfinite(dt) or dt <= 0):
        raise ValueError("invalid dynamics inputs")
    velocity = initial[3:] + np.vstack((np.zeros(3), np.cumsum(controls * dt, axis=0)))
    yaw_increment = (velocity[:-1, 2] + 0.5 * controls[:, 2] * dt) * dt
    yaw = initial[2] + np.r_[0.0, np.cumsum(yaw_increment)]
    mid_yaw = yaw[:-1] + 0.5 * dt * velocity[:-1, 2] + 0.125 * dt**2 * controls[:, 2]
    mid_v = velocity[:-1, :2] + 0.5 * controls[:, :2] * dt
    c, s = np.cos(mid_yaw), np.sin(mid_yaw)
    delta = dt * np.c_[c * mid_v[:, 0] - s * mid_v[:, 1], s * mid_v[:, 0] + c * mid_v[:, 1]]
    xy = initial[:2] + np.vstack((np.zeros(2), np.cumsum(delta, axis=0)))
    return np.c_[xy, yaw, velocity]


def braking(initial, steps, dt, limits=(1.0, 1.0, 2.0)):
    velocity = np.asarray(initial, float)[3:].copy()
    result = []
    for _ in range(steps):
        acceleration = np.clip(-velocity / dt, -np.asarray(limits), limits)
        result.append(acceleration)
        velocity += acceleration * dt
    return np.asarray(result)


def rollout_zoh(initial, controls, dt):
    """Fixed-yaw velocity targets held for each execution tick.

    Controls bound target differences, not an asserted physical acceleration.
    Actual actuator transient error must be measured separately.
    """
    initial = np.asarray(initial, float)
    controls = np.asarray(controls, float)
    if (initial.shape != (6,) or controls.ndim != 2 or controls.shape[1] != 3
            or not np.isfinite(initial).all() or not np.isfinite(controls).all()
            or not np.isfinite(dt) or dt <= 0 or abs(initial[5]) > 1e-8
            or np.any(controls[:, 2] != 0)):
        raise ValueError("invalid fixed-yaw ZOH inputs")
    velocity = initial[3:] + np.vstack((np.zeros(3), np.cumsum(controls * dt, axis=0)))
    c, s = np.cos(initial[2]), np.sin(initial[2])
    held = velocity[1:, :2]
    delta = dt * np.c_[c * held[:, 0] - s * held[:, 1], s * held[:, 0] + c * held[:, 1]]
    xy = initial[:2] + np.vstack((np.zeros(2), np.cumsum(delta, axis=0)))
    return np.c_[xy, np.full(len(velocity), initial[2]), velocity]
