#!/usr/bin/env python3
"""Offline-only directional stopping envelope for the known fixture sweep.

The command/odom velocity component box is checked with interval error bounds.
This remains an assumed simulation response model, not a hardware certificate.
"""
import itertools
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "dynamic_safety_contract_20260923"))
from braking_replay import braking_poses, placed, padded, polygon_distance, radius

BODY_GATE_M = 0.05
HOLD_S = 0.15
LINEAR_DECEL_M_S2 = 1.0
YAW_DECEL_RAD_S2 = 2.0
STEP_S = 0.04


def certificate(pose, odom_speed, proposed, body, sweep, divisions=2):
    values = (*pose, *odom_speed, *proposed)
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError("nonfinite safety input")
    if divisions not in (2, 4):
        raise ValueError("unsupported interval subdivision")
    maximum = [max(abs(odom_speed[i]), abs(proposed[i])) for i in range(3)]
    max_v = math.hypot(maximum[0], maximum[1])
    max_w = maximum[2]
    horizon = STEP_S * math.ceil((HOLD_S + max(
        maximum[0] / LINEAR_DECEL_M_S2,
        maximum[1] / LINEAR_DECEL_M_S2,
        maximum[2] / YAW_DECEL_RAD_S2) + STEP_S) / STEP_S)
    # Each axis can respond independently between measured and requested
    # velocity. Midpoint trajectories cover the 3D speed box; Lipschitz terms
    # subtract the full unsampled interval and time-integration uncertainty.
    half_axis = [abs(proposed[i] - odom_speed[i]) / (2 * divisions)
                 for i in range(3)]
    delta_v = math.hypot(*half_axis[:2])
    delta_w = half_axis[2]
    shapes = (("body", body, BODY_GATE_M), ("padded", padded(body, 0.03), 0.0))
    minimum = {name: math.inf for name, _, _ in shapes}
    trajectory_count = 0
    for indices in itertools.product(range(divisions), repeat=3):
        speed = tuple(odom_speed[i] + (proposed[i] - odom_speed[i]) *
                      (indices[i] + 0.5) / divisions for i in range(3))
        poses = braking_poses(pose, speed, HOLD_S, horizon=horizon, step=STEP_S)
        trajectory_count += 1
        for index, robot in enumerate(poses):
            t = index * STEP_S
            for name, poly, gate in shapes:
                r = radius(poly)
                box_error = t * delta_v + 0.5 * max_v * t * t * delta_w + r * t * delta_w
                between_samples = STEP_S / 2 * (max_v + r * max_w)
                integration_error = t * STEP_S / 2 * (
                    math.sqrt(2) * LINEAR_DECEL_M_S2 + max_v * max_w +
                    r * YAW_DECEL_RAD_S2)
                gap = polygon_distance(placed(poly, robot), sweep)
                lower = gap - box_error - between_samples - integration_error
                minimum[name] = min(minimum[name], lower)
                if lower < gate or (gate == 0 and lower <= 0):
                    return {"safe_under_model": False,
                            "body_lower_m": minimum["body"],
                            "padded_lower_m": (minimum["padded"] if math.isfinite(minimum["padded"])
                                               else None),
                            "divisions": divisions, "trajectories_checked": trajectory_count,
                            "horizon_s": horizon, "failed_shape": name,
                            "failed_time_s": t}
    return {"safe_under_model": True,
            "body_lower_m": minimum["body"],
            "padded_lower_m": minimum["padded"],
            "divisions": divisions, "trajectories_checked": trajectory_count,
            "horizon_s": horizon}
