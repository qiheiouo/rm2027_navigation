#!/usr/bin/env python3
"""Conservative full-fixture-sweep stopping test for one experimental command."""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "sim_stop_probe_20260923"))
from sweep_shadow import fixture_sweep, padded, placed, polygon_distance, radius


BODY_GATE_M = 0.05
HOLD_S = 0.15  # one 20 Hz smoother period plus a 0.1 s diagnostic response allowance
LINEAR_DECEL_M_S2 = 1.0
YAW_DECEL_RAD_S2 = 2.0


def certificate(pose, odom_speed, proposed, body, sweep,
                hold_s=HOLD_S, linear_decel=LINEAR_DECEL_M_S2,
                yaw_decel=YAW_DECEL_RAD_S2):
    values = (*pose, *odom_speed, *proposed, hold_s, linear_decel, yaw_decel)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("nonfinite safety input")
    if hold_s < 0 or linear_decel <= 0 or yaw_decel <= 0:
        raise ValueError("invalid response model")
    # This is deliberately direction-independent: any command within these
    # speed magnitudes is assumed able to head toward the obstacle. The
    # bounds apply only if the experimental response assumptions hold.
    linear_speed = max(math.hypot(*odom_speed[:2]), math.hypot(*proposed[:2]))
    yaw_speed = max(abs(odom_speed[2]), abs(proposed[2]))
    displacement = linear_speed * hold_s + linear_speed ** 2 / (2 * linear_decel)
    angle = yaw_speed * hold_s + yaw_speed ** 2 / (2 * yaw_decel)
    body_gap = polygon_distance(placed(body, pose), sweep)
    padding = padded(body, 0.03)
    padded_gap = polygon_distance(placed(padding, pose), sweep)
    body_lower = body_gap - displacement - radius(body) * angle
    padded_lower = padded_gap - displacement - radius(padding) * angle
    return {
        "safe_under_model": body_lower >= BODY_GATE_M and padded_lower > 0,
        "body_gap_m": body_gap, "padded_gap_m": padded_gap,
        "body_lower_m": body_lower, "padded_lower_m": padded_lower,
        "linear_speed_bound_m_s": linear_speed, "yaw_speed_bound_rad_s": yaw_speed,
        "translation_reach_m": displacement, "rotation_reach_rad": angle,
    }
