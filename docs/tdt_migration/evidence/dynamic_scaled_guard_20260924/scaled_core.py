#!/usr/bin/env python3
"""Find the largest same-direction command admitted by the frozen stop rule."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "dynamic_guard_pilot_20260923"))
from guard_core import certificate

BISECTION_STEPS = 16


def scale_command(pose, odom_speed, proposed, body, sweep):
    full = certificate(pose, odom_speed, proposed, body, sweep)
    if full["safe_under_model"]:
        return 1.0, full, full, "admit"
    zero = certificate(pose, odom_speed, (0.0, 0.0, 0.0), body, sweep)
    if not zero["safe_under_model"]:
        return 0.0, zero, full, "reject_unstoppable"
    low, high = 0.0, 1.0
    admitted = zero
    for _ in range(BISECTION_STEPS):
        mid = (low + high) / 2
        trial = certificate(pose, odom_speed,
                            tuple(mid * component for component in proposed),
                            body, sweep)
        if trial["safe_under_model"]:
            low, admitted = mid, trial
        else:
            high = mid
    assert admitted["safe_under_model"]
    return low, admitted, full, "scale_sweep"
