#!/usr/bin/env python3
"""Scale commands using the unchanged stop rule plus explicit data/control age."""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "dynamic_guard_pilot_20260923"))
from guard_core import HOLD_S, certificate

BISECTION_STEPS = 16


def response_horizon(odom_age_s, smoother_frequency_hz):
    if not math.isfinite(odom_age_s) or not math.isfinite(smoother_frequency_hz):
        raise ValueError("nonfinite command timing")
    if odom_age_s < -0.02 or odom_age_s > 0.1 or smoother_frequency_hz <= 0:
        raise ValueError("invalid command timing")
    # HOLD_S includes one smoother period and a diagnostic response allowance.
    # Reserve an additional period for the newly released command.
    return HOLD_S + 1.0 / smoother_frequency_hz + max(0.0, odom_age_s)


def scale_command(pose, odom_speed, proposed, body, sweep,
                  odom_age_s, smoother_frequency_hz):
    hold = response_horizon(odom_age_s, smoother_frequency_hz)

    def check(command):
        return certificate(pose, odom_speed, command, body, sweep, hold_s=hold)

    full = check(proposed)
    if full["safe_under_model"]:
        return 1.0, full, full, "admit", hold
    zero = check((0.0, 0.0, 0.0))
    if not zero["safe_under_model"]:
        return 0.0, zero, full, "reject_unstoppable", hold
    low, high, selected = 0.0, 1.0, zero
    for _ in range(BISECTION_STEPS):
        mid = (low + high) / 2
        trial = check(tuple(mid * component for component in proposed))
        if trial["safe_under_model"]:
            low, selected = mid, trial
        else:
            high = mid
    assert selected["safe_under_model"]
    return low, selected, full, "scale_sweep", hold
