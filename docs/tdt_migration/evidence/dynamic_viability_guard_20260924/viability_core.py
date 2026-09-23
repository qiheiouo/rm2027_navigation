#!/usr/bin/env python3
"""Diagnostic next-decision stop viability for the known fixture sweep."""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent/'dynamic_guard_pilot_20260923'))
from guard_core import BODY_GATE_M, HOLD_S, certificate, padded, radius

STEPS = 16
NEXT_DECISION_S = 0.12  # Existing guard watchdog threshold; diagnostic, not WCET.
MAX_ODOM_AGE_S = 0.10  # Existing guard stale-input cutoff.
NEXT_RESPONSE_S = HOLD_S + 0.05 + MAX_ODOM_AGE_S


def viability(pose, odom_speed, command, body, sweep, age_s):
    if not math.isfinite(age_s) or not -0.02 <= age_s <= MAX_ODOM_AGE_S:
        raise ValueError('stale odometry')
    current = certificate(pose, odom_speed, command, body, sweep,
                          hold_s=HOLD_S + 0.05 + max(0, age_s))
    v = max(math.hypot(*odom_speed[:2]), math.hypot(*command[:2]))
    w = max(abs(odom_speed[2]), abs(command[2]))
    # No deceleration credit in the interval. The worst next speed is v/w.
    # Both the robot motion and the subsequently required stop subtract from
    # the *current* full-sweep polygon distance.
    translation = v * NEXT_DECISION_S + v * NEXT_RESPONSE_S + v*v/2
    rotation = w * NEXT_DECISION_S + w * NEXT_RESPONSE_S + w*w/4
    rb = radius(body)
    rp = radius(padded(body, .03))
    body_next = current['body_gap_m'] - translation - rb*rotation
    padded_next = current['padded_gap_m'] - translation - rp*rotation
    return {
        'safe_under_model': current['safe_under_model'] and
                            body_next >= BODY_GATE_M and padded_next > 0,
        'current': current, 'next_body_lower_m': body_next,
        'next_padded_lower_m': padded_next,
        'next_interval_s': NEXT_DECISION_S,
        'next_response_s': NEXT_RESPONSE_S,
        'no_deceleration_credit_in_interval': True,
    }


def scale_command(pose, speed, proposed, body, sweep, age_s):
    full = viability(pose,speed,proposed,body,sweep,age_s)
    if full['safe_under_model']:
        return 1.,full,full,'admit'
    zero = viability(pose,speed,(0.,0.,0.),body,sweep,age_s)
    if not zero['safe_under_model']:
        return 0.,zero,full,'reject_next_unstoppable'
    lo,hi,selected=0.,1.,zero
    for _ in range(STEPS):
        mid=(lo+hi)/2
        trial=viability(pose,speed,tuple(mid*x for x in proposed),body,sweep,age_s)
        if trial['safe_under_model']:
            lo,selected=mid,trial
        else:
            hi=mid
    assert selected['safe_under_model']
    return lo,selected,full,'scale_viability'
