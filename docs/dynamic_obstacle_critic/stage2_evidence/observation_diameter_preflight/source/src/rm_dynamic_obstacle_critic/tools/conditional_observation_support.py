"""Conditional CV support from the latest real observation, not scan reception.

No runtime policy, ROS dependency, geometry certification or physical default.
The public anchor is q_o + v_hat * coast_time; all uncertainty starts at o.
"""
from fractions import Fraction

from conditional_cv_support import (
    KEYS, finite, outward_float, speed_upper, validate_assumptions,
)


def conditional_observation_radius(document, velocity_xy, observation_coast_sec,
                                   source_age_sec, future_offset_sec):
    if not isinstance(document, dict) or document.get('time_reference') != 'last_observation_epoch':
        raise ValueError('explicit last-observation epoch required')
    if document.get('motion_semantics') != 'complete_projection_directed_Hausdorff_from_observation':
        raise ValueError('complete projection motion from last observation required')
    # Reuse bound/provenance validation, while keeping the old source API intact.
    source_document = dict(document, time_reference='message_source_epoch',
        motion_semantics='complete_projection_directed_Hausdorff_from_source')
    bounds, sources = validate_assumptions(source_document)
    if not isinstance(velocity_xy, (list, tuple)) or len(velocity_xy) != 2:
        raise ValueError('two planar estimated velocity components required')
    velocity = [finite(v) for v in velocity_xy]
    coast, age, future = [finite(v, True) for v in
                         (observation_coast_sec, source_age_sec, future_offset_sec)]
    elapsed = Fraction(coast) + Fraction(age) + Fraction(future)
    required = KEYS if elapsed > 0 else KEYS[:2]
    reasons = []
    for key in required:
        if bounds[key] is None:
            reasons.append('missing_bound:' + key)
        elif sources[key] is None:
            reasons.append('missing_source:' + key)
    if bounds['valid_for_sec'] is not None and elapsed > Fraction(bounds['valid_for_sec']):
        reasons.append('outside_observation_validity_interval')
    result = {'status': 'UNKNOWN', 'radius_m': None, 'reasons': reasons,
              'observation_coast_sec': coast, 'source_age_sec': age,
              'future_offset_sec': future, 'observation_offset_sec': outward_float(elapsed),
              'scope': 'Conditional mathematics only; measurement, association, complete geometry and motion bounds are not certified.'}
    if reasons:
        return result
    radius = Fraction(bounds[KEYS[0]]) + Fraction(bounds[KEYS[1]])
    if elapsed > 0:
        speed = speed_upper(velocity)
        if speed is None:
            result['reasons'].append('numeric_velocity_norm_overflow')
            return result
        radius += (Fraction(bounds[KEYS[2]]) + speed) * elapsed
    rounded = outward_float(radius)
    if rounded is None:
        result['reasons'].append('numeric_radius_overflow')
        return result
    result.update(status='CONDITIONAL_SUPPORT', radius_m=rounded,
                  outward_rounded=True, observation_time_only=elapsed == 0)
    return result
