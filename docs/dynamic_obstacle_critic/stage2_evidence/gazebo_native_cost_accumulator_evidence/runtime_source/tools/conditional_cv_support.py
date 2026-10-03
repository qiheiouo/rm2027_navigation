"""Pure conditional support mathematics; no ROS data type or runtime policy.

For source set diameter D, anchor-to-convex-hull error e, and directed
Hausdorff displacement of the complete future projection at most B*h,
the original CV anchor has radius D+e+(B+norm(v_hat))*h. Caller-supplied
conditions remain assumptions; this function never certifies their truth.
"""
from fractions import Fraction
import math


KEYS = ('initial_diameter_m', 'anchor_to_convex_hull_error_m',
        'projection_hausdorff_speed_mps', 'valid_for_sec')


def finite(value, nonnegative=False):
    if type(value) not in (float, int):
        raise ValueError('finite numeric value with required sign expected')
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError('numeric input overflow') from error
    if not math.isfinite(value) or (nonnegative and value<0):
        raise ValueError('finite numeric value with required sign expected')
    return value


def validate_assumptions(document):
    if (not isinstance(document,dict) or type(document.get('schema')) is not int or
            document.get('schema')!=1 or document.get('units')!='m_s' or
            document.get('time_reference')!='message_source_epoch' or
            document.get('motion_semantics')!='complete_projection_directed_Hausdorff_from_source'):
        raise ValueError('explicit source-time complete-projection assumptions required')
    bounds, sources = document['bounds'], document['condition_sources']
    if not isinstance(bounds,dict) or not isinstance(sources,dict) or set(bounds)!=set(KEYS) or set(sources)!=set(KEYS):
        raise ValueError('four explicit bound/source slots required')
    for key in KEYS:
        if bounds[key] is not None:
            value = finite(bounds[key], nonnegative=True)
            if key=='initial_diameter_m' and value==0:
                raise ValueError('positive initial diameter required')
        if sources[key] is not None and (not isinstance(sources[key], str) or not sources[key].strip()):
            raise ValueError('condition source identifier must be nonblank or null')
    return bounds, sources


def outward_float(exact):
    """Least available upper float around an exact nonnegative rational."""
    try:
        value = float(exact)
    except OverflowError:
        return None
    if not math.isfinite(value):
        return None
    if Fraction(value)<exact:
        value = math.nextafter(value, math.inf)
    return value if math.isfinite(value) else None


def speed_upper(velocity):
    # Verify the rounded square against exact component squares, rather than
    # relying on a particular libm rounding guarantee for hypot.
    squared = sum((Fraction(v)**2 for v in velocity), Fraction())
    speed = math.hypot(*velocity)
    while math.isfinite(speed) and Fraction(speed)**2<squared:
        speed = math.nextafter(speed, math.inf)
    return Fraction(speed) if math.isfinite(speed) else None


def conditional_radius(document, velocity_xy, source_age_sec, future_offset_sec):
    bounds, sources = validate_assumptions(document)
    if not isinstance(velocity_xy, (list, tuple)) or len(velocity_xy)!=2:
        raise ValueError('two planar observed velocity components required')
    velocity = [finite(v) for v in velocity_xy]
    age, future = finite(source_age_sec, True), finite(future_offset_sec, True)
    offset = Fraction(age)+Fraction(future)
    required = KEYS if offset>0 else KEYS[:2]
    reasons = []
    for key in required:
        if bounds[key] is None:
            reasons.append('missing_bound:'+key)
        elif sources[key] is None:
            reasons.append('missing_source:'+key)
    if bounds['valid_for_sec'] is not None and offset>Fraction(bounds['valid_for_sec']):
        reasons.append('outside_source_validity_interval')
    result = {'status':'UNKNOWN', 'radius_m':None, 'reasons':reasons,
              'source_age_sec':age, 'future_offset_sec':future,
              'source_offset_sec':outward_float(offset),
              'scope':'Conditional mathematics only; caller-supplied bounds are not verified physical evidence.'}
    if reasons:
        return result
    radius = Fraction(bounds[KEYS[0]])+Fraction(bounds[KEYS[1]])
    if offset>0:
        speed = speed_upper(velocity)
        if speed is None:
            result['reasons'].append('numeric_velocity_norm_overflow')
            return result
        radius += (Fraction(bounds[KEYS[2]])+speed)*offset
    rounded = outward_float(radius)
    if rounded is None:
        result['reasons'].append('numeric_radius_overflow')
        return result
    result.update(status='CONDITIONAL_SUPPORT', radius_m=rounded,
                  outward_rounded=True, source_time_only=offset==0)
    return result
