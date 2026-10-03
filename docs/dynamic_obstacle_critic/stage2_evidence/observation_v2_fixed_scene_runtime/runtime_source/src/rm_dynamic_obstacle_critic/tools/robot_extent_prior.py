"""Pure offline geometry conditions; no ROS tracker type or runtime policy.

For a set of diameter D and a measured anchor within e of its convex hull,
all set points are within D+e. This does not supply the error bound itself.
"""
import math


def number(value, positive=False):
    if type(value) not in (float, int) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError('finite nonnegative geometric bound required')
    return float(value)


def alternative_diameter(alternative):
    if set(alternative) == {'box_xy'}:
        dimensions = alternative['box_xy']
        if not isinstance(dimensions, list) or len(dimensions) != 2: raise ValueError('two box dimensions required')
        diameter=math.hypot(*(number(value, positive=True) for value in dimensions))
        if not math.isfinite(diameter):raise ValueError('geometric diameter overflow')
        return diameter
    if set(alternative) == {'sphere_diameter'}:
        return number(alternative['sphere_diameter'], positive=True)
    raise ValueError('unsupported or ambiguous geometric rule')


def class_diameters(config):
    if config.get('schema') != 1 or config.get('units') != 'm': raise ValueError('extent reference schema/units')
    scope = config['scope']; names = scope['target_classes']
    if (not isinstance(names, list) or not names or len(names) > 32 or any(not isinstance(n, str) for n in names)
            or len(set(names)) != len(names) or set(config['classes']) != set(names)
            or scope['unknown_class_policy'] != 'maximum_only_within_listed_ground_classes'):
        raise ValueError('explicit bounded target class scope required')
    result = {}
    for name in names:
        alternatives = config['classes'][name]['alternatives']
        if not isinstance(alternatives, list) or not 1 <= len(alternatives) <= 8: raise ValueError('alternative rule budget')
        result[name] = max(alternative_diameter(a) for a in alternatives)
    return result


def anchor_envelope_radius(config, target_class, anchor_error_bound):
    """Requires an explicit error-to-convex-hull bound; never assumes a center."""
    error = number(anchor_error_bound)
    diameters = class_diameters(config)
    if target_class == 'unknown_within_listed_ground_classes': diameter = max(diameters.values())
    elif target_class in diameters: diameter = diameters[target_class]
    else: raise ValueError('target outside reference scope')
    radius = diameter+error
    if not math.isfinite(radius): raise ValueError('extent radius overflow')
    return radius
