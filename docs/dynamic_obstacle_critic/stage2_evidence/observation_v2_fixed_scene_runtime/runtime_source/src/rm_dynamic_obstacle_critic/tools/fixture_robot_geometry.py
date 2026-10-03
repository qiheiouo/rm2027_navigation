"""Frozen self-plant geometry for independent offline audits, never ROS input."""
import math
import xml.etree.ElementTree as ET


def base_body_polygon(scene):
    collision = ET.parse(scene / 'phase1_omni.sdf').getroot().find(
        "world/model[@name='rm_sentry_2027']/link[@name='base_link']/collision[@name='base_collision']")
    if collision is None or collision.find('geometry/box') is None:
        raise ValueError('frozen fixture base box missing')
    pose = collision.find('pose')
    if pose is not None and pose.attrib.get('relative_to') not in (None, 'base_link'):
        raise ValueError('base collision must be expressed in base_link')
    values = [float(v) for v in (collision.findtext('pose') or '0 0 0 0 0 0').split()]
    size = [float(v) for v in collision.findtext('geometry/box/size').split()]
    if (len(values) != 6 or len(size) != 3
            or not all(math.isfinite(v) for v in values + size)
            or any(v <= 0 for v in size) or abs(values[3]) > 1e-12 or abs(values[4]) > 1e-12):
        raise ValueError('unsupported base collision geometry')
    x, y, _ = size; c, s = math.cos(values[5]), math.sin(values[5])
    return [(values[0] + c * a - s * b, values[1] + s * a + c * b)
            # Preserve the legacy fixture's clockwise traversal, including its
            # floating-point distance evaluation order in historical reports.
            for a, b in ((-x/2, -y/2), (-x/2, y/2), (x/2, y/2), (x/2, -y/2))]


def physical_base_body(root, configured):
    """Legacy captures without scene snapshots keep their documented fallback."""
    scene = root / 'scene_inputs'
    return base_body_polygon(scene) if (scene / 'phase1_omni.sdf').is_file() else configured
