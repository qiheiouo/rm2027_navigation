"""Planar geometry for the existing static Phase 1.5D fixture; no ROS IO."""
import math

# From phase1_omni.sdf and course_wall.sdf at the unchanged launch poses.
OBSTACLES = ((1.225, -0.55, 1.575, 0.55),
             (2.0, 0.4, 4.0, 0.65), (2.0, -0.65, 4.0, -0.4))


def point_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2)) if length2 else 0.0
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def polygon_distance(a, b):
    separated = False
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:] + polygon[:1]):
            normal = (p[1] - q[1], q[0] - p[0])
            pa = [x * normal[0] + y * normal[1] for x, y in a]
            pb = [x * normal[0] + y * normal[1] for x, y in b]
            separated |= max(pa) < min(pb) or max(pb) < min(pa)
    if not separated:
        return 0.0
    return min(point_segment(p, q, r) for points, edges in ((a, b), (b, a))
               for p in points for q, r in zip(edges, edges[1:] + edges[:1]))


def clearance(x, y, yaw, padding=0.0):
    if not all(math.isfinite(v) for v in (x, y, yaw, padding)) or padding < 0:
        raise ValueError("invalid pose or padding")
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = 0.30 + padding, 0.25 + padding
    robot = [(x + c * a - s * b, y + s * a + c * b)
             for a, b in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
    return min(polygon_distance(robot, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
               for x0, y0, x1, y1 in OBSTACLES)


def path_distance(x, y, path):
    if not path:
        return None
    if len(path) == 1:
        return math.hypot(x - path[0][0], y - path[0][1])
    return min(point_segment((x, y), a, b) for a, b in zip(path, path[1:]))
