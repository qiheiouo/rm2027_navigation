"""Independent scalar physical oracle and ideal plant. No controller geometry imports."""
import math


def rectangle(x, y, yaw, hx, hy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]


def point_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def polygon_distance(a, b):
    separated = False
    for p in (a, b):
        for first, second in zip(p, p[1:] + p[:1]):
            nx, ny = first[1] - second[1], second[0] - first[0]
            pa = [x * nx + y * ny for x, y in a]
            pb = [x * nx + y * ny for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                separated = True
    if not separated:
        return 0.0
    return min(point_segment(p, first, second)
               for points, edges in ((a, b), (b, a)) for p in points
               for first, second in zip(edges, edges[1:] + edges[:1]))


def step_and_audit(initial, acceleration, start_time, duration, obstacle_at,
                   domain, obstacle_speed_bound, audit_dt=0.01):
    """Integrate actual acceleration and audit dense sweeps with a Lipschitz reserve.

    obstacle_at returns ONLY truth labels to this oracle, never future input to
    the controller. The reserve also covers endpoints at abrupt velocity changes
    when the fixture's declared global translation-speed bound remains valid.
    """
    steps = math.ceil(duration / audit_dt)
    h = duration / steps
    state = list(map(float, initial))
    clearance_min, sampled_min, collision = math.inf, math.inf, False
    for k in range(steps + 1):
        x, y, yaw, vx, vy, wz = state
        robot = rectangle(x, y, yaw, 0.325, 0.300)
        center, half_extents = obstacle_at(start_time + k * h)
        obstacle = rectangle(center[0], center[1], 0.0, *half_extents)
        actual = polygon_distance(robot, obstacle)
        xmin, xmax, ymin, ymax = domain
        walls = min(min(px - xmin, xmax - px, py - ymin, ymax - py) for px, py in robot)
        sampled = min(actual, walls)
        collision = collision or sampled <= 1e-10
        sampled_min = min(sampled_min, sampled)
        # Global interval speed bounds for both physical projections.
        speed = math.hypot(vx, vy) + h * math.hypot(acceleration[0], acceleration[1])
        point_speed = speed + math.hypot(0.325, 0.300) * (abs(wz) + h * abs(acceleration[2]))
        clearance_min = min(clearance_min, sampled - 0.5 * h * (point_speed + obstacle_speed_bound))
        if k == steps:
            break
        mid_yaw = yaw + 0.5 * h * wz + 0.125 * h * h * acceleration[2]
        mid_vx, mid_vy = vx + 0.5 * h * acceleration[0], vy + 0.5 * h * acceleration[1]
        state[0] += h * (math.cos(mid_yaw) * mid_vx - math.sin(mid_yaw) * mid_vy)
        state[1] += h * (math.sin(mid_yaw) * mid_vx + math.cos(mid_yaw) * mid_vy)
        state[2] += h * wz + 0.5 * h * h * acceleration[2]
        state[3] += h * acceleration[0]
        state[4] += h * acceleration[1]
        state[5] += h * acceleration[2]
    return state, float(clearance_min), float(sampled_min), bool(collision)
