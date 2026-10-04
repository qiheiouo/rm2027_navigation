"""One bounded lateral preference inside the planner's certified rectangle.

This changes neither static topology nor the feasible set. It uses current
public observation geometry only; the QP and its independent checks own all
temporal feasibility. A side latch is a preference, never a safety verdict.
"""
import numpy as np


class LateralReference:
    def __init__(self):
        self.reset()

    def reset(self):
        self.plan = self.track = self.normal = self.side = None
        self.mode = 'route'

    def apply(self, initial, window, timeline):
        x = np.asarray(initial)
        route = window.reference[:, :2]
        if self.plan != window.plan_id:
            self.reset()
            self.plan = window.plan_id
        tangent = route[-1] - route[0]
        length = np.linalg.norm(tangent)
        if length < 1e-5:
            self.reset()
            return route.copy()
        tangent /= length
        normal = np.array([-tangent[1], tangent[0]])
        b = window.centre_bounds
        # A goal must actually lie inside the certified rectangle, not merely
        # within its projection. No raw-map search is performed here.
        reserve = .025*np.hypot(.8,.5)
        def inside(p):
            return b[0]+reserve <= p[0] <= b[1]-reserve and b[2]+reserve <= p[1] <= b[3]-reserve
        obstacles = []
        for i, (centers, shape) in enumerate(zip(timeline.centers,timeline.geometries)):
            center = centers[0]
            radius = shape.radius if shape.kind == 'circle' else np.max(np.linalg.norm(shape.offsets,axis=1))
            # Circumscribed padded footprint here is more conservative than the
            # exact rectangle checks. Extra .12 m is a preference buffer only.
            reach = radius + np.hypot(.355,.330) + .02 + reserve + .12
            forward = (center-x[:2])@tangent
            lateral = (center-route[0])@normal
            # Inspect current geometry against the local route ray early,
            # without forecasting beyond the 1.5 s certified temporal horizon.
            if -reach < forward < 6. and abs(lateral) < reach:
                obstacles.append((forward,i,reach))
        by_id = {timeline.track_ids[i]:(f,i,r) for f,i,r in obstacles}
        selected = by_id.get(self.track)
        if selected is None:
            self.track = self.normal = self.side = None
            self.mode = 'route'
            if not obstacles:
                return route.copy()
            selected = min(obstacles, key=lambda a:a[0])
        forward,i,reach = selected
        center = timeline.centers[i][0]
        # Do not preserve an incompatible side across a planner corner.
        if self.normal is not None and self.normal@normal < .95:
            self.track = self.normal = self.side = None
        current_level = x[:2]@normal
        center_level = center@normal
        options = []
        for sign in (1.,-1.):
            level = center_level + sign*reach
            target = x[:2] + normal*(level-current_level)
            if inside(target):
                options.append((abs(level-current_level),sign,level))
        if self.side is not None:
            options = [o for o in options if o[1] == self.side]
        if not options:
            self.mode = 'corridor_insufficient'
            return route.copy()
        _,sign,level = min(options,key=lambda o:(o[0],-o[1]))
        self.track,self.normal,self.side = timeline.track_ids[i],normal.copy(),sign
        # Until sufficient sideways separation, hold longitudinal progress.
        # The target can exceed one-cycle reach; velocity/terminal stop remain
        # hard QP constraints and each proposal is fully checked.
        separated = sign*(current_level-center_level) >= reach-.08
        self.mode = 'pass' if separated else 'shift'
        reference = route.copy() if separated else np.tile(x[:2],(len(route),1))
        reference += (level-reference@normal)[:,None]*normal
        return np.clip(reference,[b[0]+reserve,b[2]+reserve],[b[1]-reserve,b[3]-reserve])
