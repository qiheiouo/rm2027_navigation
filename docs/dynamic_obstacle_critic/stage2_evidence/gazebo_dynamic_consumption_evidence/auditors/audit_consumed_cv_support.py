#!/usr/bin/env python3
"""Offline full actor support using the exact CV fields/clock consumed in score."""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics
from analyze_trial import compose, pose, rotation
from audit_scan_geometry import fixture
from dynamic_consumption_io import read_consumption
from trial_io import rows


def audit(root):
    truth = []
    for msg in rows(root, 'gazebo_poses.jsonl'):
        by = {p['name']: p for p in msg.get('pose', []) if p.get('name') in ('moving_obstacle', 'obstacle_link')}
        if len(by) != 2: continue
        stamp = msg['header']['stamp']; t = float(stamp.get('sec', 0))+float(stamp.get('nsec', 0))*1e-9
        if truth and t <= truth[-1][0]: continue
        truth.append((t, compose(pose(by['moving_obstacle']), pose(by['obstacle_link']))))
    times = [t for t, _ in truth]
    def actual(t):
        if not times or t < times[0] or t > times[-1]: return None
        j = bisect.bisect_right(times, t)
        if j == len(times): return truth[-1][1]
        a, b = truth[j-1], truth[j]; fraction = (t-a[0])/(b[0]-a[0])
        yaw = a[1][2]+fraction*math.remainder(b[1][2]-a[1][2], 2*math.pi)
        return (*(x+fraction*(y-x) for x, y in zip(a[1][:2], b[1][:2])), yaw)
    dimensions = fixture(root/'scene_inputs')['actor_dimensions']; x, y = dimensions[:2]
    polygon = [(-x/2, -y/2), (x/2, -y/2), (x/2, y/2), (-x/2, y/2)]
    records = []
    for path in sorted((root/'dynamic_scores').glob('score_*.json'), key=lambda p: int(p.stem.split('_')[-1])):
        meta, _ = read_consumption(path)
        if meta['input_used']['frame'] != 'map' or meta['evaluation_frame'] != 'odom':
            raise ValueError('this fixture support audit requires map-to-odom world alignment')
        tx, ty, angle = meta['world_transform']; c, s = math.cos(angle), math.sin(angle)
        def align(point): return tx+c*point[0]-s*point[1], ty+s*point[0]+c*point[1]
        now = meta['score_stamp']; at_now = actual(now)
        if at_now is None: raise ValueError('exact consumer clock lacks actor truth')
        usable = [t for t in meta['input_used']['tracks'] if t['state'] in (2, 3)]
        age = meta['source_age_used']
        def predict(t, future): return [t['xy'][i]+t['vxy'][i]*(age+future) for i in (0, 1)]
        # Same fixed 1m offline actor association as the registered receipt audit.
        track = min(usable, key=lambda t: math.dist(predict(t, 0), at_now[:2])) if usable else None
        if track is not None and math.dist(predict(track, 0), at_now[:2]) > 1.: track = None
        row = {'ordinal': meta['ordinal'], 'score_stamp': now, 'source_age_used': age,
            'usable_actor_track': track is not None, 'track_id': track['id'] if track else None, 'horizons': {}}
        if track:
            radius = max(meta['limits']['minimum_radius'], .5*math.hypot(*track['size_xy'])); row['used_radius'] = radius
            for future in (0., 1., 2., 3.):
                physical = actual(now+future)
                if physical is None: raise ValueError('full three-second consumer support lacks truth')
                center = align(predict(track, future)); corners = [align(p) for p in rotation(physical, polygon)]
                maximum = max(math.dist(center, corner) for corner in corners)
                row['horizons'][str(future)] = {'full_physical_box_covered': maximum <= radius,
                    'center_error': math.dist(center, align(physical[:2])), 'support_deficit': max(0., maximum-radius)}
        records.append(row)
    usable = [r for r in records if r['usable_actor_track']]; support = {}
    for future in ('0.0', '1.0', '2.0', '3.0'):
        samples = [r['horizons'][future] for r in usable]
        support[future] = {'scores': len(samples), 'full_physical_box_covered': sum(r['full_physical_box_covered'] for r in samples),
            'center_error_median': statistics.median(r['center_error'] for r in samples) if samples else None,
            'center_error_max': max((r['center_error'] for r in samples), default=None),
            'support_deficit_max': max((r['support_deficit'] for r in samples), default=None)}
    return {'schema': 1, 'scope': 'Exact consumed public fields, age and score clock; actor truth only offline in the fixed Gazebo map/world alignment.',
        'scores': len(records), 'usable_actor_scores': len(usable), 'scores_without_usable_actor': len(records)-len(usable),
        'CV_support': support, 'records': records,
        'limits': 'Full physical actor projection and linear truth interpolation are fixture labels. Dimension rules do not bound CV turn error; no online geometry or physical safety certificate.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); report = audit(args.trial)
    with args.output.open('x') as stream: stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}))
