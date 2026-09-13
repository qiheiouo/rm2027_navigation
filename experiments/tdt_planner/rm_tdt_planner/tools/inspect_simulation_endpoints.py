#!/usr/bin/env python3
"""Inspect published historical costmaps around endpoint refusals; never run a planner."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def endpoint(grid, x, y, radius, clearance):
    r = grid['resolution']
    ox, oy = grid['origin']
    w, h = grid['width'], grid['height']
    ix, iy = math.floor((x-ox)/r), math.floor((y-oy)/r)
    if not (0 <= ix < w and 0 <= iy < h):
        return {'xy': [x, y], 'cell': [ix, iy], 'inside': False}
    # OccupancyGrid encoding: 100=lethal, -1=unknown, 99=inscribed inflation.
    hard = min(ix, iy, w-1-ix, h-1-iy)*r  # retained outer border seeds
    inscribed = math.inf
    for i, cost in enumerate(grid['data']):
        if cost not in (-1, 99, 100):
            continue
        d = math.hypot(ix-i % w, iy-i // w)*r
        if cost == 99:
            inscribed = min(inscribed, d)
        else:
            hard = min(hard, d)
    limit = radius + clearance + math.sqrt(2)*r + 1e-6
    cell_cost = grid['data'][iy*w+ix]
    return {'xy': [x, y], 'cell': [ix, iy], 'inside': True, 'occupancy_value': cell_cost,
            'nearest_hard_or_border_cell_centre_m': hard,
            'nearest_inscribed_cell_centre_m': inscribed if math.isfinite(inscribed) else None,
            'reserved_cell_centre_distance_m': limit,
            'legacy_seed_model_free': cell_cost not in (-1, 99, 100) and min(hard, inscribed) > limit,
            'nav2_master_seed_model_free': cell_cost not in (-1, 99, 100) and hard > limit}


def inspect(observation, radius, clearance):
    names = ('summary.json', 'costmap.jsonl', 'events.jsonl', 'trajectory.jsonl')
    content = {name: (observation/name).read_bytes() for name in names}
    summary = json.loads(content['summary.json'])
    maps = [json.loads(line) for line in content['costmap.jsonl'].splitlines()]
    events = [json.loads(line) for line in content['events.jsonl'].splitlines()]
    trajectory = [json.loads(line) for line in content['trajectory.jsonl'].splitlines()]
    if not maps or not trajectory:
        raise ValueError('missing costmap or trajectory evidence')
    for grid in maps:
        w, h, r = grid['width'], grid['height'], grid['resolution']
        if grid['frame'] != 'map' or w < 3 or h < 3 or len(grid['data']) != w*h or r <= 0:
            raise ValueError('invalid published map geometry')
        if not all(math.isfinite(x) for x in [r, grid['t'], *grid['origin']]):
            raise ValueError('nonfinite published map geometry')
        if any(type(c) is not int or not (-1 <= c <= 100) for c in grid['data']):
            raise ValueError('invalid signed OccupancyGrid values')
    if any(b['t'] < a['t'] for a, b in zip(maps, maps[1:])):
        raise ValueError('published map timestamps reversed')
    goal_x, goal_y = summary['goal'][:2]
    rows = []
    for event in events:
        if 'start or goal outside' not in event.get('message', ''):
            continue
        t = event['t']
        pose = min(trajectory, key=lambda sample: abs(sample['t']-t))
        row = {'event_t': t, 'message': event['message'], 'nearest_pose_t': pose['t'], 'maps': {}}
        for label, candidates in [('before', [m for m in maps if m['t'] <= t]),
                                  ('after', [m for m in maps if m['t'] >= t])]:
            if not candidates:
                row['maps'][label] = None
                continue
            grid = candidates[-1] if label == 'before' else candidates[0]
            row['maps'][label] = {'map_t': grid['t'], 'origin': grid['origin'],
                                  'resolution': grid['resolution'],
                                  'start_approximation': endpoint(grid, pose['x'], pose['y'], radius, clearance),
                                  'goal': endpoint(grid, goal_x, goal_y, radius, clearance)}
        rows.append(row)
    return {'schema': 'rm_tdt_planner/endpoint_snapshot_inspection/v1',
            'kind': 'historical_published_data_analysis_not_planner_or_runtime_validation',
            'observation': str(observation), 'radius_m': radius, 'clearance_m': clearance,
            'limitations': [
                'Published costmaps bracket callbacks but are not exact planning snapshots.',
                'Start uses nearby ground truth, not the actual action request start.',
                'OccupancyGrid resolution is float32; grid-line rounding may differ from native double.',
                'Seed-model predictions do not prove a returned path or static navigation success.'],
            'input_sha256': {name: hashlib.sha256(data).hexdigest() for name, data in content.items()},
            'endpoint_refusals': len(rows), 'events': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('observation', type=Path)
    parser.add_argument('--radius', type=float, required=True)
    parser.add_argument('--clearance', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not all(math.isfinite(v) and 0 <= v <= 5 for v in (args.radius, args.clearance)):
        parser.error('radius/clearance must be finite and within [0,5]')
    report = inspect(args.observation, args.radius, args.clearance)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(f"Recorded {report['endpoint_refusals']} endpoint refusals; no planner executed.")


if __name__ == '__main__':
    main()
