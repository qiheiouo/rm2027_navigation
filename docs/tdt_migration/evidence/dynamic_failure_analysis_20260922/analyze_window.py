"""Read frozen dynamic pilots; never run navigation or mutate their artifacts."""
import bisect
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREVIOUS = HERE.parent / 'dynamic_reference_20260922'
sys.path.insert(0, str(PREVIOUS))
from dynamic_metrics import rows_from_transport, placed, padded, obstacle_polygon, polygon_distance


def read_rows(path):
    return [json.loads(line) for line in path.open()]


def interpolate(rows, t):
    times = [r['t'] for r in rows]
    if not times[0] <= t <= times[-1]:
        raise ValueError('time outside actual transport coverage')
    i = min(max(1, bisect.bisect_right(times, t)), len(rows)-1)
    a, b = rows[i-1], rows[i]
    f = (t-a['t'])/(b['t']-a['t'])
    result = {'t': t}
    for key in ('robot', 'obstacle'):
        x, y, yaw = a[key]
        u, v, angle = b[key]
        result[key] = [x+f*(u-x), y+f*(v-y), yaw+f*math.remainder(angle-yaw, 2*math.pi)]
    return result


def ray_box(origin, direction, bounds):
    """Positive ray entry distance to an axis-aligned closed box, or None."""
    lo, hi = 0., float('inf')
    for o, d, a, b in zip(origin, direction, bounds[:2], bounds[2:]):
        if abs(d) < 1e-12:
            if not a <= o <= b:
                return None
            continue
        u, v = sorted(((a-o)/d, (b-o)/d))
        lo, hi = max(lo, u), min(hi, v)
        if lo > hi:
            return None
    return lo


def scan_review(scan, poses, tolerance):
    state = interpolate(poses, scan['t'])
    x, y, theta = state['robot']
    ox, oy, angle = state['obstacle']
    assert abs(angle) < 1e-8 and scan['frame'] == 'sim_lidar_link'
    bounds = (ox-.225, oy-.275, ox+.225, oy+.275)
    matches, occluded, behind, errors, expected_ranges = 0, 0, 0, [], []
    for i, value in enumerate(scan['ranges']):
        yaw = theta+scan['angle_min']+i*scan['angle_increment']
        distance = ray_box((x,y), (math.cos(yaw), math.sin(yaw)), bounds)
        if distance is None or not scan['range_min'] <= distance <= scan['range_max']:
            continue
        expected_ranges.append(distance)
        if not isinstance(value, (float,int)):
            behind += 1
            continue
        error = value-distance
        errors.append(error)
        if abs(error) <= tolerance:
            matches += 1
        elif error < 0:
            occluded += 1
        else:
            behind += 1
    return {'t': scan['t'], 'expected_box_rays': len(expected_ranges),
            'matching_rays': matches, 'nearer_returns': occluded, 'farther_or_missing_returns': behind,
            'median_signed_error_m': statistics.median(errors) if errors else None,
            'diagnostic_match_tolerance_m': tolerance}


def map_review(grid, state, polygon):
    r, w = grid['resolution'], grid['width']
    ox, oy = grid['origin']
    x, y, _ = state['obstacle']
    counts = {'100_lethal': 0, '99_inscribed': 0, 'other': 0}
    body = placed(polygon, state['robot'])
    pad = placed(padded(polygon,.03), state['robot'])
    body_gap, padded_gap = float('inf'), float('inf')
    lethal_cells = []
    for i,cost in enumerate(grid['data']):
        cx, cy = ox+(i%w+.5)*r, oy+(i//w+.5)*r
        if abs(cx-x) <= .225 and abs(cy-y) <= .275:
            counts['100_lethal' if cost==100 else '99_inscribed' if cost==99 else 'other'] += 1
        if cost != 100:
            continue
        # Exact distance to every published lethal closed cell, not its centre.
        cell = [(cx-r/2,cy-r/2),(cx+r/2,cy-r/2),(cx+r/2,cy+r/2),(cx-r/2,cy+r/2)]
        body_gap = min(body_gap, polygon_distance(body,cell))
        padded_gap = min(padded_gap, polygon_distance(pad,cell))
        if abs(cx-x) < 1 and abs(cy-y) < 1:
            lethal_cells.append([cx,cy])
    return {'t': state['t'], 'map_stamp':grid['t'],'stamp_age_s':state['t']-grid['t'],
            'frame':grid['frame'], 'box_centre_counts':counts,
            'body_to_published_lethal_cells_m': body_gap if math.isfinite(body_gap) else None,
            'padded_to_published_lethal_cells_m': padded_gap if math.isfinite(padded_gap) else None,
            'lethal_centres_near_box':lethal_cells}


def first_stop(rows, after):
    return next((r for r in rows if r['t'] >= after and max(abs(r[k]) for k in ('vx','vy','wz')) <= .01), None)


def analyze(name, critical, polygon, tolerance):
    p = ROOT/'build/tdt_p2b/runs/dynamic_reference_pilot_v2'/f'{name}_1'
    poses = rows_from_transport(p/'gazebo_poses.jsonl')
    t0 = critical['first_below_005']['t']
    collision_t = critical['first_sampled_body_overlap']['t']
    start, end = t0-1.5, collision_t+1.5
    commands = read_rows(p/'observation/commands.jsonl')
    trajectory = read_rows(p/'observation/trajectory.jsonl')
    scans = [scan_review(s,poses,tolerance) for s in read_rows(p/'observation/scans.jsonl') if start <= s['t'] <= end]
    maps = read_rows(p/'observation/local_costmap.jsonl')
    box = obstacle_polygon()
    series = []
    for s in poses:
        if not start <= s['t'] <= end:
            continue
        row = dict(s)
        row['body_gap_m'] = polygon_distance(placed(polygon,s['robot']),placed(box,s['obstacle']))
        series.append(row)
    points = {}
    for label,t in [('one_second_before',t0-1),('half_second_before',t0-.5),('gate_violation',t0),('first_overlap',collision_t)]:
        state = interpolate(poses,t)
        local = max((g for g in maps if g['t'] <= t), key=lambda g:g['t'])
        scan = max((g for g in scans if g['t'] <= t), key=lambda g:g['t'])
        horizon = [r for r in poses if t <= r['t'] <= t+1]
        frozen_robot = placed(polygon,state['robot'])
        frozen_obstacle = placed(box,state['obstacle'])
        points[label] = {'state':state,'local_map':map_review(local,state,polygon),'last_scan':scan,
            'last_scan_stamp_age_s':t-scan['t'],
            'last_command':next(r for r in reversed(commands) if r['t'] <= t),
            'actual_twist_sample':min(trajectory,key=lambda r:abs(r['t']-t)),
            'counterfactual_next_1s':{
                'freeze_robot_actual_obstacle_gap_m':min(polygon_distance(frozen_robot,placed(box,r['obstacle'])) for r in horizon),
                'actual_robot_freeze_obstacle_gap_m':min(polygon_distance(placed(polygon,r['robot']),frozen_obstacle) for r in horizon),
                'scope':'Diagnostic counterfactual using observed future path; instant freeze is not a feasible braking trajectory or controller replay.'}}
    command_stop = first_stop(commands,t0)
    actual_stop = first_stop(trajectory,t0)
    errors = [r for r in read_rows(p/'observation/events.jsonl') if start <= r['t'] <= end and r.get('level',0)>=40]
    result = {'planner':name,'interval_sim_s':[start,end],'points':points,'first_output_stop_after_violation':command_stop,
        'first_measured_stop_after_violation':actual_stop,'controller_errors':errors,
        'scans':scans,'actual_pose_series':series,
        'commands':[r for r in commands if start <= r['t'] <= end],
        'trajectory':[r for r in trajectory if start <= r['t'] <= end],
        'max_scan_header_gap_s':max(b['t']-a['t'] for a,b in zip(scans,scans[1:])),
        'max_local_map_header_gap_s':max(b['t']-a['t'] for a,b in zip(maps,maps[1:]) if start <= a['t'] <= end)}
    return result


def main():
    manifest=json.loads((PREVIOUS/'manifest.json').read_text())
    for n,h in manifest['files'].items():
        assert hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==h,n
    profiles=json.loads((ROOT/'build/tdt_p2b/runs/dynamic_reference_pilot_v2/inputs.json').read_text())
    sdf=ET.parse(ROOT/'src/rm_simulation/worlds/phase1_omni.sdf')
    sensor=sdf.find('.//sensor[@name="phase1_planar_lidar"]')
    tolerance=3*float(sensor.find('.//noise/stddev').text)+float(sensor.find('.//range/resolution').text)
    prior=json.loads((PREVIOUS/'aggregate.json').read_text())
    results=[analyze(t['planner'],t,profiles['body_polygon_m'],tolerance) for t in prior['trials']]
    out={'schema':'rm_tdt_planner/dynamic_failure_window/v1','source_series':'dynamic_reference_pilot_v2',
        'new_simulation_trials':0,'runtime_changes':False,'scan_matching_scope':'Geometric ray/box range correspondence using actual poses and declared zero XY lidar offset; diagnostic 3 sigma plus range resolution, not a safety gate or measured perception latency.',
        'timing_scope':'Scan/map header stamps; command/log times are observer latest /clock at callback. No receive/publish timestamp reconstruction.',
        'trials':results}
    with (HERE/'window_analysis.json').open('x') as f:
        json.dump(out,f,indent=2,allow_nan=False);f.write('\n')
    for t in results:
        print(t['planner'])
        for name,p in t['points'].items():
            m=p['local_map'];s=p['last_scan']
            print(name,'t',p['state']['t'],'map',m['box_centre_counts'],'map_body_gap',m['body_to_published_lethal_cells_m'],'scan',s['matching_rays'],s['expected_box_rays'],'counterfactual',p['counterfactual_next_1s'])
        print('stops',t['first_output_stop_after_violation']['t'],t['first_measured_stop_after_violation']['t'])

if __name__=='__main__':main()
