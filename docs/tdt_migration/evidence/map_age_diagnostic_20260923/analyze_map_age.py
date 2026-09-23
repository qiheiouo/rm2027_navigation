#!/usr/bin/env python3
"""Join an MPPI locked raw map to its latest completed Nav2 master update."""
import bisect
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUNS = ROOT / 'build/tdt_p2b/runs/dynamic_map_age_pilot_v1'
sys.path.insert(0, str(HERE.parent / 'mppi_cycle_diagnostic_20260922'))
sys.path.insert(0, str(HERE.parent / 'dynamic_reference_20260922'))
sys.path.insert(0, str(HERE.parent / 'dynamic_failure_analysis_20260922'))
from read_trace import read_cycle, last
from audit_cycles import first_below, moving_gap, actual_box_cells, gap_to_raw_lethal
from dynamic_metrics import rows_from_transport, obstacle_polygon
from analyze_window import interpolate


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fnv64(data):
    value = 14695981039346656037
    for byte in data:
        value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return f'{value:016x}'


def pair_updates(events):
    """A completed map carries only observations selected in that update call."""
    result, selected = [], []
    for event in events:
        if event['kind'] == 'marking_selected':
            selected.append(event)
        elif event['kind'] == 'master_complete':
            row = dict(event)
            row['marking_stamps_ns'] = [x['stamp_ns'] for x in selected]
            row['selection_steady_ns'] = [x['steady_ns'] for x in selected]
            result.append(row)
            selected = []
        else:
            raise ValueError(f'unknown costmap event: {event["kind"]}')
    if selected:
        raise ValueError('unpaired marking observations')
    if any(b['steady_ns'] <= a['steady_ns'] for a, b in zip(result, result[1:])):
        raise ValueError('non-monotonic map completion events')
    return result


def latest_update(updates, complete_times, lock_ns, raw, meta):
    index = bisect.bisect_right(complete_times, lock_ns) - 1
    if index < 0:
        raise ValueError('no completed map before MPPI lock')
    row = updates[index]
    expected = (meta['frame'], meta['width'], meta['height'], meta['resolution'], meta['origin'])
    actual = (row['frame'], row['width'], row['height'], row['resolution'], row['origin'])
    if expected != actual or fnv64(memoryview(raw).cast('B')) != row['fnv64']:
        raise ValueError('latest completed map does not match locked MPPI raw bytes and geometry')
    return row


def percentile(values, p):
    return float(np.percentile(values, p)) if values else None


def mark_y_range(raw, meta):
    """Lethal cell centre Y range in fixed moving-obstacle X slab, not a box pose."""
    yy, xx = np.where(raw == 254)
    x = meta['origin'][0] + (xx + .5) * meta['resolution']
    y = meta['origin'][1] + (yy + .5) * meta['resolution']
    mask = (x >= 4.675) & (x <= 5.125) & (y >= -1.5) & (y <= 1.5)
    if not mask.any():
        return None
    return [float(y[mask].min()), float(y[mask].max())]


def analyze(name, inputs):
    trial = RUNS / f'{name}_1'
    assert sha(trial/'profile.yaml') == inputs['profiles'][name]
    summary = json.loads((trial/'dynamic_summary.json').read_text())
    assert summary['evidence_valid'] and not summary['limited_dynamic_geometry_and_goal_pass']
    geometry = json.loads((trial/'runtime_geometry_preflight.json').read_text())
    assert all(v['actual'] == v['expected'] for v in geometry['runtime'].values())
    assert (trial/'runtime_geometry.json').is_file()
    status = json.loads((trial/'mppi_cycles/writer_status.json').read_text())
    files = sorted((trial/'mppi_cycles').glob('cycle_*.json'),
                   key=lambda p: int(p.stem.split('_')[1]))
    assert status == {'attempted':len(files), 'written':len(files), 'dropped':0,
                      'errors':0, 'closed':True}
    assert [int(p.stem.split('_')[1]) for p in files] == list(range(len(files)))
    loaded = (trial/'mppi_cycles/loaded_maps.txt').read_text()
    assert '/work/map_age_diagnostic_v1/install/nav2_costmap_2d/lib/libnav2_costmap_2d_core.so' in loaded
    assert '/work/mppi_cycle_diagnostic_v1/install/nav2_mppi_controller/lib/libmppi_controller.so' in loaded
    event_files = sorted((trial/'map_trace').glob('map_*.jsonl'))
    assert len(event_files) >= 2
    by_file = [[json.loads(s) for s in p.read_text().splitlines()] for p in event_files]
    local_files = [events for events in by_file if any(e.get('frame') == 'odom' for e in events)]
    assert len(local_files) == 1, 'ambiguous local costmap writer'
    local = pair_updates(local_files[0])
    assert local and all(e['frame'] == 'odom' for e in local)
    times = [e['steady_ns'] for e in local]
    poses = rows_from_transport(trial/'gazebo_poses.jsonl')
    body = inputs['body_polygon_m']
    box = obstacle_polygon()
    start, end = summary['navigation_interval_sim_s']
    gate = first_below(poses, body, box, .05, start, end)
    assert gate is not None
    all_cycles = []
    for path in files:
        cycle, arrays = read_cycle(path)
        raw = last(arrays, 'locked.raw_map')
        update = latest_update(local, times, cycle['lock_acquired_steady_ns'], raw, cycle['map'])
        stamp_ns = max(update['marking_stamps_ns']) if update['marking_stamps_ns'] else None
        sim_t = cycle['map_sim_ns'] * 1e-9
        map_age_ms = (cycle['lock_acquired_steady_ns'] - update['steady_ns']) * 1e-6
        assert map_age_ms >= 0
        row = {'cycle_id':cycle['cycle_id'], 'sim_s':sim_t,
               'map_age_ms':map_age_ms, 'map_complete_steady_ns':update['steady_ns'],
               'map_hash_fnv64':update['fnv64'],
               'map_origin':update['origin'],
               'latest_selected_scan_stamp_s':stamp_ns*1e-9 if stamp_ns is not None else None,
               'selected_scan_age_sim_s':(cycle['map_sim_ns']-stamp_ns)*1e-9 if stamp_ns is not None else None,
               'selected_observation_count':len(update['marking_stamps_ns']),
               'nonzero_input_speed':math.hypot(*cycle['speed'][:2]) > .1,
               'unwinding_exception':cycle['unwinding_exception']}
        all_cycles.append((row, cycle, arrays, raw))
    assert len(all_cycles) == len(files)
    closest = min(range(len(all_cycles)), key=lambda i:abs(all_cycles[i][0]['sim_s']-gate))
    window = []
    for row, cycle, arrays, raw in all_cycles[max(0,closest-3):closest+4]:
        actual = interpolate(poses, row['sim_s'])
        scan_state = interpolate(poses, row['latest_selected_scan_stamp_s']) if row['latest_selected_scan_stamp_s'] is not None else None
        displacement = math.dist(actual['obstacle'][:2], scan_state['obstacle'][:2]) if scan_state else None
        output = [e['value'] for e in cycle['events'] if e['kind']=='output']
        window.append({**row,
            'actual_obstacle_pose':actual['obstacle'],
            'observed_obstacle_displacement_since_selected_scan_m':displacement,
            'actual_robot_body_to_box_gap_m':moving_gap(actual,body,box),
            'raw_254_body_gap_m':gap_to_raw_lethal(raw,cycle['map'],cycle['pose'],body),
            'actual_box_cell_counts':actual_box_cells(raw,cycle['map'],actual['obstacle']),
            'moving_x_slab_raw_254_y_range_m':mark_y_range(raw,cycle['map']),
            'cost_critic_collision_rollouts':int(last(arrays,'cost_critic.collisions').sum()),
            'returned_command':output[0] if len(output)==1 else None})
    ages = [r['map_age_ms'] for r,*_ in all_cycles]
    obs_ages = [r['selected_scan_age_sim_s'] for r,*_ in all_cycles if r['selected_scan_age_sim_s'] is not None]
    return {'planner':name,'scope':'fixed-phase instrumented first case, not acceptance',
            'action_status':summary['action_status'],'recoveries':summary['recoveries'],
            'body_interpolation_bound_m':summary['geometry']['new_car_reference']['body']['moving_interpolation_bound_m'],
            'padded_interpolation_bound_m':summary['geometry']['new_car_reference']['padded']['moving_interpolation_bound_m'],
            'dynamic_gate_pass':summary['limited_dynamic_geometry_and_goal_pass'],
            'first_body_below_005_sim_s':gate,
            'cycle_count':len(files),'local_update_count':len(local),
            'all_cycles_latest_completed_map_matches_raw':True,
            'nonzero_speed_cycles':sum(r['nonzero_input_speed'] for r,*_ in all_cycles),
            'exception_cycles':sum(r['unwinding_exception'] for r,*_ in all_cycles),
            'cycles_without_selected_observation':len(all_cycles)-len(obs_ages),
            'map_age_ms':{'median':statistics.median(ages),'p95':percentile(ages,95),'max':max(ages)},
            'selected_scan_age_sim_s':{'median':statistics.median(obs_ages) if obs_ages else None,
                                       'p95':percentile(obs_ages,95),'max':max(obs_ages) if obs_ages else None},
            'window':window}


def main():
    inputs=json.loads((RUNS/'inputs.json').read_text())
    result={'schema':'tdt_costmap_age_analysis/v1','series':str(RUNS.relative_to(ROOT)),
            'trials':[analyze(name,inputs) for name in ('tdt_astar','tdt_qp')],
            'interpretation':'map age uses steady clock; selected scan age uses simulation stamps; neither proves causation or braking feasibility'}
    dest=HERE/'map_age_analysis_v2.json'
    with dest.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    for t in result['trials']:
        print(t['planner'],t['cycle_count'],'cycles, map age median/p95/max ms',t['map_age_ms'],
              'scan age median/p95/max sim s',t['selected_scan_age_sim_s'])

if __name__=='__main__':main()
