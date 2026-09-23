"""Recompute first-case MPPI window evidence from immutable v2 raw artifacts."""
from pathlib import Path
import hashlib
import json
import math
import statistics
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUNS = ROOT / 'build/tdt_p2b/runs/dynamic_cycle_diagnostic_v2'
sys.path.insert(0, str(HERE.parent / 'dynamic_reference_20260922'))
sys.path.insert(0, str(HERE.parent / 'dynamic_failure_analysis_20260922'))
from dynamic_metrics import rows_from_transport, placed, padded, obstacle_polygon, polygon_distance
from analyze_window import interpolate
from read_trace import read_cycle, last


def jsonlines(path):
    return [json.loads(s) for s in path.open()]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def moving_gap(row, shape, box):
    return polygon_distance(placed(shape, row['robot']), placed(box, row['obstacle']))


def first_below(rows, shape, box, threshold, start, end):
    return next((r['t'] for r in rows if start <= r['t'] <= end and moving_gap(r, shape, box) < threshold), None)


def actual_box_cells(raw, meta, box_state):
    x, y, _ = box_state
    res = meta['resolution']
    ox, oy = meta['origin']
    out = {'253_inscribed': 0, '254_lethal': 0, '255_unknown': 0}
    for j in range(raw.shape[0]):
        cy = oy + (j+.5)*res
        if abs(cy-y) > .275:
            continue
        for i in range(raw.shape[1]):
            cx = ox + (i+.5)*res
            if abs(cx-x) > .225:
                continue
            cost = int(raw[j, i])
            if cost == 253:
                out['253_inscribed'] += 1
            elif cost == 254:
                out['254_lethal'] += 1
            elif cost == 255:
                out['255_unknown'] += 1
    return out


def gap_to_raw_lethal(raw, meta, robot_pose, shape):
    res = meta['resolution']
    ox, oy = meta['origin']
    robot = placed(shape, robot_pose)
    gap = math.inf
    for j, i in zip(*np.where(raw == 254)):
        x, y = ox+i*res, oy+j*res
        cell = ((x,y), (x+res,y), (x+res,y+res), (x,y+res))
        gap = min(gap, polygon_distance(robot, cell))
    return None if math.isinf(gap) else gap


def nearest_before(rows, topic, t):
    candidate = [r for r in rows if r['topic'] == topic and r['t'] <= t]
    return max(candidate, key=lambda r:r['t']) if candidate else None


def trial(name, inputs):
    d = RUNS / (name+'_1')
    summary = json.loads((d/'dynamic_summary.json').read_text())
    assert summary['evidence_valid'] and not summary['limited_dynamic_geometry_and_goal_pass']
    assert sha(d/'profile.yaml') == inputs['profiles'][name]
    geo = json.loads((d/'runtime_geometry_preflight.json').read_text())
    for entry in geo['runtime'].values():
        assert entry['actual'] == entry['expected']
    rows = rows_from_transport(d/'gazebo_poses.jsonl')
    poly = inputs['body_polygon_m']; padded_poly = padded(poly, .03); box = obstacle_polygon()
    nav_start, nav_end = summary['navigation_interval_sim_s']
    gate = first_below(rows, poly, box, .05, nav_start, nav_end)
    padded_contact = first_below(rows, padded_poly, box, 1e-12, nav_start, nav_end)
    body_contact = first_below(rows, poly, box, 1e-12, nav_start, nav_end)
    assert gate is not None
    files = sorted((d/'mppi_cycles').glob('cycle_*.json'), key=lambda p:int(p.stem.split('_')[1]))
    status = json.loads((d/'mppi_cycles/writer_status.json').read_text())
    assert status == {'attempted':len(files), 'written':len(files), 'dropped':0, 'errors':0, 'closed':True}
    assert [int(p.stem.split('_')[1]) for p in files] == list(range(len(files)))
    maps = (d/'mppi_cycles/loaded_maps.txt').read_text()
    assert '/work/mppi_cycle_diagnostic_v1/install/nav2_mppi_controller/lib/libmppi_controller.so' in maps
    assert '/work/mppi_cycle_diagnostic_v1/install/nav2_mppi_controller/lib/libmppi_critics.so' in maps
    commands = jsonlines(d/'observation/command_chain.jsonl')
    nav_cmds = [r for r in commands if r['topic'] == '/cmd_vel_nav']
    cycle_rows=[]; output_matches_topic=0; output_filter_mismatch=0; exceptions=0; no_output=0
    for f in files:
        m, arrays = read_cycle(f)
        assert m['schema'] == 'tdt_mppi_cycle/v1'
        raw = last(arrays, 'locked.raw_map')
        assert raw.shape == (m['map']['height'],m['map']['width'])
        for key in ('rollout.x','rollout.y','rollout.yaw','scored.costs','weighted.costs'):
            last(arrays,key)
        mask = last(arrays,'cost_critic.collisions')
        assert mask.size == 300 and np.all((mask==0)|(mask==1))
        exceptions += bool(m['unwinding_exception'])
        outputs = [e['value'] for e in m['events'] if e['kind']=='output']
        no_output += len(outputs)!=1
        output = outputs[0] if len(outputs)==1 else None
        if output is not None:
            filtered = [float(last(arrays, 'after_filter.'+k)[1]) for k in ('vx','vy','wz')]
            output_filter_mismatch += output != filtered
            output_matches_topic += any(max(abs(r[k]-output[i]) for i,k in enumerate(('vx','vy','wz'))) < 1e-7 for r in nav_cmds)
        cycle_rows.append((m['map_sim_ns']*1e-9,f,m,arrays,raw,mask,output))
    assert not exceptions and not no_output and not output_filter_mismatch and output_matches_topic == len(files)
    cycle_rows.sort(key=lambda row:row[0])
    index = min(range(len(cycle_rows)), key=lambda i:abs(cycle_rows[i][0]-gate))
    window=[]
    for t,f,m,arrays,raw,mask,output in cycle_rows[max(0,index-2):index+3]:
        actual = interpolate(rows,t)
        before = [float(last(arrays,'before_filter.'+k)[1]) for k in ('vx','vy','wz')]
        after = [float(last(arrays,'after_filter.'+k)[1]) for k in ('vx','vy','wz')]
        window.append({'cycle':int(m['cycle_id']),'map_capture_sim_s':t,'relative_to_first_gate_s':t-gate,
            'actual_robot_to_box_body_gap_m':moving_gap(actual,poly,box),
            'actual_robot_to_box_padded_gap_m':moving_gap(actual,padded_poly,box),
            'actual_box_raw_cell_counts':actual_box_cells(raw,m['map'],actual['obstacle']),
            'robot_pose_to_raw_254_body_gap_m':gap_to_raw_lethal(raw,m['map'],m['pose'],poly),
            'robot_pose_to_raw_254_padded_gap_m':gap_to_raw_lethal(raw,m['map'],m['pose'],padded_poly),
            'costcritic_colliding_rollouts':int(mask.sum()),'rollout_count':int(mask.size),
            'before_filter_selected_index_1':before,'after_filter_selected_index_1':after,
            'returned_mppi_command':output,'observer_copy_ms':m['observer_copy_ns']/1e6,
            'full_compute_cycle_ms':(m['finish_steady_ns']-m['request_steady_ns'])/1e6})
    copy_ms = [m['observer_copy_ns']/1e6 for _,_,m,*_ in cycle_rows]
    full_ms = [(m['finish_steady_ns']-m['request_steady_ns'])/1e6 for _,_,m,*_ in cycle_rows]
    receipt = {}
    for topic in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel','/simulation/ground_truth/odom'):
        sub = [r for r in commands if r['topic']==topic]
        before = nearest_before(commands,topic,gate)
        receipt[topic] = {'count':len(sub),'last_at_or_before_gate':None if before is None else
            {k:before[k] for k in ('t','vx','vy','wz','publisher_gid','callback_steady_ns')}}
    return {'planner':name,'source_series':'dynamic_cycle_diagnostic_v2','action_status':summary['action_status'],
        'recoveries':summary['recoveries'],'final_xy_error_m':summary['final_xy_error_m'],
        'final_yaw_error_rad':summary['final_yaw_error_rad'],
        'body_interpolation_lower_bound_m':summary['geometry']['new_car_reference']['body']['overall_interpolation_bound_m'],
        'padded_interpolation_lower_bound_m':summary['geometry']['new_car_reference']['padded']['overall_interpolation_bound_m'],
        'first_sample_body_below_005_sim_s':gate,'first_sample_padded_contact_sim_s':padded_contact,
        'first_sample_body_contact_sim_s':body_contact,
        'writer':status,'cycle_count':len(files),'output_matches_after_filter_index_1':len(files),
        'output_matches_cmd_vel_nav_value':output_matches_topic,
        'observer_copy_median_ms':statistics.median(copy_ms),'observer_copy_max_ms':max(copy_ms),
        'full_compute_cycle_median_ms':statistics.median(full_ms),'full_compute_cycle_max_ms':max(full_ms),
        'window_cycles':window,'command_chain':receipt,'runtime_geometry_preflight_verified':True,
        'postnavigation_runtime_geometry_present':(d/'runtime_geometry.json').is_file(),
        'per_message_publisher_gid_available':any(r['publisher_gid'] is not None for r in commands)}


def main():
    inputs = json.loads((RUNS/'inputs.json').read_text())
    for block in ('files','diagnostic_tools','diagnostic_libraries'):
        for name, digest in inputs[block].items():
            assert sha(ROOT/name) == digest, name
    result = {'schema':'rm_tdt_planner/mppi_cycle_diagnostic/v1',
        'source_series':'dynamic_cycle_diagnostic_v2',
        'source_inputs_sha256':sha(RUNS/'inputs.json'),
        'scope':'Two phase-zero first cases; instrumented MPPI, not dynamic acceptance or causal A/B.',
        'timing_limit':'Cycle map_capture_sim_s is time of locked raw copy, not last map update time; topic t is observer latest /clock callback, not calibrated transport latency.',
        'trials':[trial(name,inputs) for name in ('tdt_astar','tdt_qp')]}
    out = HERE/'cycle_analysis.json'
    if out.exists():
        assert json.loads(out.read_text()) == result, 'frozen cycle analysis changed'
    else:
        with out.open('x') as f:
            json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({t['planner']:{k:t[k] for k in ('cycle_count','recoveries','first_sample_body_below_005_sim_s','first_sample_padded_contact_sim_s','first_sample_body_contact_sim_s')} for t in result['trials']},indent=2))

if __name__=='__main__':
    main()
