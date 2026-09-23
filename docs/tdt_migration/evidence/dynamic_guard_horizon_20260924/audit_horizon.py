#!/usr/bin/env python3
"""Recompute every frozen horizon-guard decision and compare the two QP pilots."""
import collections
import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUNS = ROOT / 'build/tdt_p2b/runs'
SERIES = RUNS / 'dynamic_guard_horizon_pilot_v2'
TRIAL = SERIES / 'tdt_qp_1'
sys.path.insert(0, str(HERE.parent / 'dynamic_guard_pilot_20260923'))
from guard_core import BODY_GATE_M, HOLD_S, certificate


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.open()]


def cell_at(grid, x, y):
    ox, oy = grid['origin']
    res = grid['resolution']
    ix, iy = math.floor((x - ox) / res), math.floor((y - oy) / res)
    if not (0 <= ix < grid['width'] and 0 <= iy < grid['height']):
        return None
    return grid['data'][iy * grid['width'] + ix]


def near_equal(left, right):
    return all(abs(a-b) < 1e-11 for a, b in zip(left, right))


def main():
    inputs = json.loads((SERIES/'inputs.json').read_text())
    old = RUNS/'dynamic_scaled_guard_pilot_v1/profile.yaml'
    assert sha(SERIES/'profile.yaml') == sha(old) == inputs['profile_sha256']
    assert sha(TRIAL/'profile.yaml') == inputs['profile_sha256']
    for path, digest in inputs['files'].items():
        assert sha(ROOT/path) == digest, path
    assert int((TRIAL/'docker_exit.txt').read_text()) == 0
    assert int((TRIAL/'geometry_capture_exit.txt').read_text()) == 0
    summary = json.loads((TRIAL/'observation/summary.json').read_text())
    dynamic = json.loads((TRIAL/'dynamic_summary.json').read_text())
    assert summary['evidence_valid'] and dynamic['evidence_valid']
    assert dynamic['original_static_summary_matches']
    decisions = rows(TRIAL/'guard_decisions.jsonl')
    assert decisions[0]['kind'] == 'configuration'
    assert decisions[1]['kind'] == 'policy'
    assert decisions[1]['next_interval_s'] == .05
    body = decisions[0]['body']
    from guard_core import fixture_sweep
    sweep, fixture = fixture_sweep()
    actual = [x for x in decisions if x['kind'] == 'decision']
    counts = collections.Counter(x['reason'] for x in actual)
    invalid_zero = []
    for row in actual:
        hold = HOLD_S + .05 + max(0, row['odom_age_s'])
        assert abs(hold - row['response_horizon_s']) < 1e-12
        recomputed = certificate(row['pose'], row['odom_speed'], row['emitted'], body,
                                 sweep, hold_s=hold)
        logged = row['certificate']
        assert recomputed['safe_under_model'] == logged['safe_under_model']
        for key in ('body_lower_m','padded_lower_m','body_gap_m','padded_gap_m'):
            assert abs(recomputed[key]-logged[key]) < 1e-9, (row['sim_s'], key)
        assert recomputed['safe_under_model'] == (recomputed['body_lower_m'] >= BODY_GATE_M
                                                  and recomputed['padded_lower_m'] > 0)
        if row['reason'] == 'admit':
            assert row['scale'] == 1 and near_equal(row['emitted'], row['proposed'])
            assert recomputed['safe_under_model']
        elif row['reason'] == 'scale_sweep':
            assert 0 < row['scale'] < 1 and recomputed['safe_under_model']
            assert not row['full_command_certificate']['safe_under_model']
            assert near_equal(row['emitted'], [row['scale']*v for v in row['proposed']])
        elif row['reason'] == 'reject_unstoppable':
            assert row['scale'] == 0 and near_equal(row['emitted'], (0,0,0))
            assert not recomputed['safe_under_model']
            invalid_zero.append(row)
        else:
            raise AssertionError(row['reason'])
    first_bad = next((i for i,x in enumerate(actual) if x['reason']=='reject_unstoppable'),None)
    assert first_bad is not None and first_bad > 0
    previous = actual[first_bad-1]
    assert previous['certificate']['safe_under_model']
    chain = rows(TRIAL/'observation/command_chain.jsonl')
    chain_counts = collections.Counter(x['topic'] for x in chain)
    assert chain_counts['/cmd_vel'] == chain_counts['/cmd_vel_guarded'] == chain_counts['/simulation/chassis/cmd_vel']
    maps={}
    for name,file,lethal in (('global','costmap.jsonl',100),('local','raw_local_maps.jsonl',254)):
        grids=rows(TRIAL/'observation'/file)
        checked=[g for g in grids if g['t']>=24 and cell_at(g,4.9,0) is not None]
        assert checked and all(cell_at(g,4.9,0)==lethal for g in checked)
        maps[name]={'recorded':len(grids),'sweep_center_checked':len(checked),'lethal':len(checked)}
    old_audit=json.loads((HERE.parent/'dynamic_scaled_guard_20260924/scaled_audit.json').read_text())
    geom=dynamic['geometry']['new_car_reference']
    result={
      'schema':'tdt_dynamic_guard_horizon_audit/v1',
      'profile_sha256':inputs['profile_sha256'],
      'same_profile_as_scaled_pilot':True,
      'fixture_source':fixture,
      'v1_launch_error':(RUNS/'dynamic_guard_horizon_pilot_v1/tdt_qp_1/observer.log').read_text().strip(),
      'v1_entered_preflight':False,
      'v2':{
        'docker_exit':0,'preflight_status':summary['preflight']['status'],
        'action_status':summary['action_status'],'timed_out':summary['timed_out'],
        'recoveries':summary['recoveries'],'final_xy_error_m':summary['final_xy_error_m'],
        'final_yaw_error_rad':summary['final_yaw_error_rad'],
        'cross_track_rms_m':summary['cross_track_rms_m'],
        'checks':dynamic['checks'],
        'limited_dynamic_geometry_and_goal_pass':dynamic['limited_dynamic_geometry_and_goal_pass'],
        'body_moving_interpolation_bound_m':geom['body']['moving_interpolation_bound_m'],
        'padded_moving_interpolation_bound_m':geom['padded']['moving_interpolation_bound_m'],
        'max_gazebo_gap_s':dynamic['continuity']['max_gazebo_gap_s'],
        'guard_decisions_recomputed':len(actual),
        'guard_decisions_by_reason':dict(counts),
        'model_unstoppable_zero_commands':len(invalid_zero),
        'min_selected_body_lower_m':min(x['certificate']['body_lower_m'] for x in actual),
        'min_selected_padded_lower_m':min(x['certificate']['padded_lower_m'] for x in actual),
        'first_unstoppable_transition':{
          'previous_sim_s':previous['sim_s'],
          'previous_reason':previous['reason'],
          'previous_body_lower_m':previous['certificate']['body_lower_m'],
          'first_unstoppable_sim_s':actual[first_bad]['sim_s'],
          'first_unstoppable_body_lower_m':actual[first_bad]['certificate']['body_lower_m'],
          'first_unstoppable_padded_lower_m':actual[first_bad]['certificate']['padded_lower_m'],
          'delta_sim_s':actual[first_bad]['sim_s']-previous['sim_s']},
        'command_chain_counts':dict(chain_counts),
        'costmap_marking':maps,
        'qp_adopted':dynamic['qp_adopted'],
        'validated_astar_fallbacks':dynamic['validated_astar_fallbacks']},
      'prior_scaled_pilot':{
        'action_status':old_audit['action_status'],
        'recoveries':old_audit['recoveries'],
        'model_unstoppable_zero_commands':old_audit['model_unstoppable_zero_commands'],
        'body_moving_interpolation_bound_m':old_audit['body_moving_interpolation_bound_m']},
      'accepted_for_deployment':False}
    with (HERE/'horizon_audit.json').open('x') as f:
        json.dump(result,f,indent=2,allow_nan=False)
        f.write('\n')


if __name__=='__main__':
    main()
