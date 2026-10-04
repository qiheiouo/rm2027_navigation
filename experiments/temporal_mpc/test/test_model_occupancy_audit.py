import math
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'gazebo'))
from audit_model_occupancy import evaluate, free_sections, goal_occupancy, preference_at_cut
from temporal_mpc.contracts import Geometry
from temporal_mpc.geometry import clearance


def obstacle(x, y, radius, track=1):
    return dict(center=[x, y], radius=radius, expanded_radius=radius,
                speed=0., track_id=track)


def test_analytic_sections_match_all_track_rectangle_checks():
    obs = [obstacle(1., .2, .8), obstacle(1.4, -.8, .7, 2),
           obstacle(5., 0., 1., 3)]
    bounds = (-1., 6., -2.4, 2.4)
    ys = np.linspace(-2.3, 2.3, 501)
    for x in np.linspace(0., 5.5, 17):
        sections = free_sections(x, bounds, obs)['free_intervals']
        states = np.zeros((len(ys), 6)); states[:, 0] = x; states[:, 1] = ys
        values = []
        for o in obs:
            shape = Geometry('circle', radius=o['expanded_radius'], source='independent fixture')
            values.append(clearance(states, np.tile(o['center'], (len(ys), 1)), shape, (.355, .330)))
        slack = np.min(values, axis=0)
        # The production geometry uses signed interior distance; both checks
        # have the same feasible set. Ignore numerical boundary coincidences.
        for y, value in zip(ys, slack):
            if abs(value) < 1e-9: continue
            assert any(a <= y <= b for a, b in sections) == (value > 0.)
    # Exact tangency to the expanded circle permits the whole section.
    assert len(free_sections(1. + .355 + .8, bounds, [obs[0]])['free_intervals']) == 1


def test_point_exclusion_does_not_imply_goal_disk_exclusion():
    o = obstacle(0., 0., .5)
    assert goal_occupancy([.8, 0.], .15, [o])['point_excluded']
    assert not goal_occupancy([.8, 0.], .15, [o])['entire_goal_disk_excluded']
    blocked = goal_occupancy([.7, 0.], .15, [o])
    assert blocked['entire_goal_disk_excluded']
    angles = np.linspace(0., 2 * math.pi, 73)
    points = np.array([.7, 0.]) + .15 * np.c_[np.cos(angles), np.sin(angles)]
    states = np.zeros((len(points), 6)); states[:, :2] = points
    shape = Geometry('circle', radius=.5, source='disk boundary fixture')
    assert np.max(clearance(states, np.zeros_like(points), shape, (.355, .330))) < 0.
    empty = goal_occupancy([0., 0.], .15, [])
    assert empty['point_slack_m'] is None and not empty['entire_goal_disk_excluded']


def test_preference_insufficiency_is_separate_from_current_hard_space():
    radius = 1.6970562748477143
    o = obstacle(3., 0., radius)
    o['expanded_radius'] += .02 + .025 * math.hypot(.8, .5)
    result = preference_at_cut([0., 0.], (-.6, 7., -2.2, 2.2), [o])
    assert not any(result['preference_target_fits'])
    assert result['hard_current_section']['free_width_m'] > .1
    assert result['preference_excludes_both_but_current_section_exists']
    narrow = preference_at_cut([0., 0.], (-.6, 7., -1., 1.), [o])
    assert narrow['hard_current_section']['free_width_m'] == 0.
    assert not narrow['preference_excludes_both_but_current_section_exists']
    assert not free_sections(10., (-.6, 7., -2.2, 2.2), [o])['in_corridor']


def test_recorded_source_and_native_epochs_are_distinct_and_stale_is_rejected(tmp_path):
    from test_contracts import message, stamp
    def plain(value):
        if hasattr(value, '__dict__'): return {k: plain(v) for k, v in vars(value).items()}
        if isinstance(value, list): return [plain(v) for v in value]
        return value
    msg = message(); msg.header.stamp = stamp(1, 100_000_000)
    msg.tracks[0].state = 3; msg.tracks[0].position.x = 2.05
    stale = message(); stale.header.stamp = stamp(1, 200_000_000)
    stale.tracks[0].state = 3; stale.tracks[0].position.x = 2.1
    def event(topic, data, receipt):
        return dict(topic=topic, data=data, receipt_sim_ns=receipt)
    health = dict(evaluation_ns=1_250_000_000, proposal_ns=1_220_000_000,
                  prediction_ns=1_100_000_000, executed=True, ready=True,
                  step=30, track_id=0, slack=None, constraint='accepted',
                  initial_state=[0., 0., 0., 0., 0., 0.])
    proposal = dict(header=dict(stamp=dict(sec=1, nanosec=220_000_000)),
                    centre_bounds=[-.6, 7., -2.4, 2.4], fixed_yaw=0.)
    events = [event('goal_sent', dict(pose=dict(header=dict(frame_id='map'),
              pose=dict(position=dict(x=5.6, y=0.)))), 1_100_000_000),
              event('/dynamic_obstacle_predictions', plain(msg), 1_200_000_000),
              event('/temporal_mpc/proposal', proposal, 1_230_000_000),
              event('/temporal_mpc/health', dict(data=json.dumps(health)), 1_260_000_000),
              event('/temporal_mpc/health', dict(data=json.dumps(dict(health,
                    constraint='input', ready=False, executed=False))), 1_270_000_000),
              event('/dynamic_obstacle_predictions', plain(stale), 1_700_000_000)]
    (tmp_path/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
    (tmp_path/'run_summary.json').write_text(json.dumps(dict(goal_epoch_s=1.1)))
    (tmp_path/'scene').mkdir()
    (tmp_path/'scene/nav2.yaml').write_text('controller_server:\n  ros__parameters:\n    general_goal_checker:\n      xy_goal_tolerance: 0.15\n')
    report, rows = evaluate(tmp_path)
    source, native = rows
    assert source['kind'] == 'public_source' and native['kind'] == 'native_evaluation'
    assert source['obstacles'][0]['center'][0] == 2.05
    assert abs(native['obstacles'][0]['center'][0]-2.125) < 1e-12
    assert report['current_public_source']['samples'] == 1
    assert sum(report['public_receipt_rejections'].values()) == 1
    assert report['diagnostic_missing_inputs'] == {}
