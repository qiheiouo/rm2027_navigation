#!/usr/bin/env python3
"""Independent physical contact timeline with reported guard snapshots, offline only."""
import argparse
import bisect
import json
import math
from pathlib import Path
import yaml
from analyze_trial import distance, load_truth_rows, rotation, travel
from audit_scan_geometry import fixture, interpolate
from trial_io import rows


def analyze(root):
    execution = json.loads((root / 'execution.json').read_text())
    if execution['execution'] != 'PASS':
        return {'scope': 'infrastructure_before_goal', 'witnesses': {}}
    profile = yaml.safe_load((root / 'profile.yaml').read_text())
    body = yaml.safe_load(profile['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    cfg = yaml.safe_load((root / 'installed_inputs/guard.yaml').read_text())['dynamic_safety_guard']['ros__parameters']
    footprint = list(zip(cfg['footprint'][::2], cfg['footprint'][1::2]))
    geometry = fixture(root / 'scene_inputs'); x, y = geometry['actor_dimensions']
    actor_box = [(-x/2, -y/2), (x/2, -y/2), (x/2, y/2), (-x/2, y/2)]
    physical = load_truth_rows(root, execution); times = [r['t'] for r in physical]
    robot_poses = [r['robot'] for r in physical]; actor_poses = [r['obstacle'] for r in physical]
    records = list(rows(root, 'observations.jsonl'))
    guards = sorted((r for r in records if r['kind'] == 'guard'), key=lambda r: r['receive_sim'])
    guard_times = [r['receive_sim'] for r in guards]
    obstacles = {round(r['stamp'], 9): r for r in records if r['kind'] == 'obstacles'}
    commands = [r for r in records if r['kind'] == 'final_cmd']
    gaps = [distance(rotation(r['robot'], body), rotation(r['obstacle'], actor_box)) for r in physical]
    radius = max(math.hypot(*p) for p in body); witnesses = {}
    policy = json.loads((root / 'policy.json').read_text())
    for label, limit in [('first_body_margin_violation', policy['body_clearance']), ('first_body_contact', 0.)]:
        i = next((i for i, gap in enumerate(gaps) if (gap < limit if limit > 0 else gap <= 0)), None)
        if i is None:
            continue
        truth = physical[i]; stamp = truth['t']; start = stamp - .2
        before = [r for r in physical if start <= r['t'] <= stamp]
        window_commands = [r for r in commands if start <= r['receive_sim'] <= stamp]
        j = bisect.bisect_right(guard_times, stamp) - 1
        witness = {'time': stamp, 'physical_body_gap': gaps[i], 'physical_pose': truth['robot'], 'physical_actor_pose': truth['obstacle'],
                   'prior_window_s': .2, 'prior_window_truth_rows': len(before),
                   'prior_window_max_pose_deviation_m': max((travel(before[0]['robot'], r['robot'], radius) for r in before), default=None),
                   'prior_window_final_commands': len(window_commands),
                   'prior_window_max_abs_final_command': max((max(abs(v) for v in r['velocity']) for r in window_commands), default=None)}
        if j >= 0:
            guard = guards[j]; status = guard['statuses'][0]; values = status['values']
            witness['latest_observer_available_guard'] = guard
            witness['guard_receipt_age'] = stamp - guard['receive_sim']
            if values.get('evaluated') == '1':
                source = float(values['source_stamp']); epoch = guard['stamp']
                obstacle = obstacles.get(round(source, 9))
                witness['matched_guard_source_obstacle_receipt'] = obstacle
                actual = interpolate(times, actor_poses, epoch, .04)
                actual_robot = interpolate(times, robot_poses, epoch, .04)
                if obstacle is not None and actual is not None and actual_robot is not None:
                    corners = rotation(actual, actor_box); disks = []
                    a = float(values['world_transform_yaw']); c, s = math.cos(a), math.sin(a)
                    for track in obstacle['tracks']:
                        if track['state'] not in (2, 3): continue
                        px = track['xy'][0] + track['vxy'][0] * (epoch - source)
                        py = track['xy'][1] + track['vxy'][1] * (epoch - source)
                        center = (float(values['world_transform_x']) + c*px-s*py,
                                  float(values['world_transform_y']) + s*px+c*py)
                        disk_radius = max(cfg['minimum_obstacle_radius'], .5*math.hypot(*track['size']))
                        disks.append({'id': track['id'], 'state': track['state'], 'age': epoch-source,
                            'observation_age': epoch-track['observed'], 'CV_center_at_guard_epoch': center,
                            'radius': disk_radius, 'physical_actor_center_error': math.hypot(center[0]-actual[0], center[1]-actual[1]),
                            'full_physical_actor_box_covered': all(math.hypot(p[0]-center[0], p[1]-center[1]) <= disk_radius for p in corners)})
                    witness['source_matched_model_labels'] = disks
                    witness['physical_gap_at_guard_epoch'] = distance(rotation(actual_robot, body), corners)
        witnesses[label] = witness
    return {'scope': 'offline body/actor contact chronology; guard snapshot chosen by observer receipt, not native optimizer/control identity',
            'witnesses': witnesses,
            'limits': 'Prior pose deviation includes physical contact motion; zero reported commands alone do not prove all actuator responses. Observer source matching is not exact DDS consumption. No safe-control/SG coverage witness or attribution of a single root cause.'}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); args = parser.parse_args()
    result = analyze(args.trial)
    (args.trial / 'contact_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
