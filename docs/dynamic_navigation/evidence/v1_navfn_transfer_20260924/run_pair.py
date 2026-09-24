#!/usr/bin/env python3
"""One prespecified Navfn phase-0 dynamic V1 transfer pair; no overwrites."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT / 'build/tdt_p2b'
SERIES = WORK / 'runs/dynamic_prediction_navfn_transfer_v1'
TDT_PROFILE = WORK / 'runs/dynamic_odom_routing_pilot_v1/profiles/tdt_qp.yaml'
NAVFN_PROFILE = WORK / 'runs/static_v5/navfn_1/profile.yaml'
TDT_SHA = '1862d4dc9742f10d8d1db1d4404ef5f8123c7184e876c7e6179410ad9f3f9505'
RUN_TRIAL = ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/run_trial.sh'
AUDIT_PATH = ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def base_profile():
    assert sha(TDT_PROFILE) == TDT_SHA
    profile = yaml.safe_load(TDT_PROFILE.read_text())
    original = yaml.safe_load(TDT_PROFILE.read_text())
    known_navfn = yaml.safe_load(NAVFN_PROFILE.read_text())['planner_server']['ros__parameters']['GridBased']
    assert known_navfn['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    profile['planner_server']['ros__parameters']['GridBased'] = known_navfn
    comparison = yaml.safe_load(yaml.safe_dump(profile))
    comparison['planner_server']['ros__parameters']['GridBased'] = original['planner_server']['ros__parameters']['GridBased']
    assert comparison == original, 'Navfn baseline changed another dynamic parameter'
    assert profile['controller_server']['ros__parameters']['odom_topic'] == '/odometry/lio'
    return profile


def candidate_profile(baseline):
    profile = yaml.safe_load(yaml.safe_dump(baseline))
    follow = profile['controller_server']['ros__parameters']['FollowPath']
    assert 'PredictionV1Critic' not in follow['critics']
    follow['critics'].append('PredictionV1Critic')
    follow['PredictionV1Critic'] = {
        'enabled': True,
        'topic': '/perception/dynamic_obstacles_shadow/predictions',
        'object_width': .45,
        'object_height': .55,
        'reference_acceleration': .9 * (2 * 3.141592653589793 / 8) ** 2,
        'max_age': .4,
        'horizon': 1.0,
    }
    reverse = yaml.safe_load(yaml.safe_dump(profile))
    reverse_follow = reverse['controller_server']['ros__parameters']['FollowPath']
    del reverse_follow['PredictionV1Critic']
    assert reverse_follow['critics'].pop() == 'PredictionV1Critic'
    assert reverse == baseline, 'Candidate changed more than the critic'
    return profile


def prepare():
    SERIES.mkdir(parents=True, exist_ok=False)
    image = subprocess.check_output(
        ['docker', 'image', 'inspect', '--format', '{{.Id}}', 'rm2027_navigation:humble'],
        text=True).strip()
    old_manifest = json.loads((ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/manifest.json').read_text())['files_sha256']
    assert all(sha(ROOT / name) == digest for name, digest in old_manifest.items())
    baseline = base_profile()
    candidate = candidate_profile(baseline)
    plan = {
        'schema': 'rm_dynamic_prediction_navfn_transfer/v1',
        'scope': 'Separate Navfn planner transfer probe, one phase-0 baseline then one phase-0 V1 candidate; not the T-DT matrix.',
        'phase_seconds': 0,
        'run_order': ['baseline_navfn_1', 'candidate_navfn_1'],
        'stop_rule': 'Retain first startup or evidence failure; no repeat or favorable-phase selection.',
        'goal': [5.6, 0., 0.],
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'image_id': image,
        'plugin_so_sha256': sha(WORK / 'dynamic_prediction_critic_build/install/rm_dynamic_prediction_critic/lib/librm_dynamic_prediction_critic.so'),
        'input_sha256': {str(p.relative_to(ROOT)): sha(p) for p in (
            TDT_PROFILE, NAVFN_PROFILE, RUN_TRIAL, HERE / 'run_pair.py',
            ROOT / 'docs/tdt_migration/evidence/dynamic_reference_20260922/dynamic.launch.py',
            ROOT / 'docs/dynamic_navigation/evidence/v1_tracker_probe_20260924/live_nodes.py',
            ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/static_map.py',
            ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/audit.py',
            ROOT / 'src/rm_simulation/models/moving_obstacle.sdf',
            ROOT / 'experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp',
            ROOT / 'experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/include/rm_dynamic_prediction_critic/geometry.hpp')},
        'old_manifest_entries_verified': len(old_manifest),
        'accepted_for_deployment': False,
    }
    for mode, profile in (('baseline', baseline), ('candidate', candidate)):
        trial = SERIES / f'{mode}_navfn_1'
        trial.mkdir(exist_ok=False)
        with (trial / 'profile.yaml').open('x') as f:
            yaml.safe_dump(profile, f, sort_keys=False)
        metadata = {**plan, 'mode': mode, 'profile_sha256': sha(trial / 'profile.yaml'),
                    'file_sha256': plan['input_sha256']}
        with (trial / 'metadata.json').open('x') as f:
            json.dump(metadata, f, indent=2)
            f.write('\n')
    with (SERIES / 'plan.json').open('x') as f:
        json.dump(plan, f, indent=2)
        f.write('\n')
    print(json.dumps({'series': str(SERIES), 'profiles': {
        mode: sha(SERIES / f'{mode}_navfn_1/profile.yaml') for mode in ('baseline', 'candidate')}}))


def run(mode):
    assert mode in ('baseline', 'candidate')
    trial = SERIES / f'{mode}_navfn_1'
    assert trial.is_dir() and not (trial / 'docker_exit.txt').exists()
    metadata = json.loads((trial / 'metadata.json').read_text())
    assert sha(trial / 'profile.yaml') == metadata['profile_sha256']
    for name, digest in metadata['input_sha256'].items():
        assert sha(ROOT / name) == digest, name
    target = '/work/' + str(trial.relative_to(WORK))
    args = ['docker', 'run', '--rm', '--init', '--network', 'none', '--user',
            f'{os.getuid()}:{os.getgid()}', '--entrypoint', 'bash',
            '-v', f'{ROOT}:/ws:ro', '-v', f'{WORK}:/work',
            '--cidfile', str(trial / 'container_id')]
    for env in ('ROS_DOMAIN_ID=174', 'ROS_LOCALHOST_ONLY=1', 'PYTHONDONTWRITEBYTECODE=1',
                'TMPDIR=/work/tmp', 'LIBGL_ALWAYS_SOFTWARE=true', 'QT_QPA_PLATFORM=offscreen',
                'TDT_PHASE_SECONDS=0', f'IGN_PARTITION=dynamic_prediction_navfn_transfer_v1_{mode}'):
        args.extend(('-e', env))
    args += ['rm2027_navigation:humble', '/ws/' + str(RUN_TRIAL.relative_to(ROOT)),
             target, target + '/profile.yaml', mode]
    status = subprocess.call(args)
    with (trial / 'docker_exit.txt').open('x') as f:
        f.write(f'{status}\n')
    print(json.dumps({'mode': mode, 'docker_exit': status}), flush=True)
    sys.path.insert(0, str(AUDIT_PATH))
    import audit
    audit.SERIES = SERIES
    result = audit.trial(mode, 'navfn')
    with (trial / 'audit.json').open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({'mode': mode, 'evidence_valid': result['evidence_valid'],
                      'action': result['action_status'], 'recoveries': result['recoveries'],
                      'body_bound_m': result['geometry']['body']['moving_interpolation_bound_m'],
                      'padded_bound_m': result['geometry']['padded']['moving_interpolation_bound_m'],
                      'limited_dynamic_gate_pass': result['limited_dynamic_gate_pass']}), flush=True)


if __name__ == '__main__':
    command = sys.argv[1]
    if command == 'prepare':
        prepare()
    elif command in ('baseline', 'candidate'):
        run(command)
    else:
        raise ValueError(command)
