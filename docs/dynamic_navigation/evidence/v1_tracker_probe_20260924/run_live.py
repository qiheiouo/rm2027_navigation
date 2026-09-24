#!/usr/bin/env python3
"""Run one shadow-only QP fixture probe in a new isolated directory."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT / 'build/tdt_p2b'
SERIES = WORK / 'runs/dynamic_prediction_tracker_live_v2'
PROFILE = WORK / 'runs/dynamic_odom_routing_pilot_v1/profiles/tdt_qp.yaml'
PROFILE_SHA = '1862d4dc9742f10d8d1db1d4404ef5f8123c7184e876c7e6179410ad9f3f9505'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert sha(PROFILE) == PROFILE_SHA
    assert not subprocess.check_output(['git', 'status', '--porcelain', '--', 'experiments', 'src'], cwd=ROOT, text=True)
    trial = SERIES / 'tdt_qp_1'
    trial.mkdir(parents=True, exist_ok=False)
    (trial / 'profile.yaml').write_bytes(PROFILE.read_bytes())
    image = subprocess.check_output(['docker', 'image', 'inspect', 'rm2027_navigation:humble', '--format', '{{.Id}}'], text=True).strip()
    inputs = [HERE / name for name in ('probe.py', 'live_nodes.py', 'run_live.py', 'run_live.sh')]
    inputs += [ROOT / p for p in (
        'src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py',
        'src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py',
        'src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml',
        'src/rm_simulation/worlds/phase1_omni.sdf',
        'src/rm_simulation/models/course_wall.sdf',
        'src/rm_simulation/models/moving_obstacle.sdf',
    )]
    metadata = {'schema': 'rm_dynamic_prediction_v1_live_shadow_probe/v1',
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'runtime_tdt_source_commit': 'a419654a8fed1fc8321c234fb212abd0a6cabe04',
        'profile_sha256': PROFILE_SHA, 'image_id': image,
        'files_sha256': {str(p.relative_to(ROOT)): sha(p) for p in inputs},
        'scope': 'One fixed phase QP navigation trial with read-only tracker. This is not MPPI prediction A/B.',
        'tracker_overrides': {'use_sim_time': True, 'map_topic': '/prediction_v1/static_map',
            'scan_topic': '/scan', 'prediction.velocity_decay_tau': 0.0},
        'accepted_for_deployment': False}
    with (trial / 'metadata.json').open('x') as f:
        json.dump(metadata, f, indent=2); f.write('\n')
    container_trial = '/work/' + str(trial.relative_to(WORK))
    args = ['docker', 'run', '--rm', '--init', '--network', 'none',
        '--user', f'{os.getuid()}:{os.getgid()}', '--entrypoint', 'bash',
        '-v', f'{ROOT}:/ws:ro', '-v', f'{WORK}:/work', '--cidfile', str(trial / 'container_id')]
    for env in ('ROS_DOMAIN_ID=174', 'ROS_LOCALHOST_ONLY=1', 'PYTHONDONTWRITEBYTECODE=1',
                'TMPDIR=/work/tmp', 'LIBGL_ALWAYS_SOFTWARE=true', 'QT_QPA_PLATFORM=offscreen',
                'TDT_PHASE_SECONDS=0', 'IGN_PARTITION=dynamic_prediction_tracker_live_v2'):
        args.extend(('-e', env))
    args += ['rm2027_navigation:humble', '/ws/' + str((HERE / 'run_live.sh').relative_to(ROOT)),
             container_trial, container_trial + '/profile.yaml']
    status = subprocess.call(args)
    (trial / 'docker_exit.txt').write_text(f'{status}\n')
    print(json.dumps({'trial': str(trial), 'docker_exit': status}))


if __name__ == '__main__':
    main()
