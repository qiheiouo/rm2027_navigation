#!/usr/bin/env python3
"""One fixed-phase repetition of the first valid Navfn+V1 collision, in a new series."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORK = ROOT / 'build/tdt_p2b'
FIRST = WORK / 'runs/dynamic_prediction_navfn_transfer_v1/candidate_navfn_1'
SERIES = WORK / 'runs/dynamic_prediction_navfn_repro_v1'
TRIAL = SERIES / 'candidate_navfn_1'
RUN_TRIAL = ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924/run_trial.sh'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    original = json.loads((FIRST / 'metadata.json').read_text())
    assert sha(FIRST / 'profile.yaml') == original['profile_sha256']
    assert sha(WORK / 'dynamic_prediction_critic_build/install/rm_dynamic_prediction_critic/lib/librm_dynamic_prediction_critic.so') == original['plugin_so_sha256']
    for name, digest in original['input_sha256'].items():
        assert sha(ROOT / name) == digest, name
    SERIES.mkdir(parents=True, exist_ok=False)
    TRIAL.mkdir(exist_ok=False)
    (TRIAL / 'profile.yaml').write_bytes((FIRST / 'profile.yaml').read_bytes())
    plan = {
        'schema': 'rm_dynamic_prediction_navfn_reproduction/v1',
        'purpose': 'One fixed phase-0 repetition of a valid sampled polygon collision; no outcome-based additional runs.',
        'phase_seconds': 0, 'trial': 'candidate_navfn_1',
        'reference_trial': str(FIRST.relative_to(ROOT)),
        'reference_profile_sha256': original['profile_sha256'],
        'reference_audit_sha256': sha(FIRST / 'audit.json'),
        'plugin_so_sha256': original['plugin_so_sha256'],
        'input_sha256': original['input_sha256'],
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'accepted_for_deployment': False,
    }
    with (SERIES / 'plan.json').open('x') as f:
        json.dump(plan, f, indent=2)
        f.write('\n')
    metadata = {**plan, 'mode': 'candidate', 'profile_sha256': sha(TRIAL / 'profile.yaml'),
                'file_sha256': original['file_sha256']}
    with (TRIAL / 'metadata.json').open('x') as f:
        json.dump(metadata, f, indent=2)
        f.write('\n')
    target = '/work/' + str(TRIAL.relative_to(WORK))
    args = ['docker', 'run', '--rm', '--init', '--network', 'none', '--user',
            f'{os.getuid()}:{os.getgid()}', '--entrypoint', 'bash',
            '-v', f'{ROOT}:/ws:ro', '-v', f'{WORK}:/work', '--cidfile', str(TRIAL / 'container_id')]
    for env in ('ROS_DOMAIN_ID=174', 'ROS_LOCALHOST_ONLY=1', 'PYTHONDONTWRITEBYTECODE=1',
                'TMPDIR=/work/tmp', 'LIBGL_ALWAYS_SOFTWARE=true', 'QT_QPA_PLATFORM=offscreen',
                'TDT_PHASE_SECONDS=0', 'IGN_PARTITION=dynamic_prediction_navfn_repro_v1_candidate'):
        args.extend(('-e', env))
    args += ['rm2027_navigation:humble', '/ws/' + str(RUN_TRIAL.relative_to(ROOT)),
             target, target + '/profile.yaml', 'candidate']
    status = subprocess.call(args)
    with (TRIAL / 'docker_exit.txt').open('x') as f:
        f.write(f'{status}\n')
    print(json.dumps({'docker_exit': status, 'trial': str(TRIAL)}), flush=True)
    sys.path.insert(0, str(ROOT / 'docs/dynamic_navigation/evidence/v1_mppi_ab_20260924'))
    import audit
    audit.SERIES = SERIES
    result = audit.trial('candidate', 'navfn')
    with (TRIAL / 'audit.json').open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({'action': result['action_status'], 'recoveries': result['recoveries'],
                      'body_sampled_overlap': result['geometry']['body']['sampled_moving_overlap'],
                      'body_bound_m': result['geometry']['body']['moving_interpolation_bound_m'],
                      'padded_bound_m': result['geometry']['padded']['moving_interpolation_bound_m'],
                      'limited_dynamic_gate_pass': result['limited_dynamic_gate_pass']}), flush=True)


if __name__ == '__main__':
    main()
