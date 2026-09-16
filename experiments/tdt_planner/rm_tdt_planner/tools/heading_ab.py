#!/usr/bin/env python3
"""Isolated heading A/B; reuse the P2B fixture, observer and independent static audit."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import yaml
from summarize_simulation import digest, inspect_trial, read_rows

REPO = Path(__file__).resolve().parents[4]
PKG = Path(__file__).resolve().parents[1]
WORK = REPO / 'build/tdt_p2b'
VARIANTS = ('baseline', 'path_heading_follow')
FIXTURES = ('src/rm_simulation/worlds/phase1_omni.sdf', 'src/rm_simulation/models/course_wall.sdf',
            'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml',
            'experiments/tdt_planner/rm_tdt_planner/launch/simulation_comparison.launch.py')
POLICY_COMMIT = '828d4d898d61444ed2a5fb419e02add3421b4ced'


def write_new(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def profiles():
    base_path = WORK / 'profiles_p2b/tdt_qp.yaml'
    base = yaml.safe_load(base_path.read_text())
    candidate = copy.deepcopy(base)
    candidate['planner_server']['ros__parameters']['GridBased']['experimental_path_heading'] = True
    controller = candidate['controller_server']['ros__parameters']['FollowPath']
    # Exact existing chassis-heading-motion-policy values, scoped to this candidate.
    controller['PathAngleCritic'].update(forward_preference=True, cost_weight=6.0,
                                         max_angle_to_furthest=.20)
    controller['critics'] += ['TwirlingCritic', 'PreferForwardCritic']
    controller['TwirlingCritic'] = {'enabled': True, 'cost_power': 1, 'cost_weight': 1.0}
    controller['PreferForwardCritic'] = {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
                                         'threshold_to_consider': .5}
    # Unlike PathAngle (point direction), PathAlign actually reads per-pose path yaw.
    controller['PathAlignCritic']['use_path_orientations'] = True
    return base_path, {'baseline': base, 'path_heading_follow': candidate}


def prepare(root):
    base_path, variants = profiles()
    root.mkdir(parents=True, exist_ok=False)
    (root / 'profiles').mkdir()
    manifest = {'policy_commit': POLICY_COMMIT, 'baseline_profile_sha256': digest(base_path),
                'profiles': {}, 'scope': 'QP heading bundle A/B; same circle collision model and static gates'}
    for variant, config in variants.items():
        path = root / 'profiles' / (variant + '.yaml')
        path.write_text(base_path.read_text() if variant == 'baseline' else yaml.safe_dump(config, sort_keys=False))
        manifest['profiles'][variant] = digest(path)
    write_new(root / 'profiles/manifest.json', manifest)


def verify_profiles(root):
    baseline, expected = profiles()
    manifest = json.loads((root / 'profiles/manifest.json').read_text())
    if digest(baseline) != manifest['baseline_profile_sha256']:
        raise ValueError('historical baseline profile changed')
    for name, config in expected.items():
        path = root / 'profiles' / (name + '.yaml')
        if digest(path) != manifest['profiles'][name] or yaml.safe_load(path.read_text()) != config:
            raise ValueError('heading profile differs from frozen generator: ' + name)
    return manifest


def run(root, variant, trial):
    manifest = verify_profiles(root)
    source_paths = [str(PKG.relative_to(REPO)), 'src/rm_simulation', 'src/rm_nav_config',
                    'src/rm_description', 'src/rm_localization_adapters',
                    'src/rm_chassis_interface', 'src/rm_competition_interfaces']
    if subprocess.check_output(['git', 'status', '--porcelain', '--', *source_paths], cwd=REPO):
        raise ValueError('commit relevant experiment sources before running')
    if trial > 1:
        for previous in range(1, trial):
            old = inspect_trial(root / variant / f'tdt_qp_{previous}')
            if not (old.get('raw_summary_matches') and old['summary'].get('static_geometry_and_goal_pass')):
                raise ValueError('previous trial did not pass; group repetitions stopped')
    target = root / variant / f'tdt_qp_{trial}'
    target.mkdir(parents=True, exist_ok=False)
    profile = root / 'profiles' / (variant + '.yaml')
    (target/'profile.yaml').write_bytes(profile.read_bytes())
    image = subprocess.check_output(['docker', 'image', 'inspect', 'rm2027_navigation:humble',
                                    '--format', '{{.Id}}'], text=True).strip()
    write_new(target/'metadata.json', {'planner': 'tdt_qp', 'trial': trial, 'heading_variant': variant,
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
        'image_id': image, 'profile_sha256': manifest['profiles'][variant],
        'fixture_sha256': {f: digest(REPO/f) for f in FIXTURES},
        'scope': 'static heading bundle A/B, goal (4.3,0,0), no hardware',
        'performance_is_not_algorithm_rejection': True})
    relative = '/work/' + str(target.relative_to(WORK))
    args = ['docker', 'run', '--rm', '--init', '--network', 'none', '--user', f'{os.getuid()}:{os.getgid()}',
            '--entrypoint', 'bash', '-v', f'{REPO}:/ws:ro', '-v', f'{WORK}:/work',
            '--cidfile', str(target/'container_id'), '--label', f'rm.tdt.p2b.series={root.name}']
    for value in ('ROS_DOMAIN_ID=174', 'ROS_LOCALHOST_ONLY=1', 'PYTHONDONTWRITEBYTECODE=1',
                  'TMPDIR=/work/tmp', 'LIBGL_ALWAYS_SOFTWARE=true', 'QT_QPA_PLATFORM=offscreen',
                  'TDT_HEADING_AB=1', f'IGN_PARTITION=tdt_{root.name}_{variant}_{trial}'):
        args += ['-e', value]
    args += ['rm2027_navigation:humble', '/ws/' + str(PKG.relative_to(REPO)) + '/tools/run_simulation_trial.sh',
             relative, relative + '/profile.yaml']
    code = subprocess.call(args)
    (target/'docker_exit.txt').write_text(str(code)+'\n')
    print(f'{variant} {trial}: exit={code}, evidence={target}', flush=True)
    return code


def rates(rows):
    if len(rows) < 2 or not all(math.isfinite(r[k]) for r in rows for k in ('t', 'wz')):
        raise ValueError('missing/nonfinite angular velocity evidence')
    pairs = [(a, b, b['t']-a['t']) for a, b in zip(rows, rows[1:])]
    if any(dt < 0 for _, _, dt in pairs):
        raise ValueError('angular velocity clock reversed')
    timed = [(a, b, dt) for a, b, dt in pairs if dt > 1e-6]
    if not timed:
        raise ValueError('no positive angular velocity intervals')
    duration = sum(dt for _, _, dt in timed)
    variation = [abs(b['wz']-a['wz']) for a, b, _ in pairs]
    return {'max_abs_wz_rad_s': max(abs(r['wz']) for r in rows),
            'time_weighted_rms_wz_rad_s': math.sqrt(sum(.5*(a['wz']**2+b['wz']**2)*dt for a,b,dt in timed)/duration),
            'max_abs_dwz_dt_rad_s2': max(abs(b['wz']-a['wz'])/dt for a,b,dt in timed),
            'max_abs_delta_wz_rad_s': max(variation),
            'total_variation_wz_rad_s': sum(variation),
            'zero_dt_intervals_excluded_from_derivative': len(pairs)-len(timed),
            'sample_count': len(rows), 'duration_s': duration}


def yaw_metrics(path):
    samples = read_rows(path/'observation/trajectory.jsonl')
    commands = [r for r in read_rows(path/'observation/commands.jsonl')
                if samples[0]['t'] <= r['t'] <= samples[-1]['t']+.1]
    plans = {r['id']: r for r in read_rows(path/'observation/plans.jsonl')}
    errors, motion_errors = [], []
    max_yaw_step = 0
    for plan in plans.values():
        if len(plan.get('yaw', [])) != len(plan['xy']) or not all(math.isfinite(v) for v in plan['yaw']):
            raise ValueError('missing/nonfinite plan yaw evidence')
        max_yaw_step = max([max_yaw_step] + [abs(math.remainder(b-a,2*math.pi))
                                           for a,b in zip(plan['yaw'],plan['yaw'][1:])])
    for a,b in zip(samples,samples[1:]):
        dt=b['t']-a['t']; dx=b['x']-a['x']; dy=b['y']-a['y']
        if dt <= 0 or math.hypot(dx,dy)/dt <= .10:
            continue
        motion_errors.append(abs(math.remainder(a['yaw']-math.atan2(dy,dx),2*math.pi)))
        plan=plans.get(a['plan_id'])
        if not plan or len(plan['xy']) < 2:
            continue
        candidates=[]
        for i,(p,q) in enumerate(zip(plan['xy'],plan['xy'][1:])):
            x,y=q[0]-p[0],q[1]-p[1]; den=x*x+y*y
            t=max(0.,min(1.,((a['x']-p[0])*x+(a['y']-p[1])*y)/den)) if den else 0.
            d=math.hypot(a['x']-p[0]-t*x,a['y']-p[1]-t*y)
            yaw=plan['yaw'][i]+t*math.remainder(plan['yaw'][i+1]-plan['yaw'][i],2*math.pi)
            candidates.append((d,yaw))
        errors.append(abs(math.remainder(a['yaw']-min(candidates)[1],2*math.pi)))
    def stats(values):
        return {'samples': len(values), 'rms_rad': math.sqrt(statistics.mean(x*x for x in values)),
                'median_rad': statistics.median(values), 'p95_rad': sorted(values)[math.ceil(.95*len(values))-1]} if values else None
    return {'ground_truth': rates(samples), 'command': rates(commands),
            'translating_yaw_to_path': stats(errors), 'translating_yaw_to_motion': stats(motion_errors),
            'max_plan_adjacent_yaw_step_rad': max_yaw_step,
            'scope': 'ground truth and commanded wz kept separate; derivatives use sim time; includes recovery and settle'}


def summarize(root, output):
    manifest=verify_profiles(root)
    reports=[]
    for variant in VARIANTS:
        for path in sorted((root/variant).glob('tdt_qp_*')):
            r=inspect_trial(path)
            r['heading_variant']=variant
            if r.get('raw_summary_matches'):
                events=read_rows(path/'observation/events.jsonl')
                runtime=[e for e in events if e.get('event')=='runtime_parameters']
                r['runtime_parameters_verified']=len(runtime)==2 and all(e['actual']==e['expected'] for e in runtime)
                r['yaw_metrics']=yaw_metrics(path)
                # Verify recorded expected parameters against the frozen full profile, not just each other.
                expected=yaml.safe_load((root/'profiles'/(variant+'.yaml')).read_text())
                expected['planner_server']['ros__parameters']['GridBased'].setdefault('experimental_path_heading', False)
                for e in runtime:
                    for key,value in e['expected'].items():
                        current=expected[e['node']]['ros__parameters']
                        for part in key.split('.'):
                            current=current[part]
                        if current != value:
                            raise ValueError('runtime request differs from frozen profile')
            reports.append(r)
    identities={json.dumps([r.get('source_commit'),r.get('image_id'),r.get('fixture_sha256')],sort_keys=True) for r in reports}
    audited=bool(reports and len(identities)==1 and all(r.get('raw_summary_matches') and
        r.get('runtime_parameters_verified') and r['profile_sha256']==manifest['profiles'][r['heading_variant']] for r in reports))
    report={'schema':'rm_tdt_planner/heading_ab/v1','consistent_and_audited':audited,
            'recorded':len(reports),'both_variants_present':{r['heading_variant'] for r in reports}==set(VARIANTS),
            'all_recorded_static_checks_pass':bool(audited and all(r['summary']['static_geometry_and_goal_pass'] for r in reports)),
            'trials':reports,'accepted_for_deployment':False,
            'attribution':'Bundled path yaw and existing path_aligned policy; cannot isolate individual critic effects.'}
    write_new(output,report)
    print(f"Saved {output}; audited={audited}, trials={len(reports)}")
    return 0 if audited and report['both_variants_present'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=('prepare','run','summarize'))
    p.add_argument('--series',default='heading_follow_v1')
    p.add_argument('--variant',choices=VARIANTS)
    p.add_argument('--trial',type=int,choices=range(1,6),default=1)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if not a.series.replace('_','').isalnum():
        p.error('series must use letters/digits/underscores')
    root=WORK/'runs'/a.series
    if a.mode=='prepare': prepare(root); return 0
    if a.mode=='run':
        if not a.variant: p.error('--variant required')
        return run(root,a.variant,a.trial)
    if not a.output: p.error('--output required')
    return summarize(root,a.output)

if __name__=='__main__':
    sys.exit(main())
