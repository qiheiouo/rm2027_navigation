#!/usr/bin/env python3
"""Prespecified 0/2/4/6-second, first-created dynamic V1 A/B runs."""
import hashlib,json,os,subprocess,sys
from pathlib import Path
import yaml
import audit
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
WORK=ROOT/'build/tdt_p2b'
PILOT=WORK/'runs/dynamic_prediction_mppi_ab_v2'
SERIES=WORK/'runs/dynamic_prediction_mppi_multiphase_v1'
PHASES=(0,2,4,6)
PLANNERS=('tdt_qp','tdt_astar')
MODES=('baseline','candidate')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def run(planner,mode,phase):
    assert planner in PLANNERS and mode in MODES and phase in PHASES
    trial=SERIES/f'phase_{phase}'/f'{mode}_{planner}_1'
    trial.mkdir(parents=True,exist_ok=False)
    source=PILOT/f'{mode}_{planner}_1'/'profile.yaml'
    original=PILOT/f'baseline_{planner}_1'/'profile.yaml'
    baseline=yaml.safe_load(original.read_text())
    candidate=yaml.safe_load((PILOT/f'candidate_{planner}_1'/'profile.yaml').read_text())
    follow=candidate['controller_server']['ros__parameters']['FollowPath']
    assert follow['critics'][-1]=='PredictionV1Critic'
    del follow['PredictionV1Critic'];follow['critics'].pop()
    assert candidate==baseline
    assert baseline['controller_server']['ros__parameters']['odom_topic']=='/odometry/lio'
    (trial/'profile.yaml').write_bytes(source.read_bytes())
    paths=[HERE/n for n in ('run_multiphase.py','run_trial.sh','static_map.py','audit.py')]
    paths += [ROOT/p for p in ('src/rm_simulation/models/moving_obstacle.sdf',
       'experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp',
       'experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/include/rm_dynamic_prediction_critic/geometry.hpp')]
    metadata={'schema':'rm_dynamic_prediction_v1_multiphase_trial/v1','planner':planner,'mode':mode,
      'phase_seconds':phase,'profile_sha256':sha(trial/'profile.yaml'),
      'pilot_profile_sha256':sha(source),'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
      'runtime_tdt_source_commit':'a419654a8fed1fc8321c234fb212abd0a6cabe04',
      'file_sha256':{str(p.relative_to(ROOT)):sha(p) for p in paths},
      'accepted_for_deployment':False}
    (trial/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    target='/work/'+str(trial.relative_to(WORK))
    args=['docker','run','--rm','--init','--network','none','--user',f'{os.getuid()}:{os.getgid()}',
      '--entrypoint','bash','-v',f'{ROOT}:/ws:ro','-v',f'{WORK}:/work','--cidfile',str(trial/'container_id')]
    for env in ('ROS_DOMAIN_ID=174','ROS_LOCALHOST_ONLY=1','PYTHONDONTWRITEBYTECODE=1',
      'TMPDIR=/work/tmp','LIBGL_ALWAYS_SOFTWARE=true','QT_QPA_PLATFORM=offscreen',
      f'TDT_PHASE_SECONDS={phase}',f'IGN_PARTITION=dynamic_prediction_multiphase_{planner}_{mode}_{phase}'):
        args.extend(('-e',env))
    args += ['rm2027_navigation:humble','/ws/'+str((HERE/'run_trial.sh').relative_to(ROOT)),
      target,target+'/profile.yaml',mode]
    status=subprocess.call(args)
    (trial/'docker_exit.txt').write_text(f'{status}\n')
    print(json.dumps({'trial':str(trial),'docker_exit':status}),flush=True)
    audit.SERIES=trial.parent
    result=audit.trial(mode,planner)
    result['phase_seconds']=phase
    with (trial/'ab_audit.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'planner':planner,'mode':mode,'phase':phase,'action':result['action_status'],
      'recoveries':result['recoveries'],'body_bound':result['geometry']['body']['moving_interpolation_bound_m'],
      'padded_bound':result['geometry']['padded']['moving_interpolation_bound_m'],
      'pass':result['limited_dynamic_gate_pass']}),flush=True)
    return result
def main():
    SERIES.mkdir(parents=True,exist_ok=False)
    (SERIES/'matrix_plan.json').write_text(json.dumps({'phases_seconds':PHASES,'planners':PLANNERS,
      'modes':MODES,'order':'planner then phase then mode; stop a planner at its first candidate gate failure',
      'scope':'Prespecified repeat after separate fixed-phase pilots; no choosing a favorable phase.'},indent=2)+'\n')
    for planner in PLANNERS:
        for phase in PHASES:
            run(planner,'baseline',phase)
            candidate=run(planner,'candidate',phase)
            if not candidate['limited_dynamic_gate_pass']:
                print(json.dumps({'stopped_planner':planner,'first_failed_candidate_phase':phase}),flush=True)
                break
if __name__=='__main__':main()
