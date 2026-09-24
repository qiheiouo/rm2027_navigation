#!/usr/bin/env python3
"""First-created, fixed-phase QP A/B. Run modes separately and preserve failures."""
import hashlib,json,os,subprocess,sys
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
WORK=ROOT/'build/tdt_p2b'
SERIES=WORK/'runs/dynamic_prediction_mppi_ab_v2'
BASE=WORK/'runs/dynamic_odom_routing_pilot_v1/profiles/tdt_qp.yaml'
EXPECTED='1862d4dc9742f10d8d1db1d4404ef5f8123c7184e876c7e6179410ad9f3f9505'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def profile(mode):
    assert sha(BASE)==EXPECTED
    original=yaml.safe_load(BASE.read_text())
    if mode=='baseline':return BASE.read_bytes()
    new=yaml.safe_load(BASE.read_text())
    mppi=new['controller_server']['ros__parameters']['FollowPath']
    mppi['critics'].append('PredictionV1Critic')
    mppi['PredictionV1Critic']={
      'enabled':True,'topic':'/perception/dynamic_obstacles_shadow/predictions',
      'object_width':.45,'object_height':.55,
      'reference_acceleration':.9*(2*3.141592653589793/8)**2,
      'max_age':.4,'horizon':1.0}
    assert original['controller_server']['ros__parameters']['odom_topic']=='/odometry/lio'
    comparison=yaml.safe_load(yaml.safe_dump(new))
    del comparison['controller_server']['ros__parameters']['FollowPath']['PredictionV1Critic']
    comparison['controller_server']['ros__parameters']['FollowPath']['critics'].pop()
    assert comparison==original, 'candidate changed more than the critic'
    return yaml.safe_dump(new,sort_keys=False).encode()
def run(mode):
    if mode not in ('baseline','candidate'):raise ValueError(mode)
    trial=SERIES/f'{mode}_tdt_qp_1'
    trial.mkdir(parents=True,exist_ok=False)
    data=profile(mode)
    (trial/'profile.yaml').write_bytes(data)
    files=[HERE/n for n in ('run_ab.py','run_trial.sh','static_map.py')]
    files += [ROOT/p for p in ('src/rm_simulation/models/moving_obstacle.sdf',
      'src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py',
      'src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py',
      'experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp',
      'experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/include/rm_dynamic_prediction_critic/geometry.hpp')]
    meta={'schema':'rm_dynamic_prediction_mppi_ab/v1','mode':mode,
      'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
      'runtime_tdt_source_commit':'a419654a8fed1fc8321c234fb212abd0a6cabe04',
      'baseline_profile_sha256':EXPECTED,'profile_sha256':sha(trial/'profile.yaml'),
      'file_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},
      'phase_seconds':0,'goal':[5.6,0.,0.], 'accepted_for_deployment':False}
    (trial/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    target='/work/'+str(trial.relative_to(WORK))
    args=['docker','run','--rm','--init','--network','none','--user',f'{os.getuid()}:{os.getgid()}',
      '--entrypoint','bash','-v',f'{ROOT}:/ws:ro','-v',f'{WORK}:/work','--cidfile',str(trial/'container_id')]
    for env in ('ROS_DOMAIN_ID=174','ROS_LOCALHOST_ONLY=1','PYTHONDONTWRITEBYTECODE=1',
      'TMPDIR=/work/tmp','LIBGL_ALWAYS_SOFTWARE=true','QT_QPA_PLATFORM=offscreen',
      'TDT_PHASE_SECONDS=0',f'IGN_PARTITION=dynamic_prediction_mppi_ab_v2_{mode}'):
        args.extend(('-e',env))
    args += ['rm2027_navigation:humble','/ws/'+str((HERE/'run_trial.sh').relative_to(ROOT)),
             target,target+'/profile.yaml',mode]
    status=subprocess.call(args)
    (trial/'docker_exit.txt').write_text(f'{status}\n')
    print(json.dumps({'trial':str(trial),'docker_exit':status}))
if __name__=='__main__':run(sys.argv[1])
