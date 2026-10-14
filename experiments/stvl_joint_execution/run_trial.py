"""Finite owned-process supervisor; no velocity ownership implementation."""
import json,os,pathlib,re,shutil,signal,subprocess,sys,time
import yaml
parts=sys.argv[1].split(':');scene,mode,repeat=parts[:3];tag='_'+parts[3] if len(parts)>3 else '';assert scene in ('empty','crossing','head_on','narrow','wait_release','sudden_stop','stale') and mode in ('A','B','C','D','CAL');repeat=int(repeat)
root=pathlib.Path.cwd();out=pathlib.Path('/check/runs')/(f'{scene}_{mode}_{repeat:02d}'+tag);out.mkdir(parents=True,exist_ok=False)
profile='common'
config_path=pathlib.Path('/check/assets')/('common_nav2.yaml' if profile=='common' else f'{profile}_{mode}_nav2.yaml')
config=yaml.safe_load(config_path.read_text());parameters=config['controller_server']['ros__parameters']['FollowPath']
parameters.update(research_mode=mode,research_log=str(out/'control.jsonl'),research_seed=repeat);
if mode in ('B','C','D'): parameters['critics'].append('TemporalResearchCritic');parameters['TemporalResearchCritic']={'enabled':True,'cost_weight':1.,'topic':'/research/time_predictions'}
if mode in ('B','C','D') and tag=='_aligned':parameters['TemporalResearchCritic']['prediction_epoch']='input_pose'
parameters['critics']=[('RootCostProbe' if x=='CostCritic' else x) for x in parameters['critics']]
parameters['RootCostProbe']=parameters.pop('CostCritic')
parameters['RootCostProbe']['check_all_poses']=True
parameters['RootCostProbe']['trace_alignment']=False
parameters.update(research_capture_begin=2.6,research_capture_end=2.9)
parameters['research_crash_trace']=False
if True:
 config['controller_server']['ros__parameters']['odom_topic']='/odometry/lio'
(out/'nav2.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
command=['ros2','launch',str(root/'experiments/stvl_temporal/comparison.launch.py'),'enabled:=true',f'scene:={scene}',f'seed:={repeat}',f'output:={out}']
(out/'manifest.json').write_text(json.dumps(dict(scene=scene,mode=mode,repeat=repeat,profile=profile,variant='input_pose' if tag=='_aligned' else 'consume',native_recovery_enabled=os.environ.get('R4_RESEARCH_NATIVE_RECOVERY','0')=='1',phase='calibration' if mode=='CAL' else 'finite' if repeat in (13,29,47) else 'pilot',launch=command,baseline='353fea2a54e6df858664a831dc394fb04e497223',control_intervention=False,study='joint_readonly_fresh',heldout=repeat==83,gazebo_seed=None,native_noise_seed=repeat),indent=2)+'\n')
if pathlib.Path('/check/assets/scenario.json').exists():
 for source,target in [(f'{scene}.sdf','scene.sdf'),('scenario.json','scenario.json'),(f'{scene}.pgm','map.pgm'),(f'{scene}.yaml','map.yaml')]:shutil.copy2(pathlib.Path('/check/assets')/source,out/target)
 if mode=='CAL':shutil.copy2(pathlib.Path('/check/assets')/f'{scene}_CAL.sdf',out/'scene.sdf')
processes=[];streams=[];status=1
try:
 for name,args in [('launch',command),('tap',[sys.executable,str(root/'experiments/stvl_joint_execution/tap.py'),str(out)]),('record',[sys.executable,str(root/'experiments/stvl_temporal/record.py'),scene,mode,str(out)])]:
  stream=(out/f'{name}.log').open('w');streams.append(stream)
  processes.append(subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
 status=processes[-1].wait(timeout=150)
 # Only inspect the actual sim boundary; no full graph/regression inventory.
 with (out/'output_endpoints.txt').open('w') as f:
  subprocess.run(['ros2','topic','info','-v','/simulation/chassis/cmd_vel'],stdout=f,stderr=subprocess.STDOUT,timeout=5,check=False)
 if True:
  with (out/'controller_odom_contract.txt').open('w') as f:
   subprocess.run(['ros2','param','get','/controller_server','odom_topic'],stdout=f,stderr=subprocess.STDOUT,timeout=5,check=False)
   subprocess.run(['ros2','topic','info','-v','/odometry/lio'],stdout=f,stderr=subprocess.STDOUT,timeout=5,check=False)
 if scene!='empty' and repeat==101:
  with (out/'contact_gz_endpoint.txt').open('w') as f:
   subprocess.run(['ign','topic','-i','-t','/simulation/oracle/contacts'],stdout=f,stderr=subprocess.STDOUT,timeout=3,check=False)
except Exception as exc: (out/'supervisor_error.txt').write_text(repr(exc)+'\n')
finally:
 for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
  for proc in processes:
   if proc.poll() is None:
    try: os.killpg(proc.pid,sig)
    except ProcessLookupError: pass
  for proc in processes:
   try: proc.wait(timeout=3)
   except subprocess.TimeoutExpired: pass
 for stream in streams: stream.close()
(out/'supervisor.json').write_text(json.dumps({'record_exit':status})+'\n')
print(out.name,'record_exit',status,flush=True)
sys.exit(status)
