"""Finite owned-process supervisor; no velocity ownership implementation."""
import json,os,pathlib,re,shutil,signal,subprocess,sys,time
import yaml
scene,mode,repeat=sys.argv[1].split(':');assert scene in ('S0','S1','S2','S3','S4') and mode in ('B0','R4');repeat=int(repeat)
root=pathlib.Path.cwd();out=pathlib.Path('/check/runs')/f'{scene}_{mode}_{repeat:02d}';out.mkdir(parents=True,exist_ok=False)
profile=os.environ.get('R4_COMP_PROFILE','common');assert re.fullmatch(r'[a-z0-9_]+',profile)
config_path=pathlib.Path('/check/assets')/('common_nav2.yaml' if profile=='common' else f'{profile}_{mode}_nav2.yaml')
config=yaml.safe_load(config_path.read_text());parameters=config['controller_server']['ros__parameters']['FollowPath']
parameters.update(research_mode=mode,research_log=str(out/'control.csv'));(out/'nav2.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
command=['ros2','launch',str(root/'experiments/r4_gazebo_comparison/comparison.launch.py'),'enabled:=true',f'scene:={scene}',f'seed:={repeat}',f'output:={out}']
(out/'manifest.json').write_text(json.dumps(dict(scene=scene,mode=mode,repeat=repeat,profile=profile,phase='calibration' if profile.startswith('calibration_') else 'finite' if repeat>=100 else 'pilot',launch=command,baseline='ccd3eac4',gazebo_seed=None,native_noise_seed=None),indent=2)+'\n')
if pathlib.Path('/check/assets/scenario.json').exists():
 for source,target in [(f'{scene}.sdf','scene.sdf'),('scenario.json','scenario.json'),('empty.pgm','empty.pgm'),('map.yaml','map.yaml')]:shutil.copy2(pathlib.Path('/check/assets')/source,out/target)
processes=[];streams=[];status=1
try:
 for name,args in [('launch',command),('record',[sys.executable,str(root/'experiments/r4_gazebo_comparison/record.py'),scene,mode,str(out)])]:
  stream=(out/f'{name}.log').open('w');streams.append(stream)
  processes.append(subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
 status=processes[-1].wait(timeout=150)
 # Only inspect the actual sim boundary; no full graph/regression inventory.
 with (out/'output_endpoints.txt').open('w') as f:
  subprocess.run(['ros2','topic','info','-v','/simulation/chassis/cmd_vel'],stdout=f,stderr=subprocess.STDOUT,timeout=5,check=False)
 if scene!='S0' and repeat==101:
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
