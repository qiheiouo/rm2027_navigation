#!/usr/bin/env python3
"""Finite test supervisor, never a robot command owner."""
import hashlib, json, os, pathlib, signal, subprocess, sys
root=pathlib.Path.cwd();scene=sys.argv[1];out=pathlib.Path('/check')/scene
out.mkdir(exist_ok=False);processes=[];files=[]
def start(name,args):
    f=(out/(name+'.log')).open('w');files.append(f)
    p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
cmd=['ros2','launch',str(root/'experiments/r4_runtime_shadow/shadow.launch.py'),'enabled:=true',f'scene:={scene}',f'output:={out}']
manifest=dict(scene=scene,launch_command=cmd,scenario_command=['python3',str(root/'experiments/r4_runtime_shadow/scenario.py'),scene,str(out)],environment={k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION','IGN_IP','LIBGL_ALWAYS_SOFTWARE']},sources={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (root/'experiments/r4_runtime_shadow').iterdir() if f.is_file()},caller_binary_sha256=hashlib.sha256(pathlib.Path('/check/caller_build/r4_shadow').read_bytes()).hexdigest())
(out/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
status=1
try:
    start('bag',['ros2','bag','record','-o',str(out/'rosbag'),'/clock','/scan','/map','/odometry/lio','/tf','/tf_static','/plan','/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel','/perception/dynamic_obstacles_shadow/predictions','/perception/dynamic_obstacles_shadow/observed_predictions'])
    start('launch',cmd)
    driver=start('scenario',manifest['scenario_command']);status=driver.wait(timeout=100)
    for filename,args in [('topics.txt',['ros2','topic','list']),('output_endpoints.txt',['ros2','topic','info','-v','/cmd_vel']),('prediction_endpoints.txt',['ros2','topic','info','-v','/perception/dynamic_obstacles_shadow/observed_predictions'])]:
        with (out/filename).open('w') as f:subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,timeout=8,check=False)
except Exception as e:
    (out/'supervisor_error.txt').write_text(repr(e)+'\n')
finally:
    # Signal entire owned sessions; all escalation waits are bounded.
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGKILL]:
        for p in processes:
            if p.poll() is None:
                try:os.killpg(p.pid,sig)
                except ProcessLookupError:pass
        for p in processes:
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:pass
    for f in files:f.close()
sys.exit(status)
