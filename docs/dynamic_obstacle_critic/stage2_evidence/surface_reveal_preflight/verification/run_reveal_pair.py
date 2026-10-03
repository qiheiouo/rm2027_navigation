import subprocess,sys
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');tools=repo/'src/rm_dynamic_obstacle_critic/tools';out=Path('/tmp/rm_surface_reveal_initial');out.mkdir()
for mode in ['visible','nominal']:
    root=repo/'docs/dynamic_obstacle_critic/stage2_evidence'/('gazebo_observation_v2_'+mode)
    with (out/(mode+'.log')).open('x') as stream:subprocess.run([sys.executable,'-B',str(tools/'audit_surface_reveal.py'),str(root),str(out/(mode+'.json'))],stdout=stream,stderr=subprocess.STDOUT,check=True)
    print(mode,'complete',flush=True)
