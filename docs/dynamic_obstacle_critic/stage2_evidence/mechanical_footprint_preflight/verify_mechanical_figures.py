import hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation'); archive=repo/'docs/dynamic_obstacle_critic/stage2_evidence';result=[]
for name,flags in [('gazebo_soft_map_clearance',[]),('gazebo_soft_clearance_performance',['--mechanical'])]:
 target=Path('/tmp/rm_dynamic_critic_v1/mechanical_figure_regression')/name
 shutil.copytree(archive/name,target)
 env=dict(os.environ,MPLCONFIGDIR='/tmp/rm_dynamic_critic_v1/mpl_config',PYTHONDONTWRITEBYTECODE='1')
 subprocess.run([sys.executable,'-B',str(repo/'src/rm_dynamic_obstacle_critic/tools/plot_contact_witness.py'),str(target),*flags],env=env,check=True)
 for filename in ['contact_witness.png','contact_witness.svg']:
  exact=(target/filename).read_bytes()==(archive/name/filename).read_bytes();assert exact,(name,filename)
  result.append({'trial':name,'figure':filename,'byte_exact':exact})
Path('/tmp/rm_dynamic_critic_v1/mechanical_figure_regression.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
