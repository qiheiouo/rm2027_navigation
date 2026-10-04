import concurrent.futures, os, subprocess
from pathlib import Path
work=Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004')
root=work/'build/temporal_mpc_candidates_20261004'
names=['crossing_b0_01','crossing_single_01','crossing_portfolio_01','head_on_b0_01','head_on_b0_02','head_on_single_02','head_on_portfolio_02']
env=dict(os.environ,PYTHONPATH='/tmp/temporal_mpc_deps_20261004:'+str(work/'experiments/temporal_mpc'))
def analyze(name):
 for script in ['audit_execution','audit_model_occupancy','audit_reanchor_inputs']:
  with (root/name/(script+'.log')).open('w') as log:
   subprocess.run(['python3',str(work/'experiments/temporal_mpc/gazebo'/(script+'.py')),str(root/name)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 print(name+' read-only audits complete',flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 list(pool.map(analyze,names))
