import json,os,shutil,subprocess,sys
from pathlib import Path
root=Path('/tmp/rm_observation_diameter_gazebo');tools=root/'analysis_tools'
repo=Path('/home/qihei/rm2027_navigation/src/rm_dynamic_obstacle_critic/tools')
for name in ['audit_cost_accumulator.py','plot_cost_accumulator.py']:shutil.copyfile(repo/name,tools/name)
os.environ['MPLCONFIGDIR']=str(root/'host_mpl_config');os.environ['PYTHONDONTWRITEBYTECODE']='1'
for mode in ['visible','nominal']:
    trial=root/f'gazebo_observation_v2_{mode}';verify=trial/'verification'
    commands=[('strict_radius_policy.host',[sys.executable,'-B',str(tools/'audit_cost_accumulator.py'),str(trial),str(trial/'strict_radius_policy_verified.json')]),
      ('plot_cost.host',[sys.executable,'-B',str(tools/'plot_cost_accumulator.py'),str(trial),'--title',f'V2 {mode}: observed costs; fixed-scene task FAILED']),
      ('plot_dynamic.host',[sys.executable,'-B',str(tools/'plot_dynamic_consumption.py'),str(trial),str(trial/'native_optimizer/weights_output.jsonl')]),
      ('plot_mechanical.host',[sys.executable,'-B',str(tools/'plot_mechanical_contract_trial.py'),str(trial),'--title',f'V2 {mode}: fixed-scene task FAILED','--fit-near-geometry'])]
    for name,cmd in commands:
        with (verify/(name+'.log')).open('w') as stream:subprocess.run(cmd,check=True,stdout=stream,stderr=subprocess.STDOUT)
        if name.startswith('strict'):
            assert (trial/'strict_radius_policy_verified.json').read_bytes()==(trial/'cost_accumulator_audit.json').read_bytes()
        print(mode,name,'PASS',flush=True)
(root/'host_plot_provenance.json').write_text(json.dumps({'python':sys.version,'matplotlib':__import__('matplotlib').__version__,'scope':'Host plotting only; SDK risk/control exactness verified separately'},indent=2)+'\n')
