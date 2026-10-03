import os,subprocess,sys,json
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');tools=repo/'src/rm_dynamic_obstacle_critic/tools';source=Path('/tmp/rm_surface_reveal_initial');out=Path('/tmp/rm_surface_reveal_reexecution');out.mkdir();checks={}
for mode in ('visible','nominal'):
    archive=repo/'docs/dynamic_obstacle_critic/stage2_evidence'/('gazebo_observation_v2_'+mode)
    for label,script in [('reveal','audit_surface_reveal.py'),('brake','audit_brake_response.py')]:
        name=mode+('' if label=='reveal' else '_brake')
        with (out/(name+'.log')).open('x') as log:subprocess.run([sys.executable,'-B',str(tools/script),str(archive),str(out/(name+'.json'))],stdout=log,stderr=subprocess.STDOUT,check=True)
        checks[name]=(source/(name+'.json')).read_bytes()==(out/(name+'.json')).read_bytes()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',MPLCONFIGDIR=str(out/'mpl_config'))
    with (out/(mode+'_plot.log')).open('x') as log:subprocess.run([sys.executable,'-B',str(tools/'plot_surface_reveal.py'),str(out/(mode+'.json')),str(out/(mode+'_reveal')),'--title',f'V2 {mode}: sensor reveal labels; actual braking/replanning margin UNKNOWN'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    for ext in ('png','svg'):checks[mode+'_plot_'+ext]=(source/(mode+'_reveal.'+ext)).read_bytes()==(out/(mode+'_reveal.'+ext)).read_bytes()
report={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,'scope':'Independent source-trace reexecution of reveal and empirical brake audits; actual guaranteed stopping/response remains UNKNOWN'}
(out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
if not all(checks.values()):raise SystemExit(1)
