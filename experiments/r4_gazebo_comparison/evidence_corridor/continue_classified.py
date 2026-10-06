"""Complete the fixed schedule only after classifying already understood failures."""
import csv,json,pathlib,subprocess,sys
out=pathlib.Path(sys.argv[1]).resolve();classification_file=out/'failure_classifications.json'
for attempt in range(11):
    classifications=json.loads(classification_file.read_text());new=[]
    for directory in sorted((out/'runs').glob('S3_*')):
        if not (directory/'events.json').exists() or directory.name in classifications:continue
        e=json.loads((directory/'events.json').read_text())
        if e['finished']=='goal_result' and e['result_status']==4:continue
        assert (directory/'first_failure_inputs.json').exists(),directory.name
        if e['finished']=='actor_contact' and e['controller_failure'] is None and all('moving_obstacle' in str(c['pairs']) and 'rm_sentry_2027' in str(c['pairs']) for c in e['contacts']):
            kind='physical_dynamic_actor_contact';extra=dict(pairs=e['contacts'][0]['pairs'])
        elif e['controller_failure']=='Optimizer fail to compute path':
            control=list(csv.DictReader((directory/'control.csv').open()));last=control[-1]
            assert last['reason']==e['controller_failure'] and float(last['solver_ms'])==0 and 'Optimizer fail to compute path' in (directory/'launch.log').read_text()
            kind='native_mppi_delegation_failure';extra=dict(r4_xy_solve_called_in_failed_cycle=False,failed_cycle_solver_ms=0)
        else:raise RuntimeError('new failure type requires inspection: '+directory.name+': '+str(e['controller_failure']))
        classifications[directory.name]=dict(classification=kind,task_failure=True,replacement=False,elapsed_s=(e['end_ns']-e['goal_ns'])/1e9,evidence=['events.json','first_failure_inputs.json','control.csv','launch.log','truth.jsonl'],decision='finish the unchanged fixed schedule; no tuning, replacement or bypass',**extra);new.append(directory.name)
    classification_file.write_text(json.dumps(classifications,indent=2)+'\n')
    if new:print('classified',new,flush=True)
    with (out/f'fixed_schedule_continuation_{attempt}.log').open('w') as log:
        result=subprocess.run([sys.executable,'experiments/r4_gazebo_comparison/corridor.py','batch',str(out),'5'],stdout=log,stderr=subprocess.STDOUT)
    print('fixed schedule step',attempt,'exit',result.returncode,flush=True)
    if result.returncode==0:break
else:raise RuntimeError('fixed sample bound exceeded')
