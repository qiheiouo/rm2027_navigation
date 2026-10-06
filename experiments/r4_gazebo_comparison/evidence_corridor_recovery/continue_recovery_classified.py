"""Finish only the authorized fixed S3 schedule after preserving known failure classes."""
import json,os,pathlib,subprocess,sys
out=pathlib.Path(sys.argv[1]).resolve()
assert os.environ.get('R4_RESEARCH_NATIVE_RECOVERY')=='1'
for step in range(11):
    f=out/'failure_classifications.json'
    classifications=json.loads(f.read_text()) if f.exists() else {}
    for directory in sorted((out/'runs').glob('S3_*')):
        if not (directory/'events.json').exists():raise RuntimeError('incomplete run: '+directory.name)
        e=json.loads((directory/'events.json').read_text())
        assert e['native_recovery_enabled']
        if e.get('native_mppi_errors'):
            assert (directory/'first_native_failure_inputs.json').exists() and (directory/'first_native_failure.json').exists()
        if e['finished']=='goal_result' and e['result_status']==4:continue
        if directory.name in classifications:continue
        assert (directory/'first_failure_inputs.json').exists()
        if e['finished']=='actor_contact' and e['controller_failure'] is None and all('moving_obstacle' in str(c['pairs']) and 'rm_sentry_2027' in str(c['pairs']) for c in e['contacts']):
            kind='physical_dynamic_actor_contact';extra=dict(pairs=e['contacts'][0]['pairs'])
        elif e['finished']=='controller_failure' and e['mode']=='R4' and e['controller_failure']=='degenerate path':
            kind='r4_path_input_rejection';extra=dict(reason=e['controller_failure'])
        else:raise RuntimeError('new task failure requires inspection: '+directory.name+': '+str(e['controller_failure']))
        classifications[directory.name]=dict(classification=kind,task_failure=True,replacement=False,elapsed_s=(e['end_ns']-e['goal_ns'])/1e9,native_error_events=len(e['native_mppi_errors']),evidence=['events.json','control.csv','truth.jsonl','launch.log','first_failure_inputs.json'],decision='finish original fixed n5; do not repair or replace task failures',**extra)
        print('preserved/classified',directory.name,kind,flush=True)
    f.write_text(json.dumps(classifications,indent=2)+'\n')
    with (out/f'fixed_recovery_continuation_{step}.log').open('w') as log:
        result=subprocess.run([sys.executable,'experiments/r4_gazebo_comparison/corridor.py','batch',str(out),'5'],stdout=log,stderr=subprocess.STDOUT)
    print('fixed step',step,'exit',result.returncode,flush=True)
    if result.returncode==0:break
else:raise RuntimeError('fixed sample bound exceeded')
