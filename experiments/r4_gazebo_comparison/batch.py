"""Preregistered finite schedule. Sequential fresh instances, alternate order."""
import json,pathlib,subprocess,sys,time
root=pathlib.Path(__file__).resolve().parents[2];out=pathlib.Path(sys.argv[1]).resolve()
schedule=[f'{scene}:{mode}:{repeat}' for scene,count in [('S0',5),('S1',10),('S2',10)]
          for repeat in range(101,101+count) for mode in (('B0','R4') if repeat%2 else ('R4','B0'))]
replacement_file=out/'replacement_runs.json'
replacements=json.loads(replacement_file.read_text()) if replacement_file.exists() else {}
schedule=[replacements.get(spec,spec) for spec in schedule]
(out/'schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
for spec in schedule:
    name=spec.replace(':','_');directory=out/'runs'/name
    if directory.exists():
        # Resume only already completed trials. Never rerun a failed outcome.
        if not (directory/'events.json').exists(): raise RuntimeError(f'incomplete preserved run: {name}')
        e=json.loads((directory/'events.json').read_text())
        if e['goal_ns'] is None: raise RuntimeError(f'unclassified startup failure: {name}')
        print(name,'already recorded',flush=True);continue
    with (out/f'{name}_runner.log').open('w') as f:
        result=subprocess.run(['bash',str(root/'experiments/r4_gazebo_comparison/run.sh'),str(out),spec],cwd=root,stdout=f,stderr=subprocess.STDOUT)
    if not (directory/'events.json').exists(): raise RuntimeError(f'no record: {name}, exit={result.returncode}')
    e=json.loads((directory/'events.json').read_text())
    print(name,e['finished'],'status',e['result_status'],'reason',e['controller_failure'],flush=True)
    if e['goal_ns'] is None: raise RuntimeError('startup blocked; preserve and classify before resuming')
print('finite schedule complete',flush=True)
