"""Audit the declared moving-fixture goal, without changing the static auditor."""
import json
import sys
from pathlib import Path
import experiment as ex
from simulation_evidence import analyze
from audit_endpoint_witness import audit_file


def inspect_dynamic_trial(path):
    meta=json.loads((path/'metadata.json').read_text())
    assert path.name==f"{meta['planner']}_{meta['trial']}"
    assert ex.sha(path/'profile.yaml')==meta['profile_sha256']
    s=json.loads((path/'observation/summary.json').read_text())
    assert s['goal']==[5.6,0.,0.], 'not the declared Phase1.5D dynamic goal'
    events=ex.read_rows(path/'observation/events.jsonl')
    results=[e for e in events if e.get('event')=='navigation_result']
    pre=[e for e in events if e.get('event')=='preflight']
    assert len(results)==len(pre)==1
    for k,v in [('status','action_status'),('timed_out','timed_out'),('recoveries','recoveries'),('goal','goal')]:assert results[0][k]==s[v]
    assert pre[0]['result']==s['preflight']
    expected=analyze(ex.read_rows(path/'observation/trajectory.jsonl'),ex.read_rows(path/'observation/commands.jsonl'),events,
        5.6,0.,s['action_status'],s['timed_out'],s['recoveries'])
    expected['evidence_valid'] &= not s['clock_reversed'] and s['fixture_spawn_verified']
    expected['static_geometry_and_goal_pass'] &= expected['evidence_valid']
    assert all(s[k]==v for k,v in expected.items()),'raw summary mismatch'
    parameters=[e for e in events if e.get('event')=='runtime_parameters']
    assert len(parameters)==2 and all(e['expected']==e['actual'] for e in parameters)
    return {'summary':s,'raw_summary_matches':True}

ex.inspect_trial=inspect_dynamic_trial
if __name__=='__main__':
    target=Path(sys.argv[1]);m=json.loads((target.parent/'inputs.json').read_text())
    s=ex.analyze(target,m)
    ex.write_new(target/'endpoint_audit.json',audit_file(target/'observation/events.jsonl'))
    print(json.dumps({k:s[k] for k in ('evidence_valid','limited_dynamic_geometry_and_goal_pass','action_status','recoveries','checks','continuity')}))
