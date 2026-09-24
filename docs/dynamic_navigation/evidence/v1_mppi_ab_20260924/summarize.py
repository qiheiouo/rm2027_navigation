#!/usr/bin/env python3
"""Freeze the prespecified first-trial and stopped repeat coverage."""
import hashlib,json,sys
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
WORK=ROOT/'build/tdt_p2b'
PILOT=WORK/'runs/dynamic_prediction_mppi_ab_v2'
REPEAT=WORK/'runs/dynamic_prediction_mppi_multiphase_v2'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def get(p):return json.loads(p.read_text())
def main():
    first=get(HERE/'ab_audit_v2.json')
    assert len(first['trials'])==4
    assert all(t['evidence_valid'] for t in first['trials'])
    assert all(t['limited_dynamic_gate_pass'] for t in first['trials'] if t['mode']=='candidate')
    plan=get(REPEAT/'matrix_plan.json')
    assert plan['phases_seconds']==[0,2,4,6]
    repeat=[]
    for planner in plan['planners']:
        for mode in plan['modes']:
            p=REPEAT/'phase_0'/f'{mode}_{planner}_1'
            a=get(p/'ab_audit.json');m=get(p/'metadata.json')
            assert a['planner']==planner and a['mode']==mode and a['phase_seconds']==0
            assert sha(p/'profile.yaml')==m['profile_sha256']
            assert sha(p/'profile.yaml')==sha(PILOT/f'{mode}_{planner}_1'/'profile.yaml')
            for name,digest in m['file_sha256'].items():assert sha(ROOT/name)==digest,name
            repeat.append(a)
    assert not any((REPEAT/f'phase_{phase}').exists() for phase in (2,4,6))
    assert all(a['evidence_valid'] for a in repeat)
    assert all(not a['limited_dynamic_gate_pass'] for a in repeat if a['mode']=='candidate')
    for planner in plan['planners']:
        baseline=yaml.safe_load((PILOT/f'baseline_{planner}_1'/'profile.yaml').read_text())
        candidate=yaml.safe_load((PILOT/f'candidate_{planner}_1'/'profile.yaml').read_text())
        c=candidate['controller_server']['ros__parameters']['FollowPath']
        assert c['critics'][-1]=='PredictionV1Critic'
        del c['PredictionV1Critic'];c['critics'].pop()
        assert candidate==baseline
    # Preserve the earlier failed auditor: the strict planar oracle rejected a
    # true nonplanar Gazebo sample, so that run is not silently added to v2.
    bad=WORK/'runs/dynamic_prediction_mppi_multiphase_v1/phase_0/baseline_tdt_qp_1'
    assert bad.exists() and not (bad/'ab_audit.json').exists()
    invalid={'trial':str(bad.relative_to(ROOT)),'reason':'Existing 2D oracle raised nonplanar pose requires 3D projection audit',
      'first_nonplanar_sim_s':36.522,'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in
        (bad/'gazebo_poses.jsonl',bad/'observation/summary.json',bad/'observation/events.jsonl')},
      'counted_in_v2_matrix':False}
    data={'schema':'rm_dynamic_prediction_v1_ab_review/v1','accepted_for_deployment':False,
      'first_trials':first['trials'],'prespecified_phases_seconds':plan['phases_seconds'],
      'repeat_trials':repeat,'matrix_complete':False,
      'stopped_groups':{planner:{'first_failed_candidate_phase_seconds':0,
        'reason':'Existing zero-recovery gate failed despite positive dynamic clearance'} for planner in plan['planners']},
      'invalid_earlier_repeat':invalid,
      'scope':'Fixed phase first trials and one prespecified phase-0 repeat per planner; no phase selection and no dynamic acceptance claim.'}
    with (HERE/'review.json').open('x') as f:json.dump(data,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'first_candidate_passes':sum(t['limited_dynamic_gate_pass'] for t in first['trials'] if t['mode']=='candidate'),
      'repeat_candidate_passes':sum(t['limited_dynamic_gate_pass'] for t in repeat if t['mode']=='candidate'),
      'repeat_candidate_recorded':sum(t['mode']=='candidate' for t in repeat),
      'matrix_complete':False,'accepted_for_deployment':False}))
if __name__=='__main__':main()
