#!/usr/bin/env python3
"""Audit actual DDS fixtures and reject independent captured-field corruptions."""
import argparse
import json
from pathlib import Path
import shutil
from audit_actual_surface_members import audit


def verify(fixtures,out):
    out.mkdir(parents=True,exist_ok=False);checks={};errors={};positive={}
    for path in sorted(fixtures.glob('*/fixture_summary.json')):
        identity=json.loads(path.read_text());mode=identity['mode'];source=path.parent/'member_evidence'
        if identity['incomplete_budget_refused']:
            try:audit(source,out/('incomplete_'+mode))
            except ValueError as error:
                checks[mode+' incomplete stream rejected']='incomplete member stream' in str(error);errors[mode+' incomplete stream']=str(error)
            else:checks[mode+' incomplete stream rejected']=False
        else:
            result=audit(source,out/mode)
            checks[mode+' full actual context']=result['verdict']=='EXACT CAPTURED MEMBER/TF/ASSOCIATION/PUBLIC CONTEXT PASS'
            positive[mode]=source
    if set(positive)!= {'filtered','last_observation_cv'}:raise ValueError('two positive actual DDS fixtures required')
    source=positive['last_observation_cv']
    first=next(p for p in sorted(source.glob('scan_*.json')) if json.loads(p.read_text())['status']=='accepted')
    tests=[
        ('source epoch',lambda r:r.__setitem__('source_stamp_ns',r['source_stamp_ns']+1),'source frame/epoch'),
        ('tf matrix',lambda r:r['source_tf']['used_xy_rotation'].__setitem__(0,0.),'TF XY block'),
        ('projected point',lambda r:r['projected_endpoints'][0].__setitem__(1,9.),'return bits'),
        ('subtraction member',lambda r:r['candidate_source_indices'].pop(),'subtraction membership'),
        ('cluster source beam',lambda r:r['detections'][0]['source_indices'].pop(),'cluster/assignment/filter'),
        ('cluster centroid',lambda r:r['detections'][0]['value']['centroid'].__setitem__('x',-1.),'cluster/assignment/filter'),
        ('actual track id',lambda r:r['detections'][0].__setitem__('track_id',999),'cluster/assignment/filter'),
        ('filter velocity',lambda r:r['tracker_update']['tracks'][0]['velocity'].__setitem__('x',1.),'cluster/assignment/filter'),
        ('public anchor',lambda r:r['public_prediction']['tracks'][0]['xy'].__setitem__(0,9.),'public prediction'),
        ('public z',lambda r:r['public_prediction']['tracks'][0].__setitem__('position_z',1.),'public prediction'),
        ('public complete',lambda r:r['public_prediction'].__setitem__('complete',True),'public prediction'),
    ]
    for name,mutate,reason in tests:
        probe=out/'corruptions'/name.replace(' ','_');shutil.copytree(source,probe)
        selected=probe/first.name;record=json.loads(selected.read_text());mutate(record);selected.write_text(json.dumps(record)+'\n')
        try:audit(probe,out/'corruption_audits'/name.replace(' ','_'))
        except ValueError as error:checks[name+' rejected']=reason in str(error);errors[name]=str(error)
        else:checks[name+' rejected']=False
    probe=out/'corruptions/map_data';shutil.copytree(source,probe)
    selected=next(probe.glob('map_*.json'));record=json.loads(selected.read_text());record['data'][0]=100;selected.write_text(json.dumps(record)+'\n')
    try:audit(probe,out/'corruption_audits/map_data')
    except ValueError as error:checks['map data rejected']='map identity mismatch' in str(error);errors['map data']=str(error)
    else:checks['map data rejected']=False
    report={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,'errors':errors,
            'scope':'Two actual DDS contexts replayed and twelve independently inconsistent captured fields plus two incomplete streams rejected; no surface control/C/real-hardware acceptance'}
    (out/'comparison.json').write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('fixtures',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    report=verify(args.fixtures,args.output);print(json.dumps(report))
    if report['verdict']!='PASS':raise SystemExit(1)
