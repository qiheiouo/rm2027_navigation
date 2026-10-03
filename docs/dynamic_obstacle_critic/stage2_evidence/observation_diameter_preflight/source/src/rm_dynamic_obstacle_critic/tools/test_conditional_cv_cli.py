#!/usr/bin/env python3
"""Exercise file-boundary unknown, malformed input and overwrite behavior."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();args.output.mkdir()
    tool=Path(__file__).with_name('evaluate_conditional_cv_support.py')
    checks={}
    def run(name,assumptions,query,output=None):
        target=output or args.output/(name+'.json')
        with (args.output/(name+'.log')).open('x') as log:
            result=subprocess.run([sys.executable,'-B',str(tool),str(assumptions),str(query),str(target)],stdout=log,stderr=subprocess.STDOUT)
        return result.returncode,target
    default=args.prepared/'cv_support_assumptions_offline.yaml'
    synthetic=args.prepared/'synthetic_assumptions.yaml'
    actual_query=args.prepared/'actual_query.json';synthetic_query=args.prepared/'synthetic_query.json'
    code,actual=run('actual_unknown',default,actual_query)
    rows=json.loads(actual.read_text())['results'] if code==0 else []
    checks['actual_four_queries_unknown_no_radius']=code==0 and len(rows)==4 and all(r['status']=='UNKNOWN' and r['radius_m'] is None for r in rows)
    code,known=run('synthetic_conditional',synthetic,synthetic_query)
    rows=json.loads(known.read_text())['results'] if code==0 else []
    checks['synthetic_constructive_radii']=code==0 and [r['radius_m'] for r in rows]==[1.375,3.375,5.375,7.375]
    before=actual.read_bytes();code,_=run('existing_output',default,actual_query,actual)
    checks['existing_output_preserved']=code!=0 and actual.read_bytes()==before
    for name in ['wrong_units','chassis_motion_only','nonfinite_bound','blank_source','query_schema_bool','empty_queries','query_budget','nonfinite_future','negative_age']:
        a=yaml.safe_load(synthetic.read_text());q=json.loads(synthetic_query.read_text())
        if name=='wrong_units':a['units']='mm'
        elif name=='chassis_motion_only':a['motion_semantics']='chassis_translation_only'
        elif name=='nonfinite_bound':a['bounds']['anchor_to_convex_hull_error_m']=float('nan')
        elif name=='blank_source':a['condition_sources']['initial_diameter_m']=' '
        elif name=='query_schema_bool':q['schema']=True
        elif name=='empty_queries':q['future_offsets_sec']=[]
        elif name=='query_budget':q['future_offsets_sec']=[0]*4097
        elif name=='nonfinite_future':q['future_offsets_sec']=[float('nan')]
        elif name=='negative_age':q['source_age_sec']=-.1
        ap=args.output/(name+'.yaml');qp=args.output/(name+'.query.json')
        ap.write_text(yaml.safe_dump(a,sort_keys=False));qp.write_text(json.dumps(q)+'\n')
        code,out=run(name,ap,qp)
        checks[name+'_rejected_without_output']=code!=0 and not out.exists()
    for name in ['missing_source','outside_interval']:
        a=yaml.safe_load(synthetic.read_text())
        if name=='missing_source':a['condition_sources']['projection_hausdorff_speed_mps']=None
        else:a['bounds']['valid_for_sec']=3.
        ap=args.output/(name+'.yaml');ap.write_text(yaml.safe_dump(a,sort_keys=False))
        code,out=run(name,ap,synthetic_query)
        rows=json.loads(out.read_text())['results'] if code==0 else []
        checks[name+'_unknown_no_radius']=code==0 and len(rows)==4 and rows[-1]['status']=='UNKNOWN' and rows[-1]['radius_m'] is None
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
            'scope':'Offline file contract and explicit synthetic assumptions; no physical certification.'}
    (args.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
