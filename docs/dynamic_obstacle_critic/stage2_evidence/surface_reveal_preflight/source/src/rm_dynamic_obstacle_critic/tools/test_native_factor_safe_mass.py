#!/usr/bin/env python3
"""Reject raw/SG joins and one-ULP mass tampering against frozen evidence."""
import argparse
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from verify_native_factor_safe_mass import check


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    names=['source','witness','processed','factors','aggregation','stages','raw_archive']
    for name in [*names,'report','scalar','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();args.output.mkdir();roots=[getattr(args,n).resolve() for n in names]
    report=json.loads(args.report.read_text());scalar=json.loads(args.scalar.read_text());checks={}
    positive=check(*roots,report,scalar);checks['independent_positive_exact_rational_check']=positive['verdict']=='PASS'
    (args.output/'positive.json').write_text(json.dumps(positive,indent=2)+'\n')
    def reject(name,mutate,reason):
        r,s=copy.deepcopy(report),copy.deepcopy(scalar);mutate(r,s)
        (args.output/(name+'.report.json')).write_text(json.dumps(r,indent=2)+'\n')
        if s!=scalar:(args.output/(name+'.scalar.json')).write_text(json.dumps(s,indent=2)+'\n')
        try:check(*roots,r,s)
        except ValueError as error:
            checks[name+'_rejected']=reason in str(error)
            (args.output/(name+'.log')).write_text(str(error)+'\n')
        else:checks[name+'_rejected']=False
    def mapping(r,s):r['rows'][0]['processed_witness_row']=0
    reject('raw_processed_row_confusion',mapping,'raw/individual SG row mapping confused')
    def weight(r,s):r['rows'][0]['factor_weights']['actual_baseline']=math.nextafter(r['rows'][0]['factor_weights']['actual_baseline'],math.inf)
    reject('one_ULP_double_weight',weight,'native factor weight/cost bits changed')
    def missing(r,s):del r['rows'][0]['factor_weights']['future_circle_oracle']
    reject('missing_factor_weight',missing,'complete factor weight fields required')
    def cost(r,s):r['rows'][0]['raw_factor_model_scores']['actual_baseline']=copy.deepcopy(r['rows'][0]['individual_factor_model_scores']['actual_baseline'])
    reject('raw_SG_cost_confusion',cost,'cost path identity confused')
    def fraction(r,s):
        d=r['factor_summaries']['actual_baseline']['actual_score_clock'];d['safe_individual_weight_fraction']=math.nextafter(d['safe_individual_weight_fraction'],math.inf)
    reject('one_ULP_safe_fraction',fraction,'independent rational safe mass/count differs')
    def partition(r,s):r['factor_summaries']['future_circle_oracle']['actual_score_clock']['raw_model_vs_distinct_individual_SG_partition'].pop()
    reject('incomplete_disjoint_partition',partition,'complete disjoint partition required')
    def subset(r,s):r['independent_raw_subset'].remove(63);s['verified_sampler_raw_rows'].remove(63)
    reject('omitted_original_positive_raw_label',subset,'all positive safe labels and preregistered raw subset required')
    def gate(r,s):
        d=r['rows'][73]['individual_full_labels']['actual_score_clock'];d['gates']['padded_no_contact']=True;d['conditional_safe_control_with_progress']=all(d['gates'].values())
    reject('forged_full_gate',gate,'derived full gates/progress changed')
    def scalar_mapping(r,s):s['epochs']['actual_score_clock']['rows'][0]['processed_witness_row']=0
    reject('scalar_row_confusion',scalar_mapping,'positive raw scalar/row join changed')
    def incomplete(r,s):r['rows'].pop()
    reject('incomplete_300_rows',incomplete,'complete 300-row safe mass table required')
    native=[json.loads(l) for l in gzip.open(args.aggregation/'native_output.jsonl.gz','rt')]
    native[1]['native_softmax_weights'][0]=math.nextafter(native[1]['native_softmax_weights'][0],math.inf)
    changed=args.output/'changed_native.jsonl';changed.write_text(''.join(json.dumps(r)+'\n' for r in native))
    output_paths=[args.output/'native_changed.report.json',args.output/'native_changed.scalar.json']
    with (args.output/'native_changed.log').open('x') as stream:
        code=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('audit_native_factor_safe_mass.py')),*map(str,roots),*map(str,output_paths),'--native-json',str(changed)],stdout=stream,stderr=subprocess.STDOUT).returncode
    checks['changed_native_weights_rejected_before_output']=code!=0 and 'frozen native factor costs/weights/controls changed' in (args.output/'native_changed.log').read_text() and not any(p.exists() for p in output_paths)
    existing=args.output/'existing.report.json';existing.write_bytes(args.report.read_bytes());digest=hashlib.sha256(existing.read_bytes()).hexdigest()
    fresh=args.output/'existing_new.scalar.json'
    with (args.output/'existing_output.log').open('x') as stream:
        code=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('audit_native_factor_safe_mass.py')),*map(str,roots),str(existing),str(fresh)],stdout=stream,stderr=subprocess.STDOUT).returncode
    checks['existing_report_preserved_without_partial_scalar']=code!=0 and 'refusing to overwrite safe mass reports' in (args.output/'existing_output.log').read_text() and hashlib.sha256(existing.read_bytes()).hexdigest()==digest and not fresh.exists()
    result={'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'Frozen native weight/cost/individual labels and exact rational mass integrity; no ranking or control choice.'}
    (args.output/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':raise SystemExit(main())
