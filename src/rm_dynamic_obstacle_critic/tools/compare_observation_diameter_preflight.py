#!/usr/bin/env python3
"""Compare initial archived preflight with fresh SDK execution artifacts."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
from replay_observation_anchor_preflight import verify


def bytes_at(root,relative):
    path=root/relative
    if path.is_file():return path.read_bytes()
    with gzip.open(path.with_name(path.name+'.gz'),'rb') as file:return file.read()


def normalize(value,prefix):
    if isinstance(value,str):return value.replace(prefix,'<run>')
    if isinstance(value,list):return [normalize(item,prefix) for item in value]
    if isinstance(value,dict):return {key:normalize(item,prefix) for key,item in value.items()}
    return value


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('fresh_output',type=Path)
    parser.add_argument('new_report',type=Path);args=parser.parse_args();archive=args.archive.resolve()
    verify(archive);initial=archive/'verification';fresh=args.fresh_output.resolve();checks={}
    old=json.loads((initial/'summary.json').read_text());new=json.loads((fresh/'summary.json').read_text())
    checks['initial_19_checks']=old['status']=='PASS' and len(old['checks'])==19 and all(old['checks'].values())
    checks['fresh_21_checks']=new['status']=='PASS' and len(new['checks'])==21 and all(new['checks'].values())
    checks['runtime_elf_exact']=old['runtime_elf_sha256']==new['runtime_elf_sha256']
    checks['helper_elf_exact']=old['helper_sha256']==new['helper_sha256']
    checks['dependency_manifests_exact']=old['dependency_manifests']==new['dependency_manifests']
    checks['default_core_report_exact']=bytes_at(initial,'default_core.json')==bytes_at(fresh,'default_core.json')
    for name in ['consumption_tools/summary.json','v1_regression/summary.json','v1_regression/risk.jsonl',
                 'v1_regression/prepared/input.bin','v1_regression/prepared/input_identity.json',
                 'v1_regression/consumed_cv_support_audit.json','v1_regression/mechanical_contact_audit.json',
                 'v1_regression/receipt/prediction_audit.json']:
        checks['byte_exact:'+name]=bytes_at(initial,name)==bytes_at(fresh,name)
    for label,target in [('visible','V2VisibleActualScoreAndBudgetArePreserved'),('nominal','V2NominalActualScoreAndBudgetArePreserved')]:
        for name in ['input.bin','input_identity.json','risk.jsonl','processed_paths.bin','processed.jsonl']:
            relative='consumption_tools/'+label+'/'+name
            checks['byte_exact:'+relative]=bytes_at(initial,relative)==bytes_at(fresh,relative)
        for extension in ['json','bin']:
            relative='cpp_fixtures/'+target+'/score_0.'+extension
            checks['byte_exact:'+relative]=bytes_at(initial,relative)==bytes_at(fresh,relative)
        a=json.loads((initial/('guard_'+label)/'summary.json').read_text());b=json.loads((fresh/('guard_'+label)/'summary.json').read_text())
        checks['guard_parameters:'+label]=a['parameter_readback']==b['parameter_readback']
        checks['guard_cases:'+label]=[(r['case'],r['passed']) for r in a['cases']]==[(r['case'],r['passed']) for r in b['cases']]
    a=json.loads((initial/'profiles/result.json').read_text());b=json.loads((fresh/'profiles/result.json').read_text())
    checks['actual_profile_readbacks_and_rejections']=normalize(a,'/out/final_diameter_preflight')==normalize(b,'/out/replayed')
    def observations(root):
        return {json.loads(path.read_text())['mode']:json.loads(path.read_text()) for path in root.rglob('observations.json')}
    checks['actual_dds_observations_exact']=observations(initial/'ros_fixtures')==observations(fresh/'ros_fixtures')
    a=json.loads((initial/'default_wire.json').read_text());b=json.loads((fresh/'default_wire.json').read_text())
    checks['default_wire_semantic_exact']=all(a[key]==b[key] for key in
        ['status','canonical_wire_trace_sha256','frames','coasting_tracks','truncated_frames','serialized_fields','structural_padding_bytes_examined'])
    report=dict(schema=1,status='PASS' if all(checks.values()) else 'FAILED',checks=checks,count=len(checks),
        primary_manifest_sha256=hashlib.sha256((archive/'manifest.json').read_bytes()).hexdigest(),
        scope='Fresh SDK execution compared to complete verified archive. Only run-directory prefixes normalize; timing, callback counts and CDR padding are not byte gates. No physical/task acceptance.')
    with args.new_report.open('x') as file:file.write(json.dumps(report,indent=2,sort_keys=True)+'\n')
    print(json.dumps(report,sort_keys=True));return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
