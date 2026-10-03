#!/usr/bin/env python3
"""Verify and replay frozen mechanical trial evidence into a new private copy."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    archive=args.archive.resolve();manifest=json.loads((archive/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        path=(archive/name).resolve()
        if not path.is_relative_to(archive) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('archive identity mismatch: '+name)
    shutil.copytree(archive,args.output)
    output=args.output.resolve();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    env['MPLCONFIGDIR']=str(output/'mpl_config')
    jobs=[('analyze_trial.py','analysis.json',[]),('audit_raw_costmap_gate.py','raw_costmap_audit.json',[]),
          ('audit_guard_rejections.py','guard_audit.json',[]),('audit_stopping_objective.py','stopping_audit.json',[]),
          ('audit_prediction.py','prediction_audit.json',[]),('audit_scan_geometry.py','scan_geometry_audit.json',[]),
          ('audit_mechanical_footprint.py','mechanical_footprint_audit.json',['--include-static']),
          ('audit_contact_witness.py','contact_audit.json',[]),
          ('audit_contact_witness.py','mechanical_contact_audit.json',['--mechanical-events']),
          ('audit_trial_timing.py','timing_audit.json',[]),
          ('audit_raw_transitions.py','raw_transition_audit.json',['--output',str(output/'raw_transition_audit.json')]),
          ('plot_mechanical_contract_trial.py','mechanical_contract_witness.png',[])]
    results={}
    for script,report,flags in jobs:
        with (output/(report+'.replay.log')).open('w') as log:
            subprocess.run([sys.executable,'-B',str(output/'auditors'/script),str(output),*flags],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        results[report]=(output/report).read_bytes()==(archive/report).read_bytes()
    results['mechanical_contract_witness.svg']=(output/'mechanical_contract_witness.svg').read_bytes()==(archive/'mechanical_contract_witness.svg').read_bytes()
    result={'result':'PASS' if all(results.values()) else 'FAILED','reports_and_figures_byte_exact':results,
            'source_data':'verified frozen gzip archive; output is a private copy',
            'mechanical_include_static':True,'plot':'plot_mechanical_contract_trial.py'}
    (output/'replay_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    return 0 if result['result']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
