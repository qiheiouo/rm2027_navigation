#!/usr/bin/env python3
"""Freeze new candidate evidence and all first-created raw series; exclude self."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
EVIDENCE=HERE.parent
DOCS=[
 'dynamic_sweep_routing_20260923',
 'dynamic_sweep_routing_v2_20260923',
 'dynamic_directional_guard_20260924',
 'dynamic_scaled_guard_20260924',
 'dynamic_guard_horizon_20260924',
 'dynamic_viability_guard_20260924']
RUNS=ROOT/'build/tdt_p2b/runs'
SERIES=[
 'dynamic_sweep_routing_pilot_v1','dynamic_sweep_routing_pilot_v2',
 'dynamic_scaled_guard_pilot_v1','dynamic_guard_horizon_pilot_v1',
 'dynamic_guard_horizon_pilot_v2','dynamic_viability_guard_pilot_v1']

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    files={}
    for p in [q for d in DOCS for q in (EVIDENCE/d).rglob('*')]+[
              q for d in SERIES for q in (RUNS/d).rglob('*')]:
        if not p.is_file() or any(x in p.parts for x in ('__pycache__','ros','xdg')):
            continue
        if p==HERE/'manifest.json':continue
        files[str(p.relative_to(ROOT))]=sha(p)
    old=EVIDENCE/'dynamic_guard_pilot_20260923/manifest.json'
    obj={'schema':'tdt_dynamic_viability_evidence_manifest/v1',
         'runtime_source_commit':'a419654a8fed1fc8321c234fb212abd0a6cabe04',
         'prior_manifest_sha256':sha(old),
         'self_hash_excluded':True,'accepted_for_deployment':False,
         'file_count':len(files),'files':dict(sorted(files.items()))}
    out=HERE/'manifest.json'
    with out.open('x') as f:json.dump(obj,f,indent=2);f.write('\n')
    print('manifest_files',len(files),'new_series',len(SERIES))

if __name__=='__main__':main()
