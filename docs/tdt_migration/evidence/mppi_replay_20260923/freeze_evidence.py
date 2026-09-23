#!/usr/bin/env python3
"""Freeze this new series; verify previous hashes without changing them."""
from pathlib import Path
import hashlib,json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PREV=ROOT/'docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922/manifest.json'
SERIES=ROOT/'build/tdt_p2b/runs/dynamic_odom_routing_pilot_v1'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    old=json.loads(PREV.read_text())
    for name,digest in old['files'].items():
        path=ROOT/name
        assert path.is_file() and sha(path)==digest,name
    new=json.loads((HERE/'candidate_audit.json').read_text())
    assert new['preserved_previous_manifest_entries']==len(old['files'])
    paths=[p for p in HERE.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts]
    paths += [p for p in SERIES.rglob('*') if p.is_file()]
    files={str(p.relative_to(ROOT)):sha(p) for p in sorted(paths)}
    manifest={'schema':'tdt_mppi_odom_pilot_evidence/v1',
              'scope':'offline replay and one-variable fixed-phase A*/QP pilot; no dynamic acceptance or hardware',
              'previous_manifest_sha256':sha(PREV),'previous_manifest_entries_verified':len(old['files']),
              'self_hash_excluded':True,'files':files}
    with (HERE/'manifest.json').open('x') as f:json.dump(manifest,f,indent=2);f.write('\n')
    print('new files',len(files),'previous verified',len(old['files']))
if __name__=='__main__':main()
