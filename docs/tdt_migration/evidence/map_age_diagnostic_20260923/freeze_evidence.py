#!/usr/bin/env python3
"""Freeze new diagnostic outputs after verifying the older evidence boundary."""
from pathlib import Path
import hashlib,json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PREV=ROOT/'docs/tdt_migration/evidence/mppi_replay_20260923/manifest.json'
RUNS=ROOT/'build/tdt_p2b/runs/dynamic_map_age_pilot_v1'
WORK=ROOT/'build/tdt_p2b/map_age_diagnostic_v1'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 old=json.loads(PREV.read_text())
 for name,digest in old['files'].items():
  p=ROOT/name
  assert p.is_file() and sha(p)==digest,name
 preservation=json.loads((HERE/'preservation_after.json').read_text())
 assert preservation['source_manifest_sha256']==sha(PREV)
 assert preservation['entries']==len(old['files']) and preservation['mismatches']==[]
 result=json.loads((HERE/'map_age_analysis_v2.json').read_text())
 assert len(result['trials'])==2
 for row in result['trials']:
  assert row['all_cycles_latest_completed_map_matches_raw'] and not row['dynamic_gate_pass']
  assert row['cycle_count']>=200
 paths=[p for p in HERE.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts]
 paths.extend(p for p in RUNS.rglob('*') if p.is_file())
 for name in ('build.log','build_retry.log','build_final.log','build_timestamp.log',
              'build_style.log','final_checks.log','test.log','test_offline.log',
              'analyzer_tests.log','analysis.log','linkage.log','run_astar.log','run_qp.log'):
  paths.append(WORK/name)
 for name in ('src/layered_costmap.cpp','plugins/obstacle_layer.cpp',
              'include/nav2_costmap_2d/tdt_map_trace.hpp'):
  paths.append(WORK/'source/nav2_costmap_2d'/name)
 paths.extend((WORK/'install/nav2_costmap_2d/lib').glob('libnav2_costmap_2d*.so'))
 paths.append(ROOT/'build/tdt_p2b/mppi_cycle_diagnostic_v1/navigation2-1.1.20.tar.gz')
 assert len(paths)==len(set(paths))
 assert all(p.is_file() for p in paths)
 files={str(p.relative_to(ROOT)):sha(p) for p in sorted(paths)}
 manifest={'schema':'tdt_costmap_age_diagnostic_evidence/v1',
    'scope':'instrumented fixed-phase A*/QP first cases; not dynamic acceptance or hardware',
    'previous_manifest_sha256':sha(PREV),
    'previous_manifest_entries_verified':len(old['files']),
    'self_hash_excluded':True,'files':files}
 with (HERE/'manifest.json').open('x') as f:json.dump(manifest,f,indent=2);f.write('\n')
 print('frozen files',len(files),'previous verified',len(old['files']))
if __name__=='__main__':main()
