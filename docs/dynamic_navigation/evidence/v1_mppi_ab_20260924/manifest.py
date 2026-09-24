#!/usr/bin/env python3
"""Freeze or verify dynamic V1 evidence without rewriting any raw trial."""
import hashlib,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
WORK=ROOT/'build/tdt_p2b'
SERIES=('dynamic_prediction_mppi_ab_v1','dynamic_prediction_mppi_ab_v2',
        'dynamic_prediction_mppi_multiphase_v1','dynamic_prediction_mppi_multiphase_v2')
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for part in iter(lambda:f.read(1024*1024),b''):h.update(part)
    return h.hexdigest()
def paths():
    result=[]
    for base in (HERE,ROOT/'experiments/dynamic_prediction_v1'):
        result.extend(p for p in base.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts)
    for name in SERIES:
        result.extend(p for p in (WORK/'runs'/name).rglob('*') if p.is_file())
    result += [ROOT/'docs/dynamic_navigation/v1_prediction_scope.md',
      ROOT/'src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py',
      ROOT/'src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/dynamic_obstacle_tracker_node.py',
      ROOT/'src/rm_dynamic_obstacle_tracking/config/dynamic_obstacle_tracking_shadow.yaml',
      ROOT/'src/rm_simulation/models/moving_obstacle.sdf',
      WORK/'dynamic_prediction_critic_build/install/rm_dynamic_prediction_critic/lib/librm_dynamic_prediction_critic.so',
      WORK/'dynamic_prediction_critic_build/install/rm_dynamic_prediction_critic/share/rm_dynamic_prediction_critic/critics.xml',
      WORK/'dynamic_prediction_critic_build/build/rm_dynamic_prediction_critic/ament_cmake_gtest/test_geometry.txt',
      WORK/'dynamic_prediction_critic_build/build/rm_dynamic_prediction_critic/test_results/rm_dynamic_prediction_critic/test_geometry.gtest.xml']
    return sorted(set(result))
def main():
    target=HERE/'manifest.json'
    if len(sys.argv)>1 and sys.argv[1]=='verify':
        m=json.loads(target.read_text())
        for name,digest in m['files_sha256'].items():
            p=ROOT/name
            if not p.is_file() or sha(p)!=digest:raise ValueError(f'evidence changed: {name}')
        print(json.dumps({'verified':len(m['files_sha256']),'accepted_for_deployment':False}))
        return
    if target.exists():raise FileExistsError(target)
    files={str(p.relative_to(ROOT)):sha(p) for p in paths()}
    m={'schema':'rm_dynamic_prediction_v1_evidence_manifest/v1',
      'source_head_at_run':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
      'image_id':'sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172',
      'series':list(SERIES),'files_sha256':files,'accepted_for_deployment':False,
      'note':'Self not included. Earlier evidence manifests remain separate.'}
    with target.open('x') as f:json.dump(m,f,indent=2);f.write('\n')
    print(json.dumps({'frozen':len(files),'accepted_for_deployment':False}))
if __name__=='__main__':main()
