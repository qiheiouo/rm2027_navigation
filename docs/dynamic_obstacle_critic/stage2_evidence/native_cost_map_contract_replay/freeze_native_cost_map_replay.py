import hashlib
import json
from pathlib import Path
import shutil
import sys
sys.path.insert(0,'/home/qihei/rm2027_navigation/src/rm_dynamic_obstacle_critic/tools')
from prepare_processed_cv import verify

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1');archives=repo/'docs/dynamic_obstacle_critic/stage2_evidence'
out=archives/'native_cost_map_contract_replay';out.mkdir()
for source,target in [(base/'native_cost_map_archive_replay/replay_verification.json','initial_replay_verification.json'),
    (base/'native_cost_map_archive_replay_actual/replay_verification.json','actual_replay_verification.json'),
    (base/'native_cost_map_reexecute.log','native_reexecute.log'),
    (base/'native_cost_map_native_tests_reexecuted/verification.json','native_probes_verification.json')]:shutil.copyfile(source,out/target)
for label in ['prepare','scalar','analysis','provenance_probes','figures']:
    shutil.copyfile(base/'native_cost_map_archive_replay_actual'/(label+'.replay.log'),out/(label+'.replay.log'))
result=json.loads((out/'actual_replay_verification.json').read_text())
if result['verdict']!='PASS' or len(result['byte_exact'])!=10:raise ValueError('actual SDK and all ten replay checks required')
names=['gazebo_dynamic_consumption_evidence','dynamic_consumption_witness','native_sg_stages','native_factor_safe_mass','native_cost_map_contract']
records={}
for name in names:
    root=archives/name;verify(root);manifest=json.loads((root/'manifest.json').read_text())
    records[name]={'files':len(manifest['files']),'manifest_sha256':hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest(),'unchanged':True}
(out/'reference_verification.json').write_text(json.dumps({'verdict':'PASS','archives':records,'total_manifest_files':sum(v['files'] for v in records.values()),
    'scope':'All original reference/archive files unchanged; actual SDK numerical replay result stored separately.'},indent=2)+'\n')
shutil.copyfile(Path(__file__),out/'freeze_native_cost_map_replay.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Independent actual SDK replay and reference identity, without changing original evidence.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'original_manifest_files_verified':sum(v['files'] for v in records.values())}))
