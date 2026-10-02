import hashlib
import json
from pathlib import Path
import shutil

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
archives=repo/'docs/dynamic_obstacle_critic/stage2_evidence';out=archives/'native_factor_safe_mass_replay';out.mkdir()
names=['gazebo_dynamic_consumption_evidence','dynamic_consumption_witness','processed_cv_objective','cv_support_factors',
       'native_factor_aggregation','native_sg_stages','raw_rollout_objective','native_factor_safe_mass'];identities={}
for name in names:
    root=archives/name;manifest=json.loads((root/'manifest.json').read_text())
    for path,digest in manifest['files'].items():
        if hashlib.sha256((root/path).read_bytes()).hexdigest()!=digest:raise ValueError('referenced artifact mismatch: '+name+'/'+path)
    identities[name]={'files':len(manifest['files']),'manifest_sha256':hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()}
replay=base/'native_factor_safe_mass_archive_replay';r=json.loads((replay/'replay_verification.json').read_text())
if r['verdict']!='PASS' or len(r['byte_exact'])!=6 or not all(r['byte_exact'].values()):raise ValueError('all six replay checks required')
shutil.copyfile(replay/'replay_verification.json',out/'replay_verification.json')
for name in ['audit.replay.log','rational.replay.log','negative.replay.log','figures.replay.log']:shutil.copyfile(replay/name,out/name)
shutil.copyfile(base/'freeze_native_factor_safe_mass.log',out/'freeze_native_factor_safe_mass.log')
report={'verdict':'PASS','referenced_archives':identities,'referenced_files':sum(v['files'] for v in identities.values()),'byte_exact_checks':6,
        'scope':'Independent full300 individual gate and 43-row scalar raw recomputation; original FAILED/NOT ESTABLISHED retained.'}
(out/'reference_verification.json').write_text(json.dumps(report,indent=2)+'\n')
shutil.copyfile(Path(__file__),out/Path(__file__).name)
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Separate replay integrity archive; original manifests unchanged.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),**report}))
