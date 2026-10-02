import hashlib
import json
from pathlib import Path
import shutil

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
archives=repo/'docs/dynamic_obstacle_critic/stage2_evidence';out=archives/'native_sg_stages_replay';out.mkdir()
identities={}
for name in ['gazebo_dynamic_consumption_evidence','dynamic_consumption_witness','cv_support_factors','native_factor_aggregation','native_sg_stages']:
    root=archives/name;manifest=json.loads((root/'manifest.json').read_text())
    for path,digest in manifest['files'].items():
        if hashlib.sha256((root/path).read_bytes()).hexdigest()!=digest:raise ValueError('reference artifact mismatch: '+name+'/'+path)
    identities[name]={'files':len(manifest['files']),'manifest_sha256':hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()}
for folder,name,count in [('native_sg_stage_archive_replay','frozen_replay_verification.json',12),('native_sg_stage_archive_replay_cpp','independent_CPP_replay_verification.json',15)]:
    path=base/folder/'replay_verification.json';report=json.loads(path.read_text())
    if report['verdict']!='PASS' or len(report['byte_exact'])!=count or not all(report['byte_exact'].values()):raise ValueError('all expected replay checks required')
    shutil.copyfile(path,out/name)
for name in ['native_sg_stage_archive_replay.log','native_sg_stage_archive_replay_cpp.log','native_sg_stage_independent_paths.bin',
             'native_sg_stage_independent_native.jsonl','native_sg_stage_independent_native.log','native_sg_stage_independent_scores.jsonl',
             'native_sg_stage_independent_scores.log','native_sg_stage_independent_prepare.log','freeze_native_sg_stages.log']:
    shutil.copyfile(base/name,out/name)
shutil.copyfile(base/'native_sg_stage_independent_scoring_input/input_identity.json',out/'independent_scoring_input_identity.json')
report={'verdict':'PASS','referenced_archives':identities,'referenced_files':sum(v['files'] for v in identities.values()),
        'frozen_checks':12,'independent_CPP_checks':15,'scope':'Independent stage numerical replay only; before and after full gates all fail, original FAILED/NOT ESTABLISHED unchanged.'}
(out/'reference_verification.json').write_text(json.dumps(report,indent=2)+'\n')
shutil.copyfile(Path(__file__),out/Path(__file__).name)
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Separate independent stage verification; original manifests unchanged.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),**report}))
