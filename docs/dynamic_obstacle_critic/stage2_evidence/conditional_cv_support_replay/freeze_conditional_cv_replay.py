import hashlib
import json
from pathlib import Path
import shutil

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
archive=repo/'docs/dynamic_obstacle_critic/stage2_evidence/conditional_cv_support'
out=archive.parent/'conditional_cv_support_replay';out.mkdir()
for name in ['replay_verification.json','geometry.replay.log','cli.replay.log','prepare.replay.log']:
    shutil.copyfile(base/'conditional_cv_support_archive_replay'/name,out/name)
for name in ['conditional_cv_support_archive_replay.log','freeze_conditional_cv_support.log']:
    shutil.copyfile(base/name,out/name)
verified={}
for folder in ['gazebo_dynamic_consumption_evidence','cv_support_factors','conditional_cv_support']:
    root=archive.parent/folder;manifest=json.loads((root/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise ValueError('identity mismatch '+folder+'/'+name)
    verified[folder]={'files':len(manifest['files']),'manifest_sha256':hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()}
(out/'archive_identity_verification.json').write_text(json.dumps({'verdict':'PASS','archives':verified},indent=2)+'\n')
shutil.copyfile(Path(__file__),out/'freeze_conditional_cv_replay.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Independent support-contract replay; frozen archives unchanged.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'verified_files':sum(v['files'] for v in verified.values()),'verdict':'PASS'}))
