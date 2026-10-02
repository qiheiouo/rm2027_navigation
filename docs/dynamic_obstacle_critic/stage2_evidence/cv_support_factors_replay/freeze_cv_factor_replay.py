import gzip
import hashlib
import json
from pathlib import Path
import shutil

repo = Path('/home/qihei/rm2027_navigation')
base = Path('/tmp/rm_dynamic_critic_v1')
archive = repo/'docs/dynamic_obstacle_critic/stage2_evidence/cv_support_factors'
out = archive.parent/'cv_support_factors_replay'
out.mkdir()
for label, folder, log in [('archive', 'cv_support_factor_archive_replay', 'cv_support_factor_archive_replay.log'),
                           ('cpp', 'cv_support_factor_archive_replay_cpp', 'cv_support_factor_archive_replay_cpp.log')]:
    shutil.copyfile(base/folder/'replay_verification.json', out/(label+'_verification.json'))
    shutil.copyfile(base/log, out/log)
for name in ['cv_support_factor_independent_cpp.log', 'freeze_cv_support_factors.log']:
    shutil.copyfile(base/name, out/name)
with (out/'independent_factors.jsonl.gz').open('xb') as stream:
    with gzip.GzipFile(filename='', mode='wb', fileobj=stream, mtime=0) as writer:
        writer.write((base/'cv_support_factor_independent_cpp.jsonl').read_bytes())
verified = {}
for folder in ['gazebo_dynamic_consumption_evidence', 'dynamic_consumption_witness', 'raw_rollout_objective',
               'processed_cv_objective', 'cv_support_factors']:
    root = archive.parent/folder
    manifest = json.loads((root/'manifest.json').read_text())
    for name, digest in manifest['files'].items():
        path = (root/name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('archive identity mismatch '+folder+'/'+name)
    verified[folder] = {'files': len(manifest['files']), 'manifest_sha256': hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()}
if (base/'cv_support_factor_binaries_before.txt').read_bytes()!=(base/'cv_support_factor_binaries_after.txt').read_bytes():
    raise ValueError('original seven ELF identities changed')
(out/'archive_identity_verification.json').write_text(json.dumps({'verdict':'PASS', 'archives':verified,
    'original_seven_ELF_identities_unchanged':True,
    'independent_cpp_sha256':hashlib.sha256((base/'cv_support_factor_independent_cpp.jsonl').read_bytes()).hexdigest()},indent=2)+'\n')
shutil.copyfile(Path(__file__), out/'freeze_cv_factor_replay.py')
files = {str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Independent frozen factor replay; original archive is unchanged.', 'files':files},indent=2)+'\n')
print(json.dumps({'files':len(files), 'verdict':'PASS', 'verified_archive_files':sum(v['files'] for v in verified.values())}))
