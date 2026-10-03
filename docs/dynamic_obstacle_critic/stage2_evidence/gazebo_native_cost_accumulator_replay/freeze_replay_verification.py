import gzip
import hashlib
import json
from pathlib import Path
import shutil

repo=Path('/home/qihei/rm2027_navigation');temp=Path('/tmp/rm_native_cost_runtime')
archive=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_native_cost_accumulator_evidence'
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/gazebo_native_cost_accumulator_replay';out.mkdir()
manifest=json.loads((archive/'manifest.json').read_text())
for relative,digest in manifest['files'].items():
    assert hashlib.sha256((archive/relative).read_bytes()).hexdigest()==digest,relative
for name in ('archive_replay.log','independent_replay.log','independent_native.log'):
    shutil.copyfile(temp/name,out/name)
for name,directory in [('stored_outputs_replay.json','archive_replay'),('independent_sdk_replay.json','independent_replay')]:
    data=json.loads((temp/directory/'replay_verification.json').read_text());assert data['verdict']=='PASS'
    shutil.copyfile(temp/directory/'replay_verification.json',out/name)
identities={}
for name,golden in [('dynamic_risk','dynamic_replay/native_risk.jsonl.gz'),
    ('sdk_final_output','native_optimizer/sdk_final_output.jsonl.gz'),('zero_history_output','native_optimizer/zero_history_output.jsonl.gz'),
    ('weights_output','native_optimizer/weights_output.jsonl.gz'),('uniform_output','native_optimizer/uniform_output.jsonl.gz')]:
    data=(temp/'independent_native'/(name+'.jsonl')).read_bytes();assert data==gzip.open(archive/golden,'rb').read()
    identities[name]={'decoded_sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'golden':golden,'byte_exact':True}
    with (out/(name+'.jsonl.gz')).open('xb') as sink:
        with gzip.GzipFile(filename='',mode='wb',fileobj=sink,mtime=0) as stream:stream.write(data)
(out/'independent_outputs.json').write_text(json.dumps(identities,indent=2)+'\n')
inputs={}
for name,relative in [('dynamic','regenerated_dynamic_input/input.bin'),('native','regenerated_optimizer_input/input.bin')]:
    inputs[name]=hashlib.sha256((temp/'archive_replay'/relative).read_bytes()).hexdigest()
(out/'input_identity.json').write_text(json.dumps(inputs,indent=2)+'\n')
shutil.copyfile(Path(__file__),out/'freeze_replay_verification.py')
metadata={'archive':'gazebo_native_cost_accumulator_evidence','archive_manifest_sha256':hashlib.sha256((archive/'manifest.json').read_bytes()).hexdigest(),
    'original_manifest_files_unchanged':len(manifest['files']),'stored_output_replay_byte_checks':31,'independent_sdk_replay_byte_checks':36,
    'new_sdk_outputs':5,'sdk_image':'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
    'scope':'Independent regenerated inputs and new SDK execution; same failed physical/raw203/task gates and unassigned pairs preserved.'}
(out/'verification_identity.json').write_text(json.dumps(metadata,indent=2)+'\n')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':metadata['scope'],'files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes_excluding_manifest':sum((out/n).stat().st_size for n in files),
    'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest()}))
