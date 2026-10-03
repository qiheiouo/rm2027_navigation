from pathlib import Path
import hashlib, json, shutil
repo=Path('/home/qihei/rm2027_navigation')
work=Path('/tmp/rm_observation_anchor_replay')
target=repo/'docs/dynamic_obstacle_critic/stage2_evidence/observation_anchor_replay'
target.mkdir(exist_ok=False)
def copy(p,name):
    q=target/name;q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q)
for name in ['run.sh','build.log','replay.log','source_copy_identity.json']:
    copy(work/name,'verification/'+name)
for p in sorted((work/'initial_build_failure').rglob('*')):
    if p.is_file():copy(p,'verification/'+str(p.relative_to(work)))
for p in sorted((work/'replayed').rglob('*')):
    if p.is_file():copy(p,'replayed/'+str(p.relative_to(work/'replayed')))
copy(Path(__file__),'verification/freeze.py')
base=repo/'docs/dynamic_obstacle_critic/stage2_evidence/observation_anchor_preflight'
report=json.loads((target/'replayed/replay_verification.json').read_text())
assert report['status']=='PASS' and all(report['checks'].values())
provenance=dict(schema=1,source_archive='../observation_anchor_preflight',source_manifest_sha256=hashlib.sha256((base/'manifest.json').read_bytes()).hexdigest(),checks=len(report['checks']),sdk_image='sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',metadata_only_reference_mtime_adjustment_recorded=True,scope='Reexecution of frozen 14a015b5 SDK/preflight with all source bytes unchanged. Initial GCC same-mtime pragma-once failure retained. Physical/task acceptance unchanged.')
(target/'provenance.json').write_text(json.dumps(provenance,indent=2,sort_keys=True)+'\n')
files={str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(target.rglob('*')) if p.is_file()}
(target/'manifest.json').write_text(json.dumps(dict(schema=1,files=files,scope=provenance['scope']),indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(files=len(files),bytes=sum((target/name).stat().st_size for name in files),manifest_sha256=hashlib.sha256((target/'manifest.json').read_bytes()).hexdigest(),checks=len(report['checks'])),sort_keys=True))
