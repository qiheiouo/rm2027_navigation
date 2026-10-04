import gzip,hashlib,json,shutil
from pathlib import Path
work=Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004')
source=work/'build/temporal_mpc_timing_20261004'
target=work/'docs/dynamic_navigation/evidence/temporal_mpc_timing_20261004'
def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
if target.exists():raise SystemExit('refuse archive overwrite')
target.mkdir(parents=True);files=[]
for path in sorted(source.rglob('*')):
 if not path.is_file() or '__pycache__' in path.parts:continue
 relative=path.relative_to(source);dest=target/relative;dest.parent.mkdir(parents=True,exist_ok=True)
 if path.suffix=='.jsonl':
  dest=dest.with_suffix('.jsonl.gz')
  with path.open('rb') as raw,dest.open('wb') as compressed:
   with gzip.GzipFile(filename='',fileobj=compressed,mode='wb',compresslevel=9,mtime=0) as gz:shutil.copyfileobj(raw,gz)
 else:shutil.copy2(path,dest)
 item=dict(path=str(dest.relative_to(target)),bytes=dest.stat().st_size,sha256=digest(dest))
 if path.suffix=='.jsonl':item.update(original_sha256=digest(path),original_bytes=path.stat().st_size)
 files.append(item)
 if path.name=='events.jsonl':print(relative,flush=True)
manifest=dict(files=files,lossless_raw_jsonl=True,dynamic_acceptance=False,candidate_frozen_for_deployment=False,
 physical_projection='velocity_and_stop/v2',portfolio_shared_budget='390 iterations/15ms, worker40ms',post_physics_runtime_changes=False,
 scope='One registered new head-on B0/portfolio pair with matched portfolio shadow load; real clock and Nav2 engineering fixtures; 130 unique host/Humble tests; strict clock joins and old/new age counterfactuals; prestart identities and actual binaries; failed physical/core timing observations and shutdown traces retained. No physical performance repeats or post-physics runtime edits. Documentation updated after frozen snapshots. Old input lineage refers to committed evidence. Original workspace and frozen refs preserved; oracle never control input.')
(target/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('files',len(files),'bytes',sum(f['bytes'] for f in files),flush=True)
