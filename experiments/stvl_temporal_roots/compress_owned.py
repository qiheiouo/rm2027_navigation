"""Lossless compression of completed new runs only; old raw is never touched."""
import pathlib,gzip,hashlib,json
root=pathlib.Path(__file__).resolve().parents[2];cache=root/'build/roots_cache';manifest=root/'experiments/stvl_temporal_roots/evidence/compression_manifest.json';index=json.loads(manifest.read_text()) if manifest.exists() else [];known={r['compressed'] for r in index}
for cohort in ['before','diag','repair','heldout_repair','crash_debug','crash_alignment','repair_abi','heldout_abi','heldout_abi_retry','repair_odom','heldout_odom']:
 for p in (cache/cohort/'runs').glob('*/*.jsonl'):
  if not (p.parent/'events.json').exists():continue
  data=p.read_bytes();target=p.with_suffix(p.suffix+'.gz')
  with target.open('wb') as out:
   with gzip.GzipFile(filename='',mode='wb',fileobj=out,compresslevel=9,mtime=0) as zipfile:zipfile.write(data)
  assert gzip.decompress(target.read_bytes())==data
  if str(target) not in known:index.append({'source':str(p),'compressed':str(target),'original_bytes':len(data),'compressed_bytes':target.stat().st_size,'sha256':hashlib.sha256(data).hexdigest()})
  p.unlink()
manifest.write_text(json.dumps(index,indent=2)+'\n');print('verified compression audit entries',len(index))
