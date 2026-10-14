"""Selected evidence only; complete five-run streams remain in local cache."""
import pathlib,json,tarfile,hashlib,gzip
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/lifecycle_cache';OUT=ROOT/'experiments/stvl_lifecycle_hold/evidence'
selected=[]
for label in ('timing','timing_retry','reference','brake','brake_delayed'):
 for p in sorted((CACHE/label).iterdir()):
  if not p.is_file():continue
  if p.name=='native.csv.gz' and label not in ('brake','brake_delayed'):continue
  selected.append(p)
selected += [CACHE/'timing.log',CACHE/'initial_resource.json',CACHE/'refs_before.txt']
with tarfile.open(OUT/'execution_selected.tar.gz','w:gz') as tf:
 for p in selected:tf.add(p,arcname=str(p.relative_to(CACHE)))
manifest=[dict(path=str(p.relative_to(CACHE)),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in selected]
(OUT/'archive_manifest.json').write_text(json.dumps(dict(files=manifest,omitted='full 1ms native.csv.gz of timing, timing_retry and reference remain in local cache; all inputs/physical observations/truth/events retained for every attempt',full_cache=str(CACHE)),indent=2)+'\n')
# Lossless compacting of regenerated summaries, no original raw is deleted.
for name in ('original_c83_lifecycle','durable162_lifecycle','overrun_history_lifecycle'):
 p=OUT/(name+'.json')
 if p.exists():
  raw=p.read_bytes();dest=OUT/(name+'.json.gz');dest.write_bytes(gzip.compress(raw));assert gzip.decompress(dest.read_bytes())==raw;p.unlink()
size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file());assert size<3*1024**2,size
print(json.dumps(dict(selected_bytes=size,archive_bytes=(OUT/'execution_selected.tar.gz').stat().st_size)))
