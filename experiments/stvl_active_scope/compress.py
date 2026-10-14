"""Only completed current-stage streams; never discard unique raw bytes."""
import pathlib,json,gzip,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/active_scope_cache'
for p in CACHE.glob('*/runs/*'):
 if not (p/'events.json').exists() or not (p/'supervisor.json').exists():continue
 manifests=json.loads((p/'compression_manifest.json').read_text()) if (p/'compression_manifest.json').exists() else []
 for f in list(p.glob('*.csv'))+list(p.glob('*.jsonl'))+list(p.glob('*.raw')):
  h=hashlib.sha256();dest=pathlib.Path(str(f)+'.gz')
  with f.open('rb') as a,gzip.open(dest,'wb',compresslevel=6) as b:
   for buf in iter(lambda:a.read(1024*1024),b''):h.update(buf);b.write(buf)
  verify=hashlib.sha256()
  with gzip.open(dest,'rb') as a:
   for buf in iter(lambda:a.read(1024*1024),b''):verify.update(buf)
  assert verify.digest()==h.digest();manifests.append(dict(source=f.name,bytes=f.stat().st_size,sha256=h.hexdigest(),compressed_bytes=dest.stat().st_size));f.unlink()
 (p/'compression_manifest.json').write_text(json.dumps(manifests,indent=2)+'\n')
print('completed current-stage streams retained byte-exact in gzip')
