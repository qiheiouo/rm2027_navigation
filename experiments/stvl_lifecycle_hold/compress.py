"""Only this round's completed original streams, lossless and byte-verified."""
import pathlib,json,gzip,hashlib
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/lifecycle_cache'
for p in sorted(CACHE.iterdir()):
 if not p.is_dir() or not (p/'events.json').exists():continue
 manifest=json.loads((p/'compression_manifest.json').read_text()) if (p/'compression_manifest.json').exists() else []
 for f in list(p.glob('*.csv'))+list(p.glob('*.jsonl')):
  h=hashlib.sha256()
  with f.open('rb') as a,gzip.open(str(f)+'.gz','wb',compresslevel=6) as b:
   for buf in iter(lambda:a.read(1024*1024),b''):h.update(buf);b.write(buf)
  h2=hashlib.sha256()
  with gzip.open(str(f)+'.gz','rb') as a:
   for buf in iter(lambda:a.read(1024*1024),b''):h2.update(buf)
  assert h.digest()==h2.digest();manifest=[m for m in manifest if m['source']!=f.name];manifest.append(dict(source=f.name,bytes=f.stat().st_size,sha256=h.hexdigest(),compressed_bytes=pathlib.Path(str(f)+'.gz').stat().st_size));f.unlink()
 (p/'compression_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('completed streams retained as byte-verified gzip')
