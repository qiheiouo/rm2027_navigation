"""Losslessly compress only completed owned streams; verify before unlink."""
import gzip,hashlib,json,pathlib
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/coasting_live_cache'
for p in CACHE.glob('*/runs/*'):
 if not (p/'supervisor.json').exists():continue
 entries=json.loads((p/'compression_manifest.json').read_text()) if (p/'compression_manifest.json').exists() else []
 for f in list(p.glob('*.csv'))+list(p.glob('*.jsonl')):
  dest=pathlib.Path(str(f)+'.gz');assert not dest.exists(),'retain previous compressed evidence'
  h=hashlib.sha256()
  with f.open('rb') as a,gzip.open(dest,'wb',compresslevel=6) as b:
   for buf in iter(lambda:a.read(1024*1024),b''):h.update(buf);b.write(buf)
  v=hashlib.sha256()
  with gzip.open(dest,'rb') as a:
   for buf in iter(lambda:a.read(1024*1024),b''):v.update(buf)
  assert v.digest()==h.digest();entries.append({'source':f.name,'bytes':f.stat().st_size,'sha256':h.hexdigest(),'compressed_bytes':dest.stat().st_size});f.unlink()
 (p/'compression_manifest.json').write_text(json.dumps(entries,indent=2)+'\n')
print('owned completed streams retained byte-exact, no unique bytes discarded')
