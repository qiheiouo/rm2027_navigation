"""Lossless compression of this round's completed streams only."""
import pathlib,sys,gzip,hashlib,json
r=pathlib.Path(sys.argv[1]);assert 'stvl_c83_endtoend_20261014/build/c83_cache/' in str(r.resolve());items=[]
p=r/'compression_manifest.json'
if p.exists():items=json.loads(p.read_text())
for source in list(r.glob('*.raw'))+list(r.glob('*.csv'))+list(r.glob('*.jsonl')):
 h=hashlib.sha256();size=0
 with source.open('rb') as a,gzip.open(str(source)+'.gz','wb',compresslevel=6) as b:
  for buf in iter(lambda:a.read(1024*1024),b''):h.update(buf);size+=len(buf);b.write(buf)
 h2=hashlib.sha256()
 with gzip.open(str(source)+'.gz','rb') as a:
  for buf in iter(lambda:a.read(1024*1024),b''):h2.update(buf)
 assert h.digest()==h2.digest();items.append(dict(source=source.name,sha256=h.hexdigest(),bytes=size,compressed_bytes=pathlib.Path(str(source)+'.gz').stat().st_size));source.unlink()
p.write_text(json.dumps(items,indent=2)+'\n');print('verified lossless streams',len(items))
