"""Restore selected Git evidence into a NEW review directory."""
import pathlib,sys,tarfile,gzip,shutil
source=pathlib.Path(__file__).resolve().parent/'evidence';dest=pathlib.Path(sys.argv[1]).resolve();assert not dest.exists();assert shutil.disk_usage(dest.parent).free>3.05*1024**3;dest.mkdir()
for archive in sorted(source.glob('*.tar.gz')):
 out=dest/archive.name[:-7];out.mkdir()
 with tarfile.open(archive) as t:
  for item in t.getmembers():assert pathlib.Path(item.name).name==item.name and item.isfile()
  t.extractall(out,filter='data')
 for p in out.glob('*.gz'):
  if p.name.endswith('.csv.gz') or p.name.endswith('.jsonl.gz'):
   with gzip.open(p,'rb') as a,p.with_suffix('').open('wb') as b:shutil.copyfileobj(a,b)
print(dest)
