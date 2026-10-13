"""Restore text evidence into an empty cache; no simulation or hardware starts."""
import pathlib,tarfile,gzip,sys,shutil
root=pathlib.Path(__file__).resolve().parents[2];source=root/'experiments/stvl_temporal/evidence';dest=pathlib.Path(sys.argv[1]) if len(sys.argv)>1 else root/'build/stvl_cache'
archives=sorted((source/'runs').glob('*.tar.gz'));required=0
for path in archives:
 with tarfile.open(path,'r:gz') as archive:required+=sum(m.size for m in archive.getmembers())
probe=dest
while not probe.exists():probe=probe.parent
if shutil.disk_usage(probe).free<required+3*1024**3:raise RuntimeError('restore would cross 3GiB reserve')
if (dest/'runs').exists() and any((dest/'runs').iterdir()):raise RuntimeError('destination runs must be empty; existing evidence is never overwritten')
for path in archives:
 target=dest/'runs'/path.name.removesuffix('.tar.gz');target.mkdir(parents=True)
 with tarfile.open(path,'r:gz') as archive:
  for member in archive.getmembers():
   if not member.isfile() or pathlib.PurePosixPath(member.name).name!=member.name:raise RuntimeError('unexpected archive member')
   (target/member.name).write_bytes(archive.extractfile(member).read())
(dest/'assets').mkdir(exist_ok=True)
for path in (source/'assets').glob('oracle_*.json.gz'):(dest/'assets'/path.name.removesuffix('.gz')).write_bytes(gzip.decompress(path.read_bytes()))
print('restored',len(archives),'trials to',dest)
