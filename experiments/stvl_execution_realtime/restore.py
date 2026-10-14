"""Restore text evidence to a new directory without ROS or motion."""
import pathlib,sys,tarfile,shutil,gzip
dest=pathlib.Path(sys.argv[1]).resolve();source=pathlib.Path(__file__).resolve().parent/'evidence'
assert not dest.exists();assert shutil.disk_usage(dest.parent).free>=3.05*1024**3;dest.mkdir()
for p in sorted(source.glob('*.tar.gz')):
    with tarfile.open(p) as tar:
        for m in tar.getmembers():assert (dest/m.name).resolve().is_relative_to(dest) and (m.isfile() or m.isdir())
        tar.extractall(dest)
for directory in ('fresh','boundary_enabled','no_capture','corrected_scene'):
    for p in (dest/directory).glob('*/*.csv.gz'):
        with gzip.open(p,'rb') as a,p.with_suffix('').open('wb') as b:shutil.copyfileobj(a,b)
print('restored',dest)
