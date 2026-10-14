"""Restore committed compact evidence to a new directory without ROS launch."""
import pathlib,sys,tarfile,shutil
source=pathlib.Path(__file__).resolve().parent/'evidence';dest=pathlib.Path(sys.argv[1]).resolve()
assert not dest.exists(),'refuse existing destination';assert shutil.disk_usage(dest.parent).free>=3.05*1024**3,'disk reserve'
dest.mkdir()
for archive in sorted(source.glob('*.tar.gz')):
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            path=(dest/member.name).resolve()
            assert path.is_relative_to(dest) and (member.isfile() or member.isdir()),'unsafe archive member'
        tar.extractall(dest)
print('restored',dest,'without starting any process or publishing commands')
