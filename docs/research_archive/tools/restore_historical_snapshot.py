#!/usr/bin/env python3
"""Restore a partial historical source/config/report snapshot; no experiment runs."""
import argparse,gzip,hashlib,json,pathlib

def main():
 ap=argparse.ArgumentParser();ap.add_argument('branch');ap.add_argument('destination',type=pathlib.Path);a=ap.parse_args();root=pathlib.Path(__file__).resolve().parents[1];m=json.loads(gzip.decompress((root/'historical_snapshots.json.gz').read_bytes()));b=next(b for b in m['branches'] if b['branch']==a.branch);objs={o['git_blob']:o for o in m['objects']}
 for f in b['files']:
  p=pathlib.PurePosixPath(f['path']);assert not p.is_absolute() and '..' not in p.parts;o=objs[f['git_blob']];data=gzip.decompress((root/o['object']).read_bytes());assert len(data)==o['bytes'] and hashlib.sha256(data).hexdigest()==o['sha256'];dest=a.destination/p;dest.parent.mkdir(parents=True,exist_ok=True)
  if dest.exists() and dest.read_bytes()!=data:raise FileExistsError('refusing to replace differing file: '+str(dest))
  dest.write_bytes(data)
 print('restored partial snapshot',b['branch'],b['commit'],len(b['files']),'files; consult omissions and original history')
if __name__=='__main__':main()
