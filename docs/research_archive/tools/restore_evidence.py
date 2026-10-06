#!/usr/bin/env python3
"""Restore archived selected evidence, verifying SHA256; never execute experiments."""
import argparse, gzip, hashlib, json, pathlib

def main():
 ap=argparse.ArgumentParser();ap.add_argument('destination',type=pathlib.Path);ap.add_argument('--group-prefix',default='');ap.add_argument('--verify-only',action='store_true');a=ap.parse_args()
 root=pathlib.Path(__file__).resolve().parents[1];manifest=json.loads((root/'evidence_manifest.json').read_text());count=0
 for r in manifest['records']:
  if not r['group'].startswith(a.group_prefix):continue
  relative=pathlib.PurePosixPath(r['source'])
  if relative.is_absolute() or '..' in relative.parts:raise ValueError('unsafe manifest path')
  data=gzip.decompress((root/r['object']).read_bytes())
  if len(data)!=r['bytes'] or hashlib.sha256(data).hexdigest()!=r['sha256']:raise ValueError('archive integrity mismatch: '+r['source'])
  if not a.verify_only:
   dest=a.destination/relative;dest.parent.mkdir(parents=True,exist_ok=True)
   if dest.exists():
    if dest.read_bytes()!=data:raise FileExistsError('refusing to replace differing file: '+str(dest))
   else: dest.write_bytes(data)
  count+=1
 print('verified selected original records:',count)
if __name__=='__main__':main()
