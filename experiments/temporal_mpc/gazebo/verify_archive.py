#!/usr/bin/env python3
"""Read-only byte integrity of compressed complete research evidence."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args();root=Path(args.directory)
    manifest=json.loads((root/'manifest.json').read_text());errors=[];checked=0;raw=0
    for item in manifest['files']:
        path=root/item['path'];data=path.read_bytes();checked+=1
        if len(data)!=item['bytes'] or hashlib.sha256(data).hexdigest()!=item['sha256']:errors.append(item['path']+': archive bytes')
        if path.name.endswith('.jsonl.gz'):
            original=gzip.decompress(data);raw+=1
            if hashlib.sha256(original).hexdigest()!=item['original_sha256']:errors.append(item['path']+': decompressed bytes')
            for line in original.splitlines():json.loads(line)
    result=dict(files_checked=checked,complete_record_streams_checked=raw,errors=errors,all_pass=not errors)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if not errors else 1)
