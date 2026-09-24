#!/usr/bin/env python3
"""Hash the new extent-shadow bundle; source trial hashes are in extent_shadow.json."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
files=sorted(p for p in HERE.iterdir() if p.is_file() and p.name!='manifest.json')
result={'schema':'rm_dynamic_prediction_v1_extent_manifest/v1',
        'files_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'file_count':len(files),'accepted_for_deployment':False}
with (HERE/'manifest.json').open('x') as output:
    json.dump(result,output,indent=2);output.write('\n')
print(result['file_count'])
