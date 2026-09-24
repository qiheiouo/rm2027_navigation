#!/usr/bin/env python3
"""Freeze this first-created V1 probe bundle without modifying source trials."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRIALS = ROOT / 'build/tdt_p2b/runs'
TARGETS = [HERE, ROOT / 'docs/dynamic_navigation/v1_prediction_scope.md',
           TRIALS / 'dynamic_prediction_tracker_live_v1',
           TRIALS / 'dynamic_prediction_tracker_live_v2']


def source_files():
    files = []
    for target in TARGETS:
        if target.is_file(): files.append(target)
        else: files.extend(p for p in target.rglob('*') if p.is_file())
    return sorted((p for p in files if p.name != 'manifest.json' and
                   '__pycache__' not in p.parts and '.pytest_cache' not in p.parts),
                  key=lambda p: str(p.relative_to(ROOT)))


def main():
    files = source_files()
    result = {'schema': 'rm_dynamic_prediction_v1_probe_manifest/v1',
              'scope': 'New V1 input probe files and both first-created live trial directories; manifest does not hash itself.',
              'files_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
              'file_count': len(files), 'accepted_for_deployment': False}
    with (HERE / 'manifest.json').open('x') as out:
        json.dump(result, out, indent=2); out.write('\n')
    print(result['file_count'])


if __name__ == '__main__': main()
