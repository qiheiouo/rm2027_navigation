#!/usr/bin/env python3
"""Split reported guard evaluations by the fixed goal window and tail."""
import argparse
from collections import Counter
import json
from pathlib import Path
from trial_io import rows


def analyze(root):
    execution=json.loads((root/'execution.json').read_text())
    reasons={'active':Counter(),'tail':Counter()}
    for r in rows(root,'observations.jsonl'):
        if r['kind']!='guard' or not execution['start_sim']<=r['stamp']<=execution['last_sim']:continue
        phase='active' if r['stamp']<=execution['end_sim'] else 'tail'
        reasons[phase].update(s['reason'] for s in r['statuses'])
    return {'scope':'guard evaluation ROS stamp split by goal window and cancellation tail; no claim of exact action cancellation acknowledgement',
            'reasons_by_phase':{name:dict(value) for name,value in reasons.items()}}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);args=parser.parse_args()
    result=analyze(args.trial);(args.trial/'timing_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
