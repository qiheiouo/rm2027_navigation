#!/usr/bin/env python3
"""Publisher identity and timing labels, with no optimizer/SG assignment."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
from trial_io import rows


def analyze(root):
    records=list(rows(root,'native_commands.jsonl'));counts=Counter();gids=defaultdict(set)
    finite=True;ordinals=[]
    for r in records:
        counts[r['publisher_node'] if r['publisher_node'] is not None else 'unknown']+=1
        gids[r['publisher_gid']].add((r['publisher_node'],r['publisher_namespace']))
        finite &= r['finite'];ordinals.append(r['ordinal'])
    return {'scope':'exact DDS GID matched to observer graph endpoints; publisher identity is not optimizer-cycle or SG identity',
        'records':len(records),'publisher_counts':dict(counts),'publisher_gid_count':len(gids),
        'all_publishers_known':None not in [r['publisher_node'] for r in records],
        'gid_node_mapping_consistent':all(len(v)==1 for v in gids.values()),
        'all_finite':finite,'contiguous_observer_ordinals':ordinals==list(range(len(ordinals))),
        'receive_sim_monotonic':all(b['receive_sim']>=a['receive_sim'] for a,b in zip(records,records[1:])),
        'zero_command_ordinals':[r['ordinal'] for r in records if all(v==0 for v in r['velocity'])],
        'endpoint_identities':{gid:[{'node':node,'namespace':namespace} for node,namespace in values] for gid,values in gids.items()},
        'limits':'DDS source/received timestamps are wall clock, not ROS pose time. Observer arrival and complete ordering do not prove every publisher message or expose internal optimizer resets/history.'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);args=parser.parse_args();result=analyze(args.trial)
    (args.trial/'native_command_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
