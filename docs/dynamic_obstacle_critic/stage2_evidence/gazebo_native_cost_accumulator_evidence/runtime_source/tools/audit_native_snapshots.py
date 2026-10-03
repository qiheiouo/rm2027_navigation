#!/usr/bin/env python3
"""Candidate capture integrity and pre-optimizer descriptive metrics only."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import yaml
from native_snapshot_io import read_snapshot,measured_prefix_matches,COST


def analyze(root):
    follow=yaml.safe_load((root/'profile.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
    captures=[];identities={};prefix=True;shift=True;grid=True;ordinals=[];nonfailed=True
    for path in sorted((root/'native_cycles').glob('cycle_*.json'),key=lambda p:int(p.stem.split('_')[1])):
        meta,blocks=read_snapshot(path);ordinal=meta['ordinal'];ordinals.append(ordinal)
        nonfailed &= not meta['fail_flag']
        batch,steps=blocks['x']['shape'];costs=blocks[COST]['values']
        first=measured_prefix_matches(meta,blocks);prefix &= first
        shifted=all(all(blocks[v]['bytes'][(i*steps+1)*4:(i*steps+steps)*4]==blocks[c]['bytes'][i*steps*4:(i*steps+steps-1)*4]
            for i in range(batch)) for v,c in [('vx','cvx'),('vy','cvy'),('wz','cwz')]);shift &= shifted
        full=(batch==follow['batch_size'] and steps==follow['time_steps'] and abs(meta['model_dt']-follow['model_dt'])<1e-6);grid &= full
        pose_stamp=meta['pose_stamp_sec']+meta['pose_stamp_nanosec']*1e-9
        captures.append({'ordinal':ordinal,'pose_stamp':pose_stamp,'capture_stamp':meta['capture_stamp'],
            'capture_minus_pose_stamp':meta['capture_stamp']-pose_stamp,'grid':[batch,steps],
            'actual_measured_prefix_exact':first,'native_velocity_control_shift_exact':shifted,
            'pre_regularization_cost_min':min(costs),'pre_regularization_cost_max':max(costs),
            'pre_regularization_argmin':min(range(batch),key=costs.__getitem__)})
        identities[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
        payload_hash=hashlib.sha256()
        for block in meta['blocks']:payload_hash.update(blocks[block['name']]['bytes'])
        identities[str(path.with_suffix('.bin').relative_to(root))]=payload_hash.hexdigest()
    text=((root/'launch.log').read_text() if (root/'launch.log').exists()
          else gzip.open(root/'launch.log.gz','rt').read())
    timings=[float(v) for v in re.findall(r'NativeSnapshot ordinal=\d+ capture_ms=([0-9.]+)',text)]
    monotonic=all(b['capture_stamp']>=a['capture_stamp'] for a,b in zip(captures,captures[1:]))
    gate=(bool(captures) and prefix and shift and grid and nonfailed and ordinals==list(range(len(ordinals)))
          and monotonic and len(captures)<=follow['NativeCycleSnapshotCritic']['max_snapshots'])
    return {'verdict':'CAPTURE INTEGRITY PASS' if gate else 'FAILED','scope':'native candidate evidence before gamma/softmax/aggregation/SG; not control safety or coverage proof',
        'capture_count':len(captures),'actual_measured_prefix_exact':prefix,'native_velocity_control_shift_exact':shift,'full_configured_grid':grid,
        'contiguous_captured_ordinals':ordinals==list(range(len(ordinals))),
        'sampling_period':follow['NativeCycleSnapshotCritic']['capture_period'],
        'budget':follow['NativeCycleSnapshotCritic']['max_snapshots'],'budget_reached':len(captures)>=follow['NativeCycleSnapshotCritic']['max_snapshots'],
        'capture_times_monotonic':monotonic,
        'logged_capture_ms_samples':len(timings),'logged_capture_ms_median':statistics.median(timings) if timings else None,
        'logged_capture_ms_max':max(timings) if timings else None,
        'records':captures,'logical_uncompressed_file_sha256':identities,
        'hash_scope':'decoded payloads under logical .bin names, independent of gzip storage; metadata files are byte hashed',
        'unavailable':['control_sequence_mean','SG_history','exact_dynamic_consumer_input','returned_control','native_costmap_source_stamp'],
        'limits':'Capture ordinals do not count optimizer calls skipped after earlier critic failure. Costs are pre-regularization, not final weights. Pose stamp and capture time differ. No observer command/SG association or sampler responsibility is proved.'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);args=parser.parse_args();result=analyze(args.trial)
    (args.trial/'native_snapshot_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('records','logical_uncompressed_file_sha256')}))


if __name__=='__main__':main()
