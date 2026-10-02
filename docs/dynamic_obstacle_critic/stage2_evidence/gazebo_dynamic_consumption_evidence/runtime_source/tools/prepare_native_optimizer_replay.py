#!/usr/bin/env python3
"""Prepare a fixed trace association hypothesis for independent native validation."""
from decimal import Decimal
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import yaml
from native_snapshot_io import read_snapshot,COST
from trial_io import rows


def prepare(root,out,association_window=.01):
    out.mkdir(parents=True,exist_ok=False)
    all_commands=list(rows(root,'native_commands.jsonl'))
    if not all(r['publisher_node']=='controller_server' and r['finite'] for r in all_commands):
        raise ValueError('this fixed-trace hypothesis requires known controller commands')
    if ([r['ordinal'] for r in all_commands]!=list(range(len(all_commands)))
            or any(b['dds_source_timestamp_ns']<=a['dds_source_timestamp_ns'] for a,b in zip(all_commands,all_commands[1:]))):
        raise ValueError('fixed-trace command identity/order')
    commands=[r for r in all_commands if any(v!=0 for v in r['velocity'])]
    paths=sorted((root/'native_cycles').glob('cycle_*.json'),key=lambda p:int(p.stem.split('_')[1]))
    if len(paths)!=len(commands):raise ValueError('nonzero command/capture counts differ; no association proof')
    log=(root/'launch.log').read_text() if (root/'launch.log').exists() else gzip.open(root/'launch.log.gz','rt').read()
    reset_times=[int(Decimal(value)*1_000_000_000) for value in re.findall(
        r'\[controller_server-\d+\] \[INFO\] \[([0-9.]+)\] \[controller_server\]: Optimizer reset',log)]
    profile=yaml.safe_load((root/'profile.yaml').read_text());f=profile['controller_server']['ros__parameters']['FollowPath']
    if f['motion_model']!='Omni' or f['iteration_count']!=1 or f['regenerate_noises']:
        raise ValueError('reconstruction requires fixed noises, one iteration and Omni')
    if abs(1/profile['controller_server']['ros__parameters']['controller_frequency']-f['model_dt'])>=1e-6:
        raise ValueError('reconstruction requires native offset=1')
    if not math.isfinite(association_window) or not 0<association_window<f['model_dt']:
        raise ValueError('association window must be positive and below one native period')
    parameters=[f[k] for k in ['model_dt','temperature','gamma','vx_std','vy_std','wz_std','vx_min','vx_max','vy_max','wz_max']]
    records=[];previous=None
    with (out/'input.bin').open('wb') as stream:
        stream.write(b'RMSGRPL1');stream.write(struct.pack('<III',len(paths),f['batch_size'],f['time_steps']))
        stream.write(struct.pack('<10f',*parameters))
        for i,(path,command) in enumerate(zip(paths,commands)):
            meta,blocks=read_snapshot(path)
            if meta['ordinal']!=i:raise ValueError('capture ordinal gap')
            if blocks['cvx']['shape']!=[f['batch_size'],f['time_steps']]:
                raise ValueError('capture profile grid')
            delay=command['receive_sim']-meta['capture_stamp']
            if not 0<=delay<=association_window:raise ValueError('ordered command association fails time check')
            events=[stamp for stamp in reset_times if (previous is None or previous<stamp) and stamp<=command['dds_source_timestamp_ns']]
            reset=i==0 or bool(events);stream.write(struct.pack('<II',i,int(reset)))
            for key in ['cvx','cvy','cwz',COST]:stream.write(blocks[key]['bytes'])
            records.append({'ordinal':i,'command_ordinal':command['ordinal'],'capture_stamp':meta['capture_stamp'],
                'command_receive_sim':command['receive_sim'],'association_delay':delay,'reset':reset,
                'reset_log_wall_stamps':events,'actual_command':[command['velocity'][k] for k in [0,1,5]]})
            previous=command['dds_source_timestamp_ns']
    result={'scope':'fixed-trace ordered association hypothesis; requires independent input/output numerical validation',
        'parameters':parameters,'records':records,
        'excluded_zero_command_ordinals':[r['ordinal'] for r in all_commands if all(v==0 for v in r['velocity'])],
        'reset_ordinals':[r['ordinal'] for r in records if r['reset']],
        'input_sha256':hashlib.sha256((out/'input.bin').read_bytes()).hexdigest(),
        'limits':'Zero commands are excluded only as a hypothesis for this trace, not a general stop/optimizer classifier. Reset logs supply candidate segment anchors. No safety or SG proof without native replay.'}
    # Preserve historical default reports byte-for-byte. A wider observer
    # receipt window is explicit evidence of a different association hypothesis,
    # never a tolerance in the independent tensor/output bit comparison.
    if association_window!=.01:result['association_window_seconds']=association_window
    (out/'schedule.json').write_text(json.dumps(result,indent=2)+'\n');return result


def omit_resets(input_path,output_path,ordinals):
    """Explicit negative control: copy the input and remove only named reset flags."""
    data=bytearray(input_path.read_bytes())
    if data[:8]!=b'RMSGRPL1':raise ValueError('negative replay schema')
    count,batch,steps=struct.unpack_from('<III',data,8);size=8+3*batch*steps*4+batch*4
    if len(data)!=60+count*size:raise ValueError('negative replay length')
    for ordinal in ordinals:
        if not 0<ordinal<count:raise ValueError('negative reset ordinal')
        offset=60+ordinal*size+4
        if struct.unpack_from('<I',data,offset)[0]!=1:raise ValueError('negative reset not present')
        struct.pack_into('<I',data,offset,0)
    with output_path.open('xb') as stream:stream.write(data)
    return hashlib.sha256(data).hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--association-window',type=float,default=.01);args=parser.parse_args()
    result=prepare(args.trial,args.output,args.association_window);print(json.dumps({k:v for k,v in result.items() if k not in ('records','parameters')}))


if __name__=='__main__':main()
