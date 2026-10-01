#!/usr/bin/env python3
"""Verified frozen snapshots and SG reconstruction to offline native model input."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct
import yaml
from native_snapshot_io import read_snapshot


def prepare(root,out,source_ordinals=None):
    root=root.resolve();manifest=json.loads((root/'manifest.json').read_text())
    for name,digest in manifest['files'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('frozen input identity mismatch: '+name)
    verified=json.loads((root/'native_optimizer/positive_comparison.json').read_text())
    if verified['verdict']!='RECONSTRUCTION EXACT':raise ValueError('actual history not numerically verified')
    native=[json.loads(line) for line in gzip.open(root/'native_optimizer/sdk_final_output.jsonl.gz','rt')]
    if native.pop(0)['kind']!='native_library':raise ValueError('native reconstruction identity')
    schedule=json.loads((root/'native_optimizer/schedule.json').read_text())
    follow=yaml.safe_load((root/'profile.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
    if follow['motion_model']!='Omni' or follow['time_steps']!=30 or abs(follow['model_dt']-.1)>1e-7:
        raise ValueError('witness requires registered native full three-second Omni grid')
    paths=sorted((root/'native_cycles').glob('cycle_*.json'),key=lambda p:int(p.stem.split('_')[1]))
    if len(paths)!=len(native) or len(native)!=verified['cycles']:raise ValueError('witness complete input count')
    selected=list(range(len(paths))) if source_ordinals is None else source_ordinals
    if (not selected or selected!=sorted(set(selected))
            or any(type(i) is not int or not 0<=i<len(paths) for i in selected)):
        raise ValueError('witness selected source ordinals')
    out.mkdir(parents=True,exist_ok=False);records=[]
    parameters=[follow[k] for k in ['model_dt','vx_min','vx_max','vy_max','wz_max']]
    with (out/'input.bin').open('xb') as stream:
        stream.write(b'RMSFWIT1');stream.write(struct.pack('<III5f',len(selected),follow['batch_size'],follow['time_steps'],*parameters))
        for ordinal,source_ordinal in enumerate(selected):
            path=paths[source_ordinal];reconstruction=native[source_ordinal];actual=schedule['records'][source_ordinal]
            meta,blocks=read_snapshot(path)
            if meta['ordinal']!=source_ordinal or reconstruction['ordinal']!=source_ordinal or actual['ordinal']!=source_ordinal:
                raise ValueError('witness cycle association gap')
            if (meta['pose_frame']!='odom' or meta['evaluation_frame']!='odom' or meta['base_frame']!='base_link'
                    or blocks['cvx']['shape']!=[follow['batch_size'],30]):
                raise ValueError('registered fixture frame or native grid mismatch')
            if not reconstruction['fixed_noise_input_exact']:raise ValueError('inexact reconstructed history chain')
            histories=[reconstruction[name] for name in ['history_before','mean_before_SG','mean_after_SG']]
            for values,length in zip(histories,[12,90,90]):
                if len(values)!=length or not all(math.isfinite(v) for v in values):raise ValueError('history/mean shape or finite')
            if reconstruction['returned_control']!=actual['actual_command']:raise ValueError('actual command mismatch')
            stream.write(struct.pack('<I13d',ordinal,*meta['pose'],*meta['speed']))
            for values in histories:stream.write(struct.pack('<'+str(len(values))+'f',*values))
            stream.write(struct.pack('<3d',*actual['actual_command']))
            for name in ['cvx','cvy','cwz','vx','vy','wz','x','y','yaw']:stream.write(blocks[name]['bytes'])
            stamp=meta['pose_stamp_sec']+meta['pose_stamp_nanosec']*1e-9
            records.append({'ordinal':ordinal,'source_ordinal':source_ordinal,'pose_stamp':stamp,'capture_stamp':meta['capture_stamp'],
                            'capture_minus_pose_stamp':meta['capture_stamp']-stamp,'actual_command':actual['actual_command']})
    result={'scope':'offline frozen fixture and numerically verified actual history; native model check before safety analysis',
            'cycles':len(records),'source_cycles':len(paths),'selected_source_ordinals':selected,'original_grid':[follow['batch_size'],30],
            'input_sha256':hashlib.sha256((out/'input.bin').read_bytes()).hexdigest(),
            'source_manifest_sha256':hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest(),
            'parameters':parameters,'records':records,
            'proposal_names':['actual_aggregate','zero','hold_measured','initial_world_x02_y0','initial_world_x02_yp03',
                              'initial_world_x02_ym03','initial_world_x04_y0','initial_world_x04_yp03','initial_world_x04_ym03',
                              'initial_world_x0_yp03','initial_world_x0_ym03'],
            'limits':'SG history is verified reconstruction, not direct live capture; source pose epoch differs from capture; actual measured twist source time unavailable; no safety or coverage verdict.'}
    (out/'input_identity.json').write_text(json.dumps(result,indent=2)+'\n');return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--source-ordinals',type=lambda text:[int(v) for v in text.split(',')])
    args=parser.parse_args()
    result=prepare(args.archive,args.output,args.source_ordinals)
    print(json.dumps({k:v for k,v in result.items() if k!='records' and (k!='selected_source_ordinals' or len(v)<=10)}))


if __name__=='__main__':main()
