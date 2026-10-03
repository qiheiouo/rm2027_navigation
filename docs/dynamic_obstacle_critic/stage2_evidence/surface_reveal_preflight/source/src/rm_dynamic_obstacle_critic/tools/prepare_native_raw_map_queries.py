#!/usr/bin/env python3
"""Rebuild original300 raw30 and dense queries with exact frozen provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from analyze_trial import load_truth_rows
from native_snapshot_io import read_snapshot,measured_prefix_matches
from native_witness_io import records
from prepare_processed_cv import verify


def build_queries(source,witness,raw_archive):
    source,witness,raw_archive=[p.resolve() for p in [source,witness,raw_archive]]
    for root in [source,witness,raw_archive]:verify(root)
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json');selected=list(records(witness/'selected/witnesses.bin.gz'))
    prov=json.loads((witness/'provenance.json').read_text());old=json.loads((raw_archive/'analysis.json').read_text())
    if len(selected)!=1 or prov['selected_source_ordinal']!=298 or prov['source_manifest_sha256']!=hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest():
        raise ValueError('original source298 SDK witness required')
    selected=selected[0];dt=native['model_dt'];horizon=30*dt
    if blocks['x']['shape']!=[300,30] or native['ordinal']!=298 or not measured_prefix_matches(native,blocks) or selected['model_dt']!=dt or not all(selected[k] for k in ['raw_velocity_pose_exact','aggregate_SG_exact','actual_command_double_bit_exact']):
        raise ValueError('original raw300 verified SDK/measured prefix/time context required')
    if old['native_source_ordinal']!=298 or old['source_manifest_sha256']!=prov['source_manifest_sha256'] or old['witness_manifest_sha256']!=hashlib.sha256((witness/'manifest.json').read_bytes()).hexdigest() or old['raw_candidates']!=300:
        raise ValueError('original raw path labels require exact frozen references')
    grid=native['map']
    if grid['size']!=[100,100] or grid['resolution']!=.05 or len(native['padded_footprint'])!=4:raise ValueError('fixed native map/footprint required')
    truth=load_truth_rows(source,json.loads((source/'execution.json').read_text()));truth_times=np.array([r['t'] for r in truth])
    blob=bytearray(b'RMCOSTR1'+struct.pack('<8If4d',298,100,100,4,1200,142500,2,30,dt,grid['resolution'],*grid['origin'],horizon))
    blob.extend(struct.pack('<8d',*[v for p in native['padded_footprint'] for v in p]));blob.extend(blocks['raw_costmap']['values'])
    identities=[];query_count=0;native_poses=np.stack([np.asarray(blocks[k]['values'],dtype=np.float32).reshape(300,30).astype(float) for k in ['x','y','yaw']],axis=-1)
    for ei,epoch_name in enumerate(['actual_score_clock','source_pose_clock_sensitivity']):
        epoch=old['epoch_summaries'][epoch_name]['epoch'];knots=epoch+np.arange(31)*dt
        samples=np.unique(np.concatenate((knots,truth_times[(truth_times>epoch)&(truth_times<knots[-1])])))
        if epoch!=(48.004 if ei==0 else 47.98) or len(samples)!=(208 if ei==0 else 207) or old['epoch_summaries'][epoch_name]['horizon']!=horizon:raise ValueError('fixed complete raw query epochs')
        for kind in [2,3]:
            for row in range(300):
                if kind==2:
                    poses=np.empty((len(samples),3))
                    for axis in range(3):
                        initial=native['pose'][axis] if axis<2 else selected['initial_yaw'];values=np.concatenate(([initial],native_poses[row,:,axis]))
                        if axis==2:values=np.unwrap(values)
                        poses[:,axis]=np.interp(samples,knots,values)
                    times=samples-epoch
                else:
                    poses=native_poses[row];times=np.arange(1,31)*dt
                blob.extend(struct.pack('<4Id',kind,row,ei,len(times),epoch))
                for t,pose in zip(times,poses):blob.extend(struct.pack('<4d',t,*pose))
                identities.append({'trace':len(identities),'path_kind':'original_raw_dense' if kind==2 else 'original_native_raw30_grid','raw_row':row,
                    'epoch':epoch_name,'epoch_time':epoch,'samples':len(times),'pose_source':'interpolated x/y and unwrapped yaw' if kind==2 else 'original captured f32 x/y/yaw promoted exactly to double'})
                query_count+=len(times)
    if len(blob)!=4598940 or query_count!=142500:raise ValueError('raw1200 trace exact budget')
    result={'schema':'native_raw_map_query_input_v1','source_ordinal':298,'trace_count':1200,'query_count':query_count,
        'model_dt':dt,'horizon':horizon,'initial_yaw':selected['initial_yaw'],'map':grid,'padded_footprint':native['padded_footprint'],
        'input_sha256':hashlib.sha256(blob).hexdigest(),'map_raw_bytes_sha256':hashlib.sha256(blocks['raw_costmap']['values']).hexdigest(),
        'original_raw_xyz_block_sha256':{k:hashlib.sha256(blocks[k]['bytes']).hexdigest() for k in ['x','y','yaw']},
        'original_native_measured_speed':native['speed'],'original_measured_prefix_exact':True,'frozen_SDK_raw_model_exact':True,
        'traces':identities,'reference_manifests':{n:hashlib.sha256((r/'manifest.json').read_bytes()).hexdigest() for n,r in [('source',source),('witness',witness),('raw',raw_archive)]},
        'actual_CostCritic_full_state':'unknown','scope':'Separate original raw30 poses and dense interpolated model queries; no critic costs, actual branch/grid selection or execution.'}
    return bytes(blob),result


def prepare(source,witness,raw_archive,output):
    if output.exists():raise ValueError('refusing to overwrite raw map prepared inputs')
    blob,identity=build_queries(source,witness,raw_archive);output.mkdir()
    (output/'queries.bin').write_bytes(blob);(output/'input_identity.json').write_text(json.dumps(identity,indent=2,allow_nan=False)+'\n');return identity


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','raw_archive','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=prepare(args.source,args.witness,args.raw_archive,args.output)
    print(json.dumps({k:v for k,v in result.items() if k not in ['traces','map']}),flush=True)
