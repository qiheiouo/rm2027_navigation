#!/usr/bin/env python3
"""Prepare full frozen-map SDK queries for eight stages and 43 SG individuals."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from analyze_trial import load_truth_rows
from native_sg_stage_io import read_paths
from native_snapshot_io import read_snapshot
from native_witness_io import records
from prepare_processed_cv import verify


def build_queries(source,witness,stages,mass):
    for root in [source,witness,stages,mass]:verify(root)
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json');stage=read_paths(stages/'stage_paths.bin')
    selected=list(records(witness/'selected/witnesses.bin.gz'))
    if len(selected)!=1 or selected[0]['row_count']!=311 or not all(selected[0][k] for k in ['raw_velocity_pose_exact','aggregate_SG_exact','actual_command_double_bit_exact']):
        raise ValueError('verified SDK311 individual context required')
    selected=selected[0];report=json.loads(gzip.open(mass/'report.json.gz','rb').read());scalar=json.loads(gzip.open(mass/'scalar_verification.json.gz','rb').read())
    indices=report['independent_raw_subset']
    if report['native_source_ordinal']!=298 or len(indices)!=43 or scalar['verified_sampler_raw_rows']!=indices or scalar['verdict']!='PASS':raise ValueError('original43 validated individuals required')
    grid=native['map'];dt=stage['model_dt'];horizon=30*dt;epochs=report['epoch_summaries']
    if grid['size']!=[100,100] or grid['resolution']!=.05 or len(native['padded_footprint'])!=4 or selected['model_dt']!=dt or selected['initial_yaw']!=stage['initial_yaw']:
        raise ValueError('fixed map/footprint/SDK time identity')
    truth=load_truth_rows(source,json.loads((source/'execution.json').read_text()));truth_times=np.array([r['t'] for r in truth])
    blob=bytearray(b'RMCOSTQ1'+struct.pack('<8If4d',298,100,100,4,102,21165,2,30,dt,grid['resolution'],*grid['origin'],horizon))
    blob.extend(struct.pack('<8d',*[v for p in native['padded_footprint'] for v in p]));blob.extend(blocks['raw_costmap']['values'])
    trace_identity=[];count=0
    for ei,epoch_name in enumerate(['actual_score_clock','source_pose_clock_sensitivity']):
        epoch=epochs[epoch_name]['epoch'];knots=epoch+np.arange(31)*dt
        samples=np.unique(np.concatenate((knots,truth_times[(truth_times>epoch)&(truth_times<knots[-1])])))
        if len(samples)!=(208 if ei==0 else 207) or len(samples)!=epochs[epoch_name]['samples'] or epochs[epoch_name]['horizon']!=horizon:
            raise ValueError('full208/207 sample grids required')
        for kind,row,path_row in [(0,i,i) for i in range(8)]+[(1,i,i+11) for i in indices]:
            source_blocks=stage['blocks'] if kind==0 else {k:{'values':v} for k,v in selected['blocks'].items()}
            poses=np.empty((len(samples),3))
            for axis,name in enumerate(['x','y','yaw']):
                initial=native['pose'][axis] if axis<2 else stage['initial_yaw']
                values=np.concatenate(([initial],source_blocks[name]['values'][path_row*30:(path_row+1)*30]))
                if axis==2:values=np.unwrap(values)
                poses[:,axis]=np.interp(samples,knots,values)
            blob.extend(struct.pack('<4Id',kind,row,ei,len(samples),epoch))
            for t,pose in zip(samples,poses):blob.extend(struct.pack('<4d',t-epoch,*pose))
            trace_identity.append({'trace':len(trace_identity),'path_kind':'native_stage' if kind==0 else 'individual_bounds_SG',
                'path_row':row,'witness_row':row+11 if kind else None,'stage':stage['row_identity'][row][1] if kind==0 else 'individual_actual_history_SG',
                'epoch':epoch_name,'epoch_time':epoch,'samples':len(samples)})
            count+=len(samples)
    if len(blob)!=689868 or count!=21165:raise ValueError('exact 102-trace native map query budget')
    result={'schema':'native_map_query_input_v1','source_ordinal':298,'trace_count':102,'query_count':count,'individual_raw_rows':indices,
        'map_raw_bytes_sha256':hashlib.sha256(blocks['raw_costmap']['values']).hexdigest(),'input_sha256':hashlib.sha256(blob).hexdigest(),
        'model_dt':dt,'horizon':horizon,'map':grid,'padded_footprint':native['padded_footprint'],'traces':trace_identity,
        'reference_manifests':{name:hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() for name,root in [('source',source),('witness',witness),('stages',stages),('mass',mass)]},
        'actual_CostCritic_full_state':'unknown: no captured internal inflation cache, layered costmap state or individual critic increments',
        'scope':'Native checker queries on frozen map, not actual CostCritic score/collision events or execution.'}
    return bytes(blob),result


def prepare(source,witness,stages,mass,output):
    if output.exists():raise ValueError('refusing to overwrite native map prepared inputs')
    blob,result=build_queries(source,witness,stages,mass)
    output.mkdir();(output/'queries.bin').write_bytes(blob);(output/'input_identity.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','stages','mass','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=prepare(*[getattr(args,n).resolve() for n in ['source','witness','stages','mass']],args.output)
    print(json.dumps({k:v for k,v in result.items() if k not in ['traces','map']}),flush=True)
