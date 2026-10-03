#!/usr/bin/env python3
"""Prepare a single genuine context plus four explicitly counterfactual costs."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from audit_dynamic_consumption import identity
from dynamic_consumption_io import read_consumption
from native_snapshot_io import read_snapshot, COST
from native_witness_io import records
from prepare_processed_cv import verify


FACTORS=['actual_baseline','source_geometry_oracle','source_bracket_CV_oracle','future_circle_oracle']


def prepare(source,witness,factors,output):
    for root in [source,witness,factors]:verify(root)
    references=json.loads((factors/'provenance.json').read_text())
    for root,key in [(source,'source'),(witness,'witness')]:
        if references[key+'_manifest_sha256']!=hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest():
            raise ValueError('factor archive reference mismatch')
    native,nb=read_snapshot(source/'native_cycles/cycle_298.json')
    consumed,cb=read_consumption(source/'dynamic_scores/score_298.json')
    if identity(native,nb)[1]!=identity(consumed,cb)[1] or nb['cvx']['shape']!=[300,30]:
        raise ValueError('complete fixed native/consume context identity')
    if nb[COST]['bytes']!=cb['costs_after_dynamic']['bytes']:
        raise ValueError('registered StaticStopping increment must be zero on all rows')
    if json.loads((source/'native_optimizer/positive_comparison.json').read_text())['verdict']!='RECONSTRUCTION EXACT':
        raise ValueError('actual SG history chain not verified')
    sdk=gzip.open(source/'native_optimizer/sdk_final_output.jsonl.gz','rb').read().splitlines(keepends=True)
    weighted=gzip.open(source/'native_optimizer/weights_output.jsonl.gz','rb').read().splitlines(keepends=True)
    if len(sdk)!=349 or len(weighted)!=349 or sdk[0]!=weighted[0]:
        raise ValueError('complete 348 native weight/SG chain required')
    for i,(original,line) in enumerate(zip(sdk[1:],weighted[1:])):
        row=json.loads(line)
        if (row['ordinal']!=i or line.split(b',"weights_reaggregate_bounded_mean_bit_exact"',1)[0]+b'}\n'!=original
                or not row['weights_reaggregate_bounded_mean_bit_exact'] or row['weights_reaggregate_max_error']!=0):
            raise ValueError('native chain/weights full numerical identity')
    reconstruction=json.loads(weighted[299])
    with gzip.open(factors/'factors.jsonl.gz','rt') as stream:
        data=[json.loads(line) for line in stream]
    header,entries=data[0],data[1:]
    if (header['source_ordinal']!=298 or header['actual_raw_risk_bit_exact_rows']!=300 or
            header['actual_raw_after_bit_exact_rows']!=300 or len(entries)!=2444):
        raise ValueError('factor scores must retain complete actual baseline')
    indexed={(r['factor'],r['path_kind'],r['row']):r for r in entries}
    if len(indexed)!=2444:raise ValueError('duplicate factor score')
    before=np.asarray(cb['costs_before_dynamic']['values'],dtype=np.float32)
    costs=[]
    for factor in FACTORS:
        risk=[indexed[(factor,'raw',i)]['risk'] for i in range(300)]
        if not np.isfinite(risk).all() or min(risk)<0:raise ValueError('finite nonnegative factor risk')
        if factor==FACTORS[0] and struct.pack('<300d',*risk)!=cb['dynamic_risk_double']['bytes']:
            raise ValueError('factor actual risk double bits')
        after=(before.astype(np.float64)+np.asarray(risk,dtype=np.float64)).astype(np.float32)
        if not np.isfinite(after).all():raise ValueError('counterfactual float cost overflow')
        costs.append(after.tobytes())
    if costs[0]!=nb[COST]['bytes']:raise ValueError('original float conversion/addition order mismatch')
    wit=list(records(witness/'selected/witnesses.bin.gz'))
    if len(wit)!=1 or wit[0]['row_count']!=311 or not all(wit[0][key] for key in ['raw_velocity_pose_exact','aggregate_SG_exact','actual_command_double_bit_exact']):
        raise ValueError('actual SDK baseline trajectory witness')
    schedule=json.loads((source/'native_optimizer/schedule.json').read_text())
    parameters=schedule['parameters']
    if len(parameters)!=10:raise ValueError('complete actual native parameters')
    output.mkdir();path=output/'aggregation_input.bin'
    with path.open('xb') as stream:
        stream.write(b'RMCVAGH1'+struct.pack('<4I',298,300,30,4))
        stream.write(struct.pack('<10f',*parameters))
        stream.write(struct.pack('<13d',*native['pose'],*native['speed']))
        for key,count in [('history_before',12),('mean_before',90),('mean_before_SG',90),('mean_after_SG',90)]:
            values=reconstruction[key]
            if len(values)!=count:raise ValueError('native history/control dimension')
            stream.write(struct.pack('<'+str(count)+'f',*values))
        stream.write(struct.pack('<3d',*reconstruction['returned_control']))
        for name in ['cvx','cvy','cwz','vx','vy','wz','x','y','yaw']:stream.write(nb[name]['bytes'])
        for blob in costs:stream.write(blob)
        for key in ['native_costs_after_regularization','native_softmax_weights']:
            values=reconstruction[key]
            if len(values)!=300:raise ValueError('native gamma/weight dimension')
            stream.write(struct.pack('<300f',*values))
        for name in ['cvx','cvy','cwz','vx','vy','wz','x','y','yaw']:
            stream.write(struct.pack('<30f',*wit[0]['blocks'][name][:30]))
    if path.stat().st_size!=333600:raise ValueError('private aggregation exact byte budget')
    result={'schema':1,'scope':'Fixed actual context and four explicitly offline dynamic-cost interventions; no new actual event.',
            'source_ordinal':298,'model_dt':native['model_dt'],'source_pose_clock':native['pose_stamp_sec']+native['pose_stamp_nanosec']*1e-9,
            'score_clock':consumed['score_stamp'],'actual_command':reconstruction['returned_control'],'parameters':parameters,
            'factor_names':FACTORS,'static_increment_zero_all_300':True,'whole_348_weight_SG_chain_exact':True,
            'factor_float_cost_sha256':[hashlib.sha256(blob).hexdigest() for blob in costs],
            'aggregation_input_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'source_manifests':{label:hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() for label,root in [('source',source),('witness',witness),('factors',factors)]},
            'limits':'All oracle factors are inadmissible online; baseline numerical proof precedes hypothetical aggregation; existing FAILED/NOT ESTABLISHED remain.'}
    (output/'input_identity.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','factors','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();print(json.dumps(prepare(args.source.resolve(),args.witness.resolve(),args.factors.resolve(),args.output)),flush=True)
