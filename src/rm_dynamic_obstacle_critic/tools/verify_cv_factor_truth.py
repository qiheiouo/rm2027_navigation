#!/usr/bin/env python3
"""Independent vector interpolation and binary-ledger checks for factor labels."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from analyze_trial import load_truth_rows
from dynamic_consumption_io import read_consumption
from audit_scan_geometry import fixture


def verify(source, prepared):
    identity = json.loads((prepared/'input_identity.json').read_text())
    consumed, _ = read_consumption(source/'dynamic_scores/score_298.json')
    expected_names = ['actual_baseline', 'source_geometry_oracle', 'source_bracket_CV_oracle', 'future_circle_oracle']
    factors = identity['factors']
    if [f['name'] for f in factors] != expected_names or [f['index'] for f in factors] != list(range(4)):
        raise ValueError('factor intervention names/order')
    states = np.array([f['xy_vxy_size'] for f in factors], dtype=float)
    if states.shape != (4, 6) or not np.isfinite(states).all() or not np.isfinite(identity['future_centers']).all():
        raise ValueError('factor truth values must be finite')
    truth = load_truth_rows(source, json.loads((source/'execution.json').read_text()))
    times = np.array([r['t'] for r in truth])
    xy = np.array([r['obstacle'][:2] for r in truth])
    source_stamp = consumed['input_used']['stamp_sec']+consumed['input_used']['stamp_nanosec']*1e-9
    if [identity['source_stamp'], identity['score_clock'], identity['source_age_used']] != [source_stamp, consumed['score_stamp'], consumed['source_age_used']]:
        raise ValueError('factor clock provenance mismatch')
    future = consumed['score_stamp']+np.arange(1,31)*consumed['model_dt']
    if future[-1]>times[-1] or source_stamp<times[0]:
        raise ValueError('independent truth bracket missing')
    actual = np.stack([np.interp(future,times,xy[:,i]) for i in (0,1)],axis=-1)
    current = [float(np.interp(source_stamp,times,xy[:,i])) for i in (0,1)]
    j = np.searchsorted(times,source_stamp,side='right')
    slope = (xy[j]-xy[j-1])/(times[j]-times[j-1])
    errors = {'future_centers':float(np.max(np.abs(actual-np.array(identity['future_centers'])))),
              'source_center':float(np.max(np.abs(np.array(current)-identity['source_physical_pose'][:2]))),
              'source_bracket_slope':float(np.max(np.abs(slope-identity['source_velocity_bracket']['slope'])))}
    if not all(np.isfinite(v) for v in errors.values()) or max(errors.values())>2e-12 or future.tolist()!=identity['future_times']:
        raise ValueError('independent factor truth interpolation mismatch')
    usable = [t for t in consumed['input_used']['tracks'] if t['state'] in (2, 3)]
    at_score = np.array([np.interp(consumed['score_stamp'], times, xy[:,i]) for i in (0, 1)])
    distances = [np.linalg.norm(np.array(t['xy'])+np.array(t['vxy'])*consumed['source_age_used']-at_score) for t in usable]
    if not distances or min(distances)>1:
        raise ValueError('independent actor association missing')
    track = usable[int(np.argmin(distances))]
    if identity['associated_track_id'] != track['id'] or identity['other_usable_tracks'] != len(usable)-1:
        raise ValueError('independent actor association mismatch')
    original = [*track['xy'], *track['vxy'], *track['size_xy']]
    if states[0].tolist() != original or states[1,2:4].tolist() != track['vxy']:
        raise ValueError('actual baseline or retained velocity provenance mismatch')
    if np.max(np.abs(states[1:, :2]-current))>2e-12 or np.max(np.abs(states[2:, 2:4]-slope))>2e-12:
        raise ValueError('independent oracle intervention provenance mismatch')
    dimensions = fixture(source/'scene_inputs')['actor_dimensions'][:2]
    if any(f['xy_vxy_size'][4:]!=dimensions for f in factors[1:]):
        raise ValueError('full physical box dimensions not retained')
    data = (prepared/'factor_input.bin').read_bytes()
    if data[:8]!=b'RMDYNHX1' or len(data)!=708+3*311*30*4:
        raise ValueError('factor binary exact extent')
    ordinal,rows,steps,dt,track,count = struct.unpack_from('<IIIfQI',data,8)
    if [ordinal,rows,steps,dt,track,count]!=[298,311,30,identity['model_dt'],identity['associated_track_id'],4]:
        raise ValueError('factor binary header ledger')
    for i,factor in enumerate(identity['factors']):
        if data[36+i*48:36+(i+1)*48]!=struct.pack('<6d',*factor['xy_vxy_size']):
            raise ValueError('factor binary intervention ledger mismatch')
    for i,point in enumerate(identity['future_centers']):
        if data[228+i*16:228+(i+1)*16]!=struct.pack('<2d',*point):
            raise ValueError('factor binary future ledger mismatch')
    return {'verdict':'PASS','independent_scalar_vector_truth_max_error':errors,
            'binary_ledgers_exact':True,'future_times_use_score_clock_plus_k_plus_one_dt':True,
            'factor_input_sha256':hashlib.sha256(data).hexdigest(),
            'numerical_comparison_only':2e-12,
            'scope':'Conditional offline truth interpolation/binary provenance; no physical speed or future prediction certificate.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','prepared','output']:
        parser.add_argument(name,type=Path)
    args=parser.parse_args();result=verify(args.source,args.prepared)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
