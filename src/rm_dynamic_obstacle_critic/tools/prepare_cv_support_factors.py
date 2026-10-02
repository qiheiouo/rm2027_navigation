#!/usr/bin/env python3
"""Prepare explicit offline geometry/local-CV/future-circle interventions."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
from analyze_trial import load_truth_rows
from audit_scan_geometry import fixture
from dynamic_consumption_io import read_consumption
from native_witness_io import records
from prepare_processed_cv import verify


def prepare(source, witness, processed, output):
    verify(source)
    verify(witness)
    processed_identity = json.loads((processed/'input_identity.json').read_text())
    if processed_identity['source_native_ordinal'] != 298:
        raise ValueError('registered source298 processed context required')
    for name, key in [('actual_input/input.bin', 'actual_input_sha256'), ('processed_paths.bin', 'processed_paths_sha256')]:
        if hashlib.sha256((processed/name).read_bytes()).hexdigest() != processed_identity[key]:
            raise ValueError('processed context identity mismatch')
    for root, key in [(source, 'source_manifest_sha256'), (witness, 'witness_manifest_sha256')]:
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() != processed_identity[key]:
            raise ValueError('processed referenced archive mismatch')
    consumed, _ = read_consumption(source/'dynamic_scores/score_298.json')
    if consumed['world_transform'] != [0, 0, 0] or consumed['input_used']['frame'] != 'map':
        raise ValueError('factor fixture requires the registered identity world alignment')
    truth = load_truth_rows(source, json.loads((source/'execution.json').read_text()))
    times = [row['t'] for row in truth]
    def actual(query):
        j = bisect.bisect_right(times, query)
        if not 0 < j < len(times):
            raise ValueError('factor source/future truth bracket missing')
        a, b = truth[j-1], truth[j]
        fraction = (query-a['t'])/(b['t']-a['t'])
        xy = [a['obstacle'][i]+fraction*(b['obstacle'][i]-a['obstacle'][i]) for i in (0, 1)]
        yaw = a['obstacle'][2]+fraction*math.remainder(b['obstacle'][2]-a['obstacle'][2], 2*math.pi)
        return [*xy, yaw], a, b
    now = consumed['score_stamp']
    source_stamp = consumed['input_used']['stamp_sec']+consumed['input_used']['stamp_nanosec']*1e-9
    age = consumed['source_age_used']
    at_now = actual(now)[0]
    usable = [t for t in consumed['input_used']['tracks'] if t['state'] in (2, 3)]
    associated = min(usable, key=lambda t: math.dist([t['xy'][i]+t['vxy'][i]*age for i in (0, 1)], at_now[:2])) if usable else None
    if associated is None or math.dist([associated['xy'][i]+associated['vxy'][i]*age for i in (0, 1)], at_now[:2]) > 1:
        raise ValueError('fixed 1m offline actor association failed')
    state, a, b = actual(source_stamp)
    slope = [(b['obstacle'][i]-a['obstacle'][i])/(b['t']-a['t']) for i in (0, 1)]
    dimensions = fixture(source/'scene_inputs')['actor_dimensions'][:2]
    factors = [('actual_baseline', [*associated['xy'], *associated['vxy'], *associated['size_xy']]),
               ('source_geometry_oracle', [*state[:2], *associated['vxy'], *dimensions]),
               ('source_bracket_CV_oracle', [*state[:2], *slope, *dimensions]),
               ('future_circle_oracle', [*state[:2], *slope, *dimensions])]
    dt = consumed['model_dt']
    future_times = [now+(step+1)*dt for step in range(30)]
    centers = [actual(t)[0][:2] for t in future_times]
    data = list(records(witness/'selected/witnesses.bin.gz'))
    if len(data)!=1 or data[0]['row_count']!=311 or data[0]['steps']!=30 or data[0]['model_dt']!=dt:
        raise ValueError('complete registered SDK processed matrix required')
    record = data[0]
    if not (record['raw_velocity_pose_exact'] and record['aggregate_SG_exact'] and record['actual_command_double_bit_exact']):
        raise ValueError('native SDK integrity required')
    output.mkdir()
    shutil.copyfile(processed/'actual_input/input.bin', output/'actual_input.bin')
    path = output/'factor_input.bin'
    with path.open('xb') as stream:
        stream.write(b'RMDYNHX1'+struct.pack('<IIIfQI', 298, 311, 30, dt, associated['id'], 4))
        for _, values in factors:
            stream.write(struct.pack('<6d', *values))
        for center in centers:
            stream.write(struct.pack('<2d', *center))
        for name in ['x', 'y', 'yaw']:
            for value in record['blocks'][name]:
                stream.write(struct.pack('<f', value))
    report = {'schema': 1, 'scope': 'Offline factor interventions; source/future truth never published or represented as actual consumption.',
              'source_manifest_sha256': processed_identity['source_manifest_sha256'],
              'witness_manifest_sha256': processed_identity['witness_manifest_sha256'],
              'processed_input_identity_sha256': hashlib.sha256((processed/'input_identity.json').read_bytes()).hexdigest(),
              'actual_input_sha256': hashlib.sha256((output/'actual_input.bin').read_bytes()).hexdigest(),
              'factor_input_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'source_native_ordinal': 298, 'associated_track_id': associated['id'], 'other_usable_tracks': len(usable)-1,
              'source_stamp': source_stamp, 'score_clock': now, 'source_age_used': age, 'model_dt': dt,
              'source_physical_pose': state, 'source_velocity_bracket': {'before_stamp': a['t'], 'after_stamp': b['t'],
                  'before_xy': a['obstacle'][:2], 'after_xy': b['obstacle'][:2], 'slope': slope,
                  'meaning': 'finite truth bracket slope; not certified instantaneous or maximum velocity'},
              'factors': [{'index': index, 'name': name, 'xy_vxy_size': values} for index, (name, values) in enumerate(factors)],
              'future_times': future_times, 'future_centers': centers,
              'physical_motion_bound': 'unknown',
              'limits': 'Conditional fixture factor labels only; geometry/source/future oracle inputs are inadmissible online.'}
    (output/'input_identity.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source', 'witness_archive', 'processed_input', 'output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    report = prepare(args.source.resolve(), args.witness_archive.resolve(), args.processed_input, args.output)
    print(json.dumps({k:v for k,v in report.items() if k not in ['future_times','future_centers']}), flush=True)
