#!/usr/bin/env python3
"""Baseline bit regressions and explicit hypothesis format corruption probes."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared', type=Path)
    parser.add_argument('output', type=Path)
    for name in ['binary', 'raw-reference', 'processed-reference']:
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir()
    checks = {}
    actual = args.prepared/'actual_input.bin'
    hypotheses = args.prepared/'factor_input.bin'
    def run(source, target, label):
        with (args.output/(label+'.log')).open('x') as stream:
            return subprocess.run([str(args.binary), str(actual), str(source), str(target)],
                                  stdout=stream, stderr=subprocess.STDOUT).returncode
    result = args.output/'factors.jsonl'
    code = run(hypotheses, result, 'positive')
    if code != 0:
        raise ValueError('factor evaluator positive failed')
    rows = [json.loads(line) for line in result.read_text().splitlines()]
    header, entries = rows[0], rows[1:]
    factors = ['actual_baseline', 'source_geometry_oracle', 'source_bracket_CV_oracle', 'future_circle_oracle']
    expected = {(factor, kind, row) for factor in factors for kind, count in [('raw', 300), ('processed', 311)] for row in range(count)}
    checks['four_complete_factors_on_300_and_311_paths'] = (len(entries)==2444 and
        {(r['factor'], r['path_kind'], r['row']) for r in entries}==expected and
        header['actual_raw_risk_bit_exact_rows']==300 and header['actual_raw_after_bit_exact_rows']==300)
    with gzip.open(args.raw_reference, 'rt') as stream:
        raw = next(json.loads(line) for line in stream if json.loads(line)['ordinal']==298)
    processed = [json.loads(line) for line in args.processed_reference.read_text().splitlines()][1:]
    def equal(a, b):
        return a is None and b is None or a is not None and b is not None and struct.pack('<d', a)==struct.pack('<d', b)
    for kind, count, reference in [('raw', 300, raw), ('processed', 311, processed)]:
        baseline = [r for r in entries if r['factor']=='actual_baseline' and r['path_kind']==kind]
        checks[kind+'_baseline_all_risk_clearance_TTC_bits_exact'] = len(baseline)==count and all(
            r['row']==index and all(equal(r[field], reference[field][index] if kind=='raw' else reference[index][field])
                                   for field in ['risk', 'minimum_clearance', 'collision_time'])
            for index, r in enumerate(baseline))
    blob = hypotheses.read_bytes()
    identity = json.loads((args.prepared/'input_identity.json').read_text())
    reduction = bytearray(blob)
    reduction[36+3*48:36+4*48] = reduction[36:36+48]
    state = identity['factors'][0]['xy_vxy_size']
    for step in range(30):
        elapsed = identity['source_age_used']+(step+1)*identity['model_dt']
        struct.pack_into('<2d', reduction, 228+step*16, state[0]+state[2]*elapsed, state[1]+state[3]*elapsed)
    reduction_path = args.output/'future_circle_CV_reduction_probe.bin'
    reduction_path.write_bytes(reduction)
    reduction_result = args.output/'future_circle_CV_reduction_probe.jsonl'
    code = run(reduction_path, reduction_result, 'future_circle_CV_reduction_probe')
    reduced = [json.loads(line) for line in reduction_result.read_text().splitlines()][1:] if code==0 else []
    baseline_lookup = {(r['path_kind'],r['row']):r for r in entries if r['factor']=='actual_baseline'}
    reduced_future = [r for r in reduced if r['factor']=='future_circle_oracle']
    checks['future_circle_reduces_bit_exact_to_CV_on_synthetic_CV_centers'] = code==0 and len(reduced_future)==611 and all(
        all(equal(row[field],baseline_lookup[(row['path_kind'],row['row'])][field]) for field in ['risk','minimum_clearance','collision_time'])
        for row in reduced_future)
    def mutation(offset, value, fmt):
        data = bytearray(blob)
        struct.pack_into(fmt, data, offset, value)
        return data
    dt_bits = struct.unpack_from('<I', blob, 20)[0]
    probes = [('schema', b'x'+blob[1:], 'factor hypothesis schema'),
              ('source', mutation(8, 299, '<I'), 'factor source/grid/dt/count mismatch'),
              ('rows', mutation(12, 513, '<I'), 'factor source/grid/dt/count mismatch'),
              ('dt', mutation(20, dt_bits+1, '<I'), 'factor source/grid/dt/count mismatch'),
              ('track_id', mutation(24, 2, '<Q'), 'factor actor id/state mismatch'),
              ('factor_count', mutation(32, 5, '<I'), 'factor source/grid/dt/count mismatch'),
              ('nonfinite_state', mutation(36, float('nan'), '<d'), 'nonfinite factor state'),
              ('changed_actual_baseline', mutation(36, 5.0, '<d'), 'factor baseline state is not actual'),
              ('nonpositive_size', mutation(36+4*8, 0, '<d'), 'factor physical size invalid'),
              ('nonfinite_future', mutation(228, float('nan'), '<d'), 'nonfinite oracle center'),
              ('nonfinite_processed', mutation(708, float('nan'), '<f'), 'nonfinite factor path'),
              ('truncated', blob[:-1], 'truncated dynamic replay'),
              ('trailing', blob+b'x', 'trailing factor input')]
    for name, data, reason in probes:
        source = args.output/(name+'.bin')
        source.write_bytes(data)
        target = args.output/(name+'.jsonl')
        code = run(source, target, name)
        checks[name+'_rejected'] = code==1 and reason in (args.output/(name+'.log')).read_text() and not target.exists()
    digest = hashlib.sha256(result.read_bytes()).hexdigest()
    code = run(hypotheses, result, 'existing_output')
    checks['existing_output_preserved'] = code==1 and hashlib.sha256(result.read_bytes()).hexdigest()==digest
    bad_actual = bytearray(actual.read_bytes())
    _, batch, steps, tracks, points = struct.unpack_from('<5I', bad_actual, 12)
    risk_offset = 12+20+80+16*points+57*tracks+3*batch*steps*4+batch*4
    bits = struct.unpack_from('<Q', bad_actual, risk_offset)[0]
    struct.pack_into('<Q', bad_actual, risk_offset, bits+1)
    bad_path = args.output/'corrupted_actual_risk.bin'
    bad_path.write_bytes(bad_actual)
    target = args.output/'corrupted_actual_risk.jsonl'
    with (args.output/'corrupted_actual_risk.log').open('x') as stream:
        code = subprocess.run([str(args.binary),str(bad_path),str(hypotheses),str(target)],stdout=stream,stderr=subprocess.STDOUT).returncode
    checks['corrupted_actual_risk_rejected'] = code==1 and 'actual raw risk/after bits mismatch' in (args.output/'corrupted_actual_risk.log').read_text() and not target.exists()
    report = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'checks': checks,
              'factor_binary_sha256': hashlib.sha256(args.binary.read_bytes()).hexdigest(),
              'cpp_output_sha256': digest,
              'scope': 'Offline actual-baseline identity and bounded hypothesis reader; no oracle is an online input.'}
    (args.output/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)
    return 0 if report['verdict']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
