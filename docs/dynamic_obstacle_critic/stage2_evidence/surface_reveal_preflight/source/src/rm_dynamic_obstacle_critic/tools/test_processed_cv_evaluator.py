#!/usr/bin/env python3
"""Actual offline executable regressions and bounded-input corruption probes."""
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
    for name in ['evaluator', 'replay', 'full-input', 'full-reference', 'fixtures']:
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir()
    checks = {}
    def run(binary, arguments, label):
        with (args.output/(label+'.log')).open('x') as stream:
            return subprocess.run([str(binary), *map(str, arguments)], stdout=stream, stderr=subprocess.STDOUT).returncode
    full = args.output/'full_raw_regression.jsonl'
    code = run(args.replay, [args.full_input, full], 'full_raw_regression')
    checks['348_actual_raw_scores_byte_exact'] = code==0 and full.read_bytes()==gzip.open(args.full_reference, 'rb').read()
    for fixture in sorted(args.fixtures.iterdir()):
        if not (fixture/'input.bin').is_file():
            continue
        result = args.output/(fixture.name+'.jsonl')
        code = run(args.replay, [fixture/'input.bin', result], fixture.name)
        checks[fixture.name+'_byte_exact'] = code==0 and result.read_bytes()==(fixture/'native_output.jsonl').read_bytes()
        negative = fixture/'negative_one_risk_ulp_input.bin'
        if negative.is_file():
            result = args.output/'one_risk_ULP.jsonl'
            code = run(args.replay, [negative, result], 'one_risk_ULP')
            checks['original_one_risk_ULP_rejected_byte_exact'] = code==2 and result.read_bytes()==(fixture/'negative_native_output.jsonl').read_bytes()
    actual = args.prepared/'actual_input/input.bin'
    paths = args.prepared/'processed_paths.bin'
    positive = args.output/'processed_cv.jsonl'
    code = run(args.evaluator, [actual, paths, positive], 'processed_positive')
    if code != 0:
        raise ValueError('processed positive evaluator failed')
    rows = [json.loads(line) for line in positive.read_text().splitlines()]
    checks['311_processed_rows_with_300_actual_raw_bits'] = (len(rows)==312 and rows[0]['source_ordinal']==298 and
        rows[0]['actual_raw_risk_bit_exact_rows']==300 and rows[0]['actual_raw_after_bit_exact_rows']==300 and
        [r['row'] for r in rows[1:]]==list(range(311)))
    blob = paths.read_bytes()
    def mutated(offset, value, fmt):
        data = bytearray(blob)
        struct.pack_into(fmt, data, offset, value)
        return data
    dt_bits = struct.unpack_from('<I', blob, 20)[0]
    probes = [('wrong_source_ordinal', mutated(8, 299, '<I'), 'processed source ordinal/grid/dt mismatch'),
              ('wrong_model_dt', mutated(20, dt_bits+1, '<I'), 'processed source ordinal/grid/dt mismatch'),
              ('nonfinite_path', mutated(24, float('nan'), '<f'), 'nonfinite processed path'),
              ('truncated_path', blob[:-1], 'truncated dynamic replay'),
              ('trailing_path', blob+b'x', 'trailing processed path input'),
              ('rows_over_budget', mutated(12, 513, '<I'), 'processed source ordinal/grid/dt mismatch'),
              ('wrong_schema', b'x'+blob[1:], 'processed path schema'),
              ('wrong_grid', mutated(16, 29, '<I'), 'processed source ordinal/grid/dt mismatch')]
    for name, data, reason in probes:
        source = args.output/(name+'.bin')
        source.write_bytes(data)
        result = args.output/(name+'.jsonl')
        code = run(args.evaluator, [actual, source, result], name)
        checks[name+'_rejected'] = code==1 and reason in (args.output/(name+'.log')).read_text() and not result.exists()
    before = hashlib.sha256(positive.read_bytes()).hexdigest()
    code = run(args.evaluator, [actual, paths, positive], 'existing_output')
    checks['existing_output_preserved'] = code==1 and hashlib.sha256(positive.read_bytes()).hexdigest()==before
    corrupted = bytearray(actual.read_bytes())
    _, batch, steps, tracks, points = struct.unpack_from('<5I', corrupted, 12)
    risk_offset = 12+20+80+16*points+57*tracks+3*batch*steps*4+batch*4
    risk_bits = struct.unpack_from('<Q', corrupted, risk_offset)[0]
    struct.pack_into('<Q', corrupted, risk_offset, risk_bits+1)
    bad_actual = args.output/'corrupted_actual_risk.bin'
    bad_actual.write_bytes(corrupted)
    result = args.output/'corrupted_actual_risk.jsonl'
    code = run(args.evaluator, [bad_actual, paths, result], 'corrupted_actual_risk')
    checks['corrupted_actual_raw_risk_rejected'] = code==1 and 'actual raw risk/after bits mismatch' in (args.output/'corrupted_actual_risk.log').read_text() and not result.exists()
    report = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'checks': checks,
              'binary_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.evaluator, args.replay]},
              'scope': 'Offline reader/runtime-score arithmetic integrity only; no ROS or physical acceptance.'}
    (args.output/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)
    return 0 if report['verdict']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
