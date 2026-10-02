#!/usr/bin/env python3
"""Prepare explicit SDK processed-path hypotheses and exact consumed context."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
from audit_dynamic_consumption import identity
from dynamic_consumption_io import read_consumption
from native_snapshot_io import read_snapshot
from native_witness_io import records
from prepare_dynamic_consumption_replay import prepare


def verify(root):
    for name, digest in json.loads((root/'manifest.json').read_text())['files'].items():
        path = (root/name).resolve()
        if not path.is_relative_to(root) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('frozen archive mismatch: '+name)


def prepare_paths(source, witness, output):
    verify(source)
    verify(witness)
    provenance = json.loads((witness/'provenance.json').read_text())
    ordinal = provenance['selected_source_ordinal']
    if ordinal != 298 or hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest() != provenance['source_manifest_sha256']:
        raise ValueError('registered consumed source298 required')
    native, native_blocks = read_snapshot(source/'native_cycles/cycle_298.json')
    consumed, consumed_blocks = read_consumption(source/'dynamic_scores/score_298.json')
    if identity(native, native_blocks)[1] != identity(consumed, consumed_blocks)[1]:
        raise ValueError('complete actual native/consumed identity mismatch')
    selected_identity = json.loads((witness/'selected/input_identity.json').read_text())
    if selected_identity['cycles'] != 1 or selected_identity['records'][0]['source_ordinal'] != ordinal:
        raise ValueError('selected witness identity mapping')
    data = list(records(witness/'selected/witnesses.bin.gz'))
    if len(data) != 1:
        raise ValueError('one registered processed batch required')
    record = data[0]
    if (record['row_count'] != 311 or record['steps'] != 30 or record['model_dt'] != native['model_dt']
            or not record['raw_velocity_pose_exact'] or not record['aggregate_SG_exact']
            or not record['actual_command_double_bit_exact']):
        raise ValueError('complete verified native SDK context required')
    for name, speed_axis in [('vx', 0), ('vy', 1), ('wz', 5)]:
        expected = struct.pack('<f', native['speed'][speed_axis])
        if any(struct.pack('<f', record['blocks'][name][row*30]) != expected for row in range(311)):
            raise ValueError('processed measured prefix replaced')
    output.mkdir()
    actual_identity = prepare(source/'dynamic_scores', output/'actual_input', [ordinal])
    path = output/'processed_paths.bin'
    with path.open('xb') as stream:
        stream.write(b'RMDYNPX1'+struct.pack('<IIIf', ordinal, 311, 30, record['model_dt']))
        for name in ['x', 'y', 'yaw']:
            for value in record['blocks'][name]:
                stream.write(struct.pack('<f', value))
    names = selected_identity['proposal_names']+[f'sampler_{row}_bounds_SG_counterfactual' for row in range(300)]
    if len(names) != 311:
        raise ValueError('complete processed proposal names')
    report = {'schema': 1, 'scope': 'Processed SDK hypotheses; original actual aggregate row0, remaining rows counterfactual.',
              'source_manifest_sha256': provenance['source_manifest_sha256'],
              'witness_manifest_sha256': hashlib.sha256((witness/'manifest.json').read_bytes()).hexdigest(),
              'source_native_ordinal': ordinal, 'score_clock': consumed['score_stamp'],
              'source_pose_clock': native['pose_stamp_sec']+native['pose_stamp_nanosec']*1e-9,
              'source_age_used': consumed['source_age_used'], 'model_dt': record['model_dt'],
              'grid': [311, 30], 'actual_input_sha256': actual_identity['input_sha256'],
              'processed_paths_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'witness_binary_sha256': hashlib.sha256((witness/'selected/witnesses.bin.gz').read_bytes()).hexdigest(),
              'actual_measured_prefix_exact': True,
              'rows': [{'row': row, 'name': name, 'kind': 'actual_aggregate_SG' if row==0 else 'counterfactual_bounds_SG'} for row, name in enumerate(names)]}
    (output/'input_identity.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source', 'witness_archive', 'output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    report = prepare_paths(args.source.resolve(), args.witness_archive.resolve(), args.output)
    print(json.dumps({k: v for k, v in report.items() if k!='rows'}), flush=True)
