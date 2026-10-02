#!/usr/bin/env python3
"""Prepare bounded offline risk replay from verified consumed fields and bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
from dynamic_consumption_io import BLOCKS, read_consumption, cost_relation_exact


def prepare(source, output):
    paths = sorted(source.glob('score_*.json'), key=lambda p: int(p.stem.split('_')[-1]))
    if not 1 <= len(paths) <= 1000:
        raise ValueError('dynamic replay record budget')
    output.mkdir(parents=True, exist_ok=False)
    identities = []; previous = -1
    with (output/'input.bin').open('xb') as stream:
        stream.write(b'RMDYNRP1'+struct.pack('<I', len(paths)))
        for path in paths:
            meta, blocks = read_consumption(path); ordinal = meta['ordinal']
            if ordinal != int(path.stem.split('_')[-1]) or ordinal <= previous or not cost_relation_exact(blocks):
                raise ValueError('dynamic score ordering or float cost relation')
            previous = ordinal
            batch, steps = blocks['x']['shape']; tracks = meta['input_used']['tracks']; poly = meta['padded_footprint']
            stream.write(struct.pack('<5I', ordinal, batch, steps, len(tracks), len(poly)))
            params = meta['cost_parameters']
            stream.write(struct.pack('<10d', meta['model_dt'], meta['source_age_used'], *meta['world_transform'],
                meta['limits']['minimum_radius'], params['weight'], params['influence_distance'], params['safety_margin'], params['collision_cost']))
            for point in poly: stream.write(struct.pack('<2d', *point))
            for track in tracks:
                stream.write(struct.pack('<QB6d', track['id'], track['state'], *track['xy'], *track['vxy'], *track['size_xy']))
            for name in BLOCKS: stream.write(blocks[name]['bytes'])
            identities.append({'ordinal': ordinal, 'metadata_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'payload_sha256': hashlib.sha256(b''.join(blocks[n]['bytes'] for n in BLOCKS)).hexdigest()})
    report = {'schema': 1, 'records': len(paths), 'input_sha256': hashlib.sha256((output/'input.bin').read_bytes()).hexdigest(),
        'scope': 'Exact consumed score inputs; offline helper does not publish ROS, TF or commands.', 'identities': identities}
    (output/'input_identity.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('source', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); report = prepare(args.source, args.output)
    print(json.dumps({'records': report['records'], 'input_sha256': report['input_sha256']}))
