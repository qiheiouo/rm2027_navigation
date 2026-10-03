#!/usr/bin/env python3
"""Prepare bounded offline risk replay from verified consumed fields and bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
from dynamic_consumption_io import BLOCKS, read_consumption, cost_relation_exact


def prepare(source, output, source_ordinals=None):
    paths = sorted(source.glob('score_*.json'), key=lambda p: int(p.stem.split('_')[-1]))
    if not 1 <= len(paths) <= 1000:
        raise ValueError('dynamic replay record budget')
    if source_ordinals is not None:
        if (not source_ordinals or len(source_ordinals) > 1000 or len(set(source_ordinals)) != len(source_ordinals)
                or any(type(n) is not int or not 0 <= n < 1000 for n in source_ordinals)):
            raise ValueError('explicit source ordinal selection')
        paths = [p for p in paths if int(p.stem.split('_')[-1]) in source_ordinals]
        if len(paths) != len(source_ordinals):
            raise ValueError('selected source score missing')
    versions = {read_consumption(path)[0]['schema'] for path in paths}
    if len(versions) != 1:
        raise ValueError('mixed v1/v2 dynamic trace must not be reinterpreted')
    version = versions.pop()
    output.mkdir(parents=True, exist_ok=False)
    identities = []; previous = -1
    with (output/'input.bin').open('xb') as stream:
        stream.write((b'RMDYNRP1' if version == 1 else b'RMDYNRP2')+struct.pack('<I', len(paths)))
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
            if version == 2:
                limits = meta['limits']; prior = limits['geometry_prior_source'].encode()
                stream.write(struct.pack('<IdI', int(limits['obstacle_radius_mode'] == 'nominal_diameter'),
                                         limits['nominal_obstacle_diameter'], len(prior)))
                stream.write(prior)
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
    parser.add_argument('--source-ordinals', type=int, nargs='+')
    args = parser.parse_args(); report = prepare(args.source, args.output, args.source_ordinals)
    print(json.dumps({'records': report['records'], 'input_sha256': report['input_sha256']}))
