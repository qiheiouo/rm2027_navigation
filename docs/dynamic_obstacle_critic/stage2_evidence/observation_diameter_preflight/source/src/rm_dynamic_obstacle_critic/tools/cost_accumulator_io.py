"""Strict reader and byte association for the separate CostCritic observer.

The observer token is local instrumentation state, not an optimizer cycle id.
Costs are excluded from input identity because their change is the observation.
"""
import array
import gzip
import json
import math
from pathlib import Path
import struct
import sys

MATRICES = ('x', 'y', 'yaw', 'cvx', 'cvy', 'cwz', 'vx', 'vy', 'wz')
PATHS = ('path_x', 'path_y', 'path_yaw')
COST = 'cumulative_critic_costs'
UNAVAILABLE = ['unrounded_individual_increment', 'inflation_cache',
    'actual_CostCritic_branch_and_query_selection', 'other_public_CriticData_cache_fields',
    'physical_cell_source']


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result: raise ValueError('duplicate cost observation JSON key')
        result[key] = value
    return result


def finite(value):
    if type(value) not in (int, float): return False
    try: return math.isfinite(value)
    except OverflowError: return False


def vector(value, length):
    return isinstance(value, list) and len(value) == length and all(finite(v) for v in value)


def read_observation(path):
    path = Path(path)
    with path.open('rb') as stream: raw = stream.read(262145)
    if len(raw) > 262144: raise ValueError('cost observation metadata budget')
    meta = json.loads(raw, object_pairs_hook=unique_object,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
    fields = {'schema', 'ordinal', 'stage', 'capture_stamp', 'paired_before_ordinal',
        'pose_stamp_sec', 'pose_stamp_nanosec', 'pose_frame', 'evaluation_frame', 'base_frame',
        'model_dt', 'fail_flag', 'pose', 'speed', 'map', 'map_snapshot_mutex_locked',
        'padded_footprint', 'blocks', 'payload_bytes', 'unavailable'}
    if not isinstance(meta, dict) or set(meta) != fields or meta['schema'] != 'native_cost_accumulator_observation_v1':
        raise ValueError('cost observation schema/fields')
    ordinal = meta['ordinal']; token = meta['paired_before_ordinal']
    if (type(ordinal) is not int or not 0 <= ordinal < 1000 or
            meta['stage'] not in ('before_CostCritic', 'after_CostCritic') or
            (token is not None and (type(token) is not int or not 0 <= token < 1000)) or
            (meta['stage'] == 'before_CostCritic' and token != ordinal) or
            not finite(meta['capture_stamp']) or not finite(meta['model_dt']) or meta['model_dt'] <= 0 or
            type(meta['pose_stamp_sec']) is not int or not -(2**31) <= meta['pose_stamp_sec'] < 2**31 or
            type(meta['pose_stamp_nanosec']) is not int or not 0 <= meta['pose_stamp_nanosec'] < 10**9 or
            type(meta['fail_flag']) is not bool or type(meta['map_snapshot_mutex_locked']) is not bool):
        raise ValueError('cost observation identity/grid/timestamp/flags')
    if (any(type(meta[n]) is not str or not meta[n] for n in ('pose_frame', 'evaluation_frame', 'base_frame')) or
            meta['pose_frame'] != meta['evaluation_frame'] or not vector(meta['pose'], 7) or
            not vector(meta['speed'], 6) or meta['unavailable'] != UNAVAILABLE):
        raise ValueError('cost observation frame/metadata/scope')
    footprint = meta['padded_footprint']; grid = meta['map']
    if not isinstance(footprint, list) or not 3 <= len(footprint) <= 64 or not all(vector(p, 2) for p in footprint):
        raise ValueError('cost observation footprint')
    if (not isinstance(grid, dict) or set(grid) != {'size', 'origin', 'resolution'} or
            not isinstance(grid['size'], list) or len(grid['size']) != 2 or
            any(type(v) is not int or not 1 <= v <= 65536 for v in grid['size']) or
            math.prod(grid['size']) > 65536 or not vector(grid['origin'], 2) or
            not finite(grid['resolution']) or grid['resolution'] <= 0):
        raise ValueError('cost observation map budget/geometry')
    size = meta['payload_bytes']
    if type(size) is not int or not 1 <= size <= 2_000_000: raise ValueError('cost observation payload budget')
    binary = path.with_suffix('.bin'); compressed = Path(str(binary)+'.gz')
    if binary.is_file() and compressed.is_file(): raise ValueError('ambiguous cost observation payload')
    opener = binary.open if binary.is_file() else lambda mode: gzip.open(compressed, mode)
    with opener('rb') as stream: blob = stream.read(2_000_001)
    if len(blob) != size: raise ValueError('cost observation payload size')
    descriptors = meta['blocks']
    if not isinstance(descriptors, list) or len(descriptors) != 14: raise ValueError('cost observation block count')
    blocks = {}; cursor = 0
    for block in descriptors:
        if not isinstance(block, dict) or set(block) != {'name', 'dtype', 'offset', 'bytes', 'shape'}:
            raise ValueError('cost observation block fields')
        name = block['name']; shape = block['shape']; itemsize = 1 if name == 'raw_costmap' else 4
        if (type(name) is not str or name in blocks or name not in set(MATRICES+PATHS+(COST, 'raw_costmap')) or
                not isinstance(shape, list) or not 1 <= len(shape) <= 2 or
                any(type(v) is not int or not 0 <= v <= 65536 for v in shape) or
                type(block['offset']) is not int or block['offset'] != cursor or
                type(block['bytes']) is not int or block['bytes'] != math.prod(shape)*itemsize or
                block['dtype'] != ('|u1' if itemsize == 1 else '<f4')):
            raise ValueError('cost observation duplicate/name/shape/dtype/offset')
        end = cursor+block['bytes']
        if end > len(blob): raise ValueError('cost observation truncated block')
        data = blob[cursor:end]; cursor = end
        values = data
        if itemsize == 4:
            values = array.array('f'); values.frombytes(data)
            if sys.byteorder != 'little': values.byteswap()
            if not all(math.isfinite(v) for v in values): raise ValueError('nonfinite cost observation tensor')
        blocks[name] = {'shape': shape, 'values': values, 'bytes': data}
    if cursor != size: raise ValueError('cost observation trailing payload')
    shape = blocks['x']['shape']
    if (len(shape) != 2 or not 1 <= shape[0] <= 512 or not 1 <= shape[1] <= 64 or
            any(blocks[n]['shape'] != shape for n in MATRICES) or blocks[COST]['shape'] != [shape[0]] or
            any(blocks[n]['shape'] != blocks['path_x']['shape'] for n in PATHS) or
            len(blocks['path_x']['shape']) != 1 or blocks['path_x']['shape'][0] > 2000 or
            blocks['raw_costmap']['shape'] != grid['size'][::-1]):
        raise ValueError('cost observation tensor/map shape association')
    return meta, blocks


def context_bytes(meta):
    """Shared public context identity, preserving IEEE signed zero in doubles."""
    frames = json.dumps([meta[n] for n in ('pose_frame', 'evaluation_frame', 'base_frame')],
        ensure_ascii=False, separators=(',', ':')).encode()
    return (struct.pack('<14d', *meta['pose'], *meta['speed'], meta['model_dt']) +
        struct.pack('<iI', meta['pose_stamp_sec'], meta['pose_stamp_nanosec']) + frames +
        struct.pack('<I', len(meta['padded_footprint'])) +
        b''.join(struct.pack('<2d', *p) for p in meta['padded_footprint']))


def input_bytes(meta, blocks, include_map=True):
    result = context_bytes(meta)
    for name in MATRICES+PATHS:
        shape = blocks[name]['shape']
        result += struct.pack('<I', len(shape))+struct.pack('<'+'I'*len(shape), *shape)+blocks[name]['bytes']
    if include_map:
        result += struct.pack('<2I3d', *meta['map']['size'], *meta['map']['origin'], meta['map']['resolution'])
        result += blocks['raw_costmap']['bytes']
    return result


def pair_unknown_reason(before, after):
    bmeta, bblocks = before; ameta, ablocks = after
    if bmeta['stage'] != 'before_CostCritic' or ameta['stage'] != 'after_CostCritic': return 'stage mismatch'
    if ameta['paired_before_ordinal'] != bmeta['ordinal']: return 'no matching committed before token'
    if bmeta['fail_flag'] or ameta['fail_flag']: return 'failed batch or SDK short circuit'
    if not bmeta['map_snapshot_mutex_locked'] or not ameta['map_snapshot_mutex_locked']: return 'map snapshot lock unavailable'
    if ameta['capture_stamp'] < bmeta['capture_stamp']: return 'capture clock order'
    if input_bytes(bmeta, bblocks) != input_bytes(ameta, ablocks): return 'public input or map bytes differ'
    return None
