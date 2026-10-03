"""Bounded reader of private fields actually consumed by DynamicObstacleCritic.

This file schema is evidence, not another ROS tracker interface. Future arrays,
processing stamps, z dimensions and covariance are deliberately absent.
"""
import array
import gzip
import json
import math
import struct
import sys

BLOCKS = ('x', 'y', 'yaw', 'costs_before_dynamic', 'dynamic_risk_double', 'costs_after_dynamic')
V1_SCHEMA = 'rm_dynamic_obstacle_predictions/v1'
V2_SCHEMA = 'rm_dynamic_obstacle_predictions/v2_observation_anchor'
V2_FIELDS = {'schema', 'kind', 'ordinal', 'score_stamp', 'source_age_used',
    'pose_stamp_sec', 'pose_stamp_nanosec', 'pose_frame', 'evaluation_frame', 'base_frame',
    'model_dt', 'pose', 'speed', 'world_transform', 'padded_footprint', 'limits',
    'cost_parameters', 'input_used', 'blocks', 'payload_bytes', 'scope', 'unavailable'}
LIMIT_FIELDS = {'minimum_radius', 'max_age', 'max_observation_age', 'max_speed',
    'max_extent', 'max_tracks', 'jump_tolerance', 'max_tf_age', 'input_frame'}
V2_RADIUS_FIELDS = {'input_schema', 'obstacle_radius_mode', 'nominal_obstacle_diameter', 'geometry_prior_source'}
MESSAGE_FIELDS = {'stamp_sec', 'stamp_nanosec', 'frame', 'schema', 'authority', 'complete',
                  'total_track_count', 'prediction_dt', 'prediction_steps', 'tracks'}
TRACK_FIELDS = {'id', 'state', 'xy', 'vxy', 'size_xy', 'observed_sec', 'observed_nanosec'}


def exact_fields(value, fields, label):
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('dynamic v2 '+label+' fields')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate evidence JSON key')
        result[key] = value
    return result


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def vector(value, length):
    return isinstance(value, list) and len(value) == length and all(finite(v) for v in value)


def integer(value, minimum, maximum):
    return type(value) is int and minimum <= value <= maximum


def stamp(obj, sec, ns):
    if not integer(obj[sec], -2**31, 2**31-1) or not integer(obj[ns], 0, 999_999_999):
        raise ValueError('invalid evidence timestamp')
    return obj[sec] + obj[ns] * 1e-9


def read_consumption(path):
    with path.open('rb') as stream:
        raw = stream.read(200_001)
    if len(raw) > 200_000:
        raise ValueError('dynamic metadata budget')
    meta = json.loads(raw, object_pairs_hook=unique_object)
    version = meta.get('schema')
    if type(version) is not int or version not in (1, 2) or meta.get('kind') != 'dynamic_consumption_fields_v'+str(version):
        raise ValueError('dynamic evidence schema')
    if version == 2:
        exact_fields(meta, V2_FIELDS, 'metadata')
        exact_fields(meta['limits'], LIMIT_FIELDS | V2_RADIUS_FIELDS, 'limits')
        exact_fields(meta['input_used'], MESSAGE_FIELDS, 'message')
        exact_fields(meta['cost_parameters'], {'weight', 'influence_distance', 'safety_margin', 'collision_cost'}, 'cost')
    if not integer(meta['ordinal'], 0, 999) or not finite(meta['score_stamp']) or meta['score_stamp'] <= 0:
        raise ValueError('dynamic score identity')
    stamp(meta, 'pose_stamp_sec', 'pose_stamp_nanosec')
    for name in ('pose_frame', 'evaluation_frame', 'base_frame'):
        if not isinstance(meta[name], str) or not meta[name] or len(meta[name]) > 4096:
            raise ValueError('dynamic frame identity')
    for name, length in (('pose', 7), ('speed', 6), ('world_transform', 3)):
        if not vector(meta[name], length):
            raise ValueError('nonfinite dynamic metadata')
    poly = meta['padded_footprint']
    if not isinstance(poly, list) or not 3 <= len(poly) <= 128 or not all(vector(p, 2) for p in poly):
        raise ValueError('dynamic footprint budget/geometry')
    lim = meta['limits']; params = meta['cost_parameters']
    for name in ('minimum_radius', 'max_age', 'max_observation_age', 'max_speed', 'max_extent', 'max_tf_age'):
        if not finite(lim[name]) or lim[name] <= 0:
            raise ValueError('dynamic positive limits')
    if not integer(lim['max_tracks'], 1, 256) or not finite(lim['jump_tolerance']) or lim['jump_tolerance'] < 0:
        raise ValueError('dynamic limits')
    if not isinstance(lim['input_frame'], str) or not lim['input_frame'] or len(lim['input_frame']) > 4096:
        raise ValueError('dynamic input frame')
    selected_schema = V1_SCHEMA
    if version == 2:
        selected_schema = lim['input_schema']
        mode, diameter, prior = (lim[n] for n in ('obstacle_radius_mode', 'nominal_obstacle_diameter', 'geometry_prior_source'))
        if (selected_schema != V2_SCHEMA or not finite(diameter) or not isinstance(prior, str)
                or len(prior.encode()) > 4096):
            raise ValueError('dynamic v2 radius model')
        if mode == 'visible_extent':
            if diameter != 0 or prior != '': raise ValueError('dynamic v2 unused nominal settings')
        elif mode == 'nominal_diameter':
            if not 0 < diameter <= 3 or not prior.strip(' \t\r\n\f\v'):
                raise ValueError('dynamic v2 nominal diameter/provenance')
        else:
            raise ValueError('dynamic v2 unknown radius mode')
    if not all(finite(params[n]) for n in ('weight', 'influence_distance', 'safety_margin', 'collision_cost')):
        raise ValueError('dynamic cost parameters')
    if params['weight'] < 0 or params['safety_margin'] <= 0 or params['influence_distance'] <= params['safety_margin'] or params['collision_cost'] <= 0:
        raise ValueError('dynamic cost parameter range')
    msg = meta['input_used']; tracks = msg['tracks']
    source = stamp(msg, 'stamp_sec', 'stamp_nanosec'); age = meta['score_stamp'] - source
    if (not finite(meta['source_age_used']) or struct.pack('<d', age) != struct.pack('<d', meta['source_age_used'])
            or source <= 0 or not 0 <= age <= lim['max_age']):
        raise ValueError('dynamic source age identity/range')
    if (msg['schema'] != selected_schema or msg['authority'] != 'shadow_only'
            or msg['frame'] != lim['input_frame'] or msg['complete'] is not True):
        raise ValueError('dynamic consumed input contract')
    if (not isinstance(tracks, list) or not integer(msg['total_track_count'], 0, min(64, lim['max_tracks']))
            or msg['total_track_count'] != len(tracks)):
        raise ValueError('dynamic consumed track budget')
    if not finite(meta['model_dt']) or meta['model_dt'] <= 0 or not finite(msg['prediction_dt']) or abs(msg['prediction_dt'] - meta['model_dt']) > 1e-6:
        raise ValueError('dynamic model grid')
    if not integer(msg['prediction_steps'], 1, 64):
        raise ValueError('dynamic step budget')
    identifiers = set()
    for track in tracks:
        if version == 2: exact_fields(track, TRACK_FIELDS, 'track')
        if not integer(track['id'], 0, 2**64-1) or track['id'] in identifiers or track['state'] not in (1, 2, 3) or type(track['state']) is not int:
            raise ValueError('dynamic track identity/state')
        identifiers.add(track['id'])
        if not all(vector(track[n], 2) for n in ('xy', 'vxy', 'size_xy')):
            raise ValueError('nonfinite consumed track')
        if any(not 0 < v <= lim['max_extent'] for v in track['size_xy']) or math.hypot(*track['vxy']) > lim['max_speed']:
            raise ValueError('dynamic track size/speed')
        observed = stamp(track, 'observed_sec', 'observed_nanosec')
        if observed <= 0 or observed > source or meta['score_stamp'] - observed > lim['max_observation_age']:
            raise ValueError('dynamic track observation age')
        if version == 2:
            source_ns = msg['stamp_sec']*10**9+msg['stamp_nanosec']
            observed_ns = track['observed_sec']*10**9+track['observed_nanosec']
            if (observed_ns >= source_ns if track['state'] == 3 else observed_ns != source_ns):
                raise ValueError('dynamic v2 observation epoch')
    binary = path.with_suffix('.bin')
    opener = open if binary.is_file() else gzip.open
    with opener(binary if binary.is_file() else str(binary)+'.gz', 'rb') as stream:
        blob = stream.read(2_000_001)
    if not integer(meta['payload_bytes'], 1, 2_000_000) or len(blob) != meta['payload_bytes']:
        raise ValueError('dynamic payload byte budget/length')
    blocks = {}; cursor = 0
    if not isinstance(meta['blocks'], list) or len(meta['blocks']) != len(BLOCKS):
        raise ValueError('dynamic block count')
    for expected, block in zip(BLOCKS, meta['blocks']):
        if version == 2: exact_fields(block, {'name', 'dtype', 'offset', 'bytes', 'shape'}, 'block')
        name = block['name']; dtype = '<f8' if name == 'dynamic_risk_double' else '<f4'
        shape = block['shape']; width = 8 if dtype == '<f8' else 4
        if name != expected or not isinstance(shape, list) or not shape or any(not integer(v, 1, 512) for v in shape):
            raise ValueError('dynamic block order/shape')
        length = math.prod(shape)*width
        if block['dtype'] != dtype or type(block['offset']) is not int or block['offset'] != cursor or type(block['bytes']) is not int or block['bytes'] != length or cursor+length > len(blob):
            raise ValueError('dynamic block dtype/offset/length')
        data = blob[cursor:cursor+length]; cursor += length
        values = array.array('d' if width == 8 else 'f'); values.frombytes(data)
        if sys.byteorder != 'little': values.byteswap()
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite dynamic payload')
        blocks[name] = {'shape': shape, 'bytes': data, 'values': values}
    grid = blocks['x']['shape']
    if (cursor != len(blob) or len(grid) != 2 or not 1 <= grid[0] <= 512 or not 1 <= grid[1] <= 64
            or grid[1] != msg['prediction_steps'] or any(blocks[n]['shape'] != grid for n in BLOCKS[:3])
            or any(blocks[n]['shape'] != [grid[0]] for n in BLOCKS[3:])):
        raise ValueError('dynamic complete batch grid')
    if any(v < 0 for v in blocks['dynamic_risk_double']['values']):
        raise ValueError('negative dynamic risk')
    return meta, blocks


def cost_relation_exact(blocks):
    values = zip(blocks['costs_before_dynamic']['values'], blocks['dynamic_risk_double']['values'])
    return b''.join(struct.pack('<f', a+b) for a, b in values) == blocks['costs_after_dynamic']['bytes']
