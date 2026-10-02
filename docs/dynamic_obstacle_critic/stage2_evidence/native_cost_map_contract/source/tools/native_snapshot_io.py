"""Strict independent reader of bounded little-endian native evidence blocks."""
import array
import gzip
import json
import math
import struct
import sys


MATRICES={'x','y','yaw','vx','vy','wz','cvx','cvy','cwz'}
PATHS={'path_x','path_y','path_yaw'}
COST='critic_costs_before_control_regularization'


def read_snapshot(path):
    meta=json.loads(path.read_text());binary=path.with_suffix('.bin')
    if binary.is_file():
        with binary.open('rb') as stream:blob=stream.read(2_000_001)
    else:
        with gzip.open(str(binary)+'.gz','rb') as stream:blob=stream.read(2_000_001)
    if meta.get('schema')!=1 or len(blob)!=meta['payload_bytes'] or len(blob)>2_000_000:
        raise ValueError('snapshot schema or payload byte budget')
    if not math.isfinite(meta['model_dt']) or meta['model_dt']<=0:
        raise ValueError('snapshot model grid')
    if (type(meta['ordinal']) is not int or meta['ordinal']<0 or not math.isfinite(meta['capture_stamp'])
            or type(meta['pose_stamp_sec']) is not int or type(meta['pose_stamp_nanosec']) is not int
            or not 0<=meta['pose_stamp_nanosec']<1_000_000_000):
        raise ValueError('snapshot identity or timestamp')
    if (not math.isfinite(meta['map']['resolution']) or meta['map']['resolution']<=0
            or len(meta['map']['origin'])!=2 or not all(math.isfinite(v) for v in meta['map']['origin'])):
        raise ValueError('snapshot finite map geometry')
    for field in ['pose','speed']:
        if len(meta[field])!={'pose':7,'speed':6}[field] or not all(math.isfinite(v) for v in meta[field]):
            raise ValueError('snapshot finite metadata')
    blocks={};cursor=0
    for block in meta['blocks']:
        name=block['name'];shape=block['shape'];dtype=block['dtype']
        if name in blocks or name not in MATRICES|PATHS|{COST,'raw_costmap'}:
            raise ValueError('snapshot duplicate or unknown block')
        if not shape or any(type(v) is not int or v<0 for v in shape):
            raise ValueError('snapshot invalid shape')
        itemsize=1 if name=='raw_costmap' else 4
        if dtype!=('|u1' if itemsize==1 else '<f4') or block['bytes']!=math.prod(shape)*itemsize or block['offset']!=cursor:
            raise ValueError('snapshot block dtype/size/offset')
        end=cursor+block['bytes']
        if end>len(blob):raise ValueError('snapshot truncated block')
        data=blob[cursor:end];cursor=end
        if itemsize==4:
            values=array.array('f');values.frombytes(data)
            if sys.byteorder!='little':values.byteswap()
            if not all(math.isfinite(v) for v in values):raise ValueError('nonfinite snapshot tensor')
            blocks[name]={'shape':shape,'values':values,'bytes':data}
        else:blocks[name]={'shape':shape,'values':data,'bytes':data}
    if cursor!=len(blob) or set(blocks)!=MATRICES|PATHS|{COST,'raw_costmap'}:
        raise ValueError('snapshot incomplete payload')
    shape=blocks['x']['shape']
    if len(shape)!=2 or not 1<=shape[0]<=512 or not 1<=shape[1]<=64 or any(blocks[n]['shape']!=shape for n in MATRICES):
        raise ValueError('snapshot batch matrix shape')
    if blocks[COST]['shape']!=[shape[0]] or any(len(blocks[n]['shape'])!=1 for n in PATHS):
        raise ValueError('snapshot costs/path shape')
    if blocks['path_x']['shape']!=blocks['path_y']['shape'] or blocks['path_x']['shape']!=blocks['path_yaw']['shape'] or blocks['path_x']['shape'][0]>2000:
        raise ValueError('snapshot path budget')
    nx,ny=meta['map']['size']
    if type(nx) is not int or type(ny) is not int or nx<1 or ny<1 or nx*ny>65536 or blocks['raw_costmap']['shape']!=[ny,nx]:
        raise ValueError('snapshot map shape/budget')
    return meta,blocks


def measured_prefix_matches(meta,blocks):
    batch,steps=blocks['vx']['shape']
    return all(blocks[name]['bytes'][i*steps*4:i*steps*4+4]==struct.pack('<f',meta['speed'][index])
               for name,index in [('vx',0),('vy',1),('wz',5)] for i in range(batch))
