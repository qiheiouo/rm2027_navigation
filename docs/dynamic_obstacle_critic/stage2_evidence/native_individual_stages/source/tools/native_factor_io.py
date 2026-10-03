"""Strict private four-row factor output; never an actual ROS capture type."""
import array
import math
import struct
import sys

FIELDS=('cvx','cvy','cwz','vx','vy','wz','x','y','yaw')


def read_paths(path):
    with path.open('rb') as stream:data=stream.read(4357)
    if len(data)!=4356 or data[:8]!=b'RMCVAGO1':raise ValueError('native factor output exact schema/budget')
    source,rows,steps,mask,dt,yaw=struct.unpack_from('<IIIIfd',data,8)
    if (source,rows,steps,mask)!=(298,4,30,127) or dt!=struct.unpack('<f',struct.pack('<f',.1))[0] or not math.isfinite(yaw):
        raise ValueError('native factor output source/grid/baseline proof')
    values=array.array('f');values.frombytes(data[36:])
    if sys.byteorder!='little':values.byteswap()
    if not all(math.isfinite(v) for v in values):raise ValueError('native factor nonfinite output')
    return {'source_ordinal':source,'rows':rows,'steps':steps,'model_dt':dt,'initial_yaw':yaw,
            'actual_baseline_bit_exact_mask':mask,
            'blocks':{name:{'values':values[i*120:(i+1)*120],'shape':[4,30]} for i,name in enumerate(FIELDS)}}
