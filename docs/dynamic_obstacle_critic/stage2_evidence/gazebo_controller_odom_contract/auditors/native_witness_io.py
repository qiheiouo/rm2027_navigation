"""Strict bounded reader for offline native full-grid witness outputs."""
import array
import gzip
import math
import struct
import sys

FIELDS=('cvx','cvy','cwz','vx','vy','wz','x','y','yaw')


def records(path):
    stream=gzip.open(path,'rb') if str(path).endswith('.gz') else path.open('rb')
    with stream:
        header=stream.read(24)
        if len(header)!=24 or header[:8]!=b'RMSFWTO1':raise ValueError('witness header/schema')
        count,rows,steps,dt=struct.unpack_from('<IIIf',header,8)
        if not 1<=count<=1000 or not 11<=rows<=512 or steps!=30 or not math.isfinite(dt) or abs(dt-.1)>1e-7:
            raise ValueError('witness full-grid budget')
        size=24+9*rows*steps*4
        for expected in range(count):
            blob=stream.read(size)
            if len(blob)!=size:raise ValueError('truncated witness record')
            ordinal,mask,sg,command,yaw=struct.unpack_from('<IIIId',blob)
            if ordinal!=expected or mask>63 or sg not in (0,1) or command not in (0,1) or not math.isfinite(yaw):
                raise ValueError('witness identity or evidence flag')
            values=array.array('f');values.frombytes(blob[24:])
            if sys.byteorder!='little':values.byteswap()
            if not all(math.isfinite(v) for v in values):raise ValueError('nonfinite native witness tensor')
            length=rows*steps
            yield {'ordinal':ordinal,'row_count':rows,'steps':steps,'model_dt':dt,'initial_yaw':yaw,
                   'raw_velocity_pose_exact':mask==63,'aggregate_SG_exact':bool(sg),'actual_command_double_bit_exact':bool(command),
                   'blocks':{name:values[index*length:(index+1)*length] for index,name in enumerate(FIELDS)}}
        if stream.read(1):raise ValueError('witness trailing bytes')
