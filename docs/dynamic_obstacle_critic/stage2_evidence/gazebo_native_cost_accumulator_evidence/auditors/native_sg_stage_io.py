"""Strict private eight-path stage output; not a ROS or four-path capture."""
import array
import math
import struct
import sys
from native_factor_io import FIELDS

FACTORS=('actual_baseline','source_geometry_oracle','source_bracket_CV_oracle','future_circle_oracle')
STAGES=('bounded_mean_before_SG','actual_history_after_SG')


def read_paths(path):
    with path.open('rb') as stream:data=stream.read(8677)
    if len(data)!=8676 or data[:8]!=b'RMCVAST1':raise ValueError('native stage exact schema/budget')
    source,rows,steps,mask,dt,yaw=struct.unpack_from('<IIIIfd',data,8)
    if (source,rows,steps,mask)!=(298,8,30,127) or dt!=struct.unpack('<f',struct.pack('<f',.1))[0] or not math.isfinite(yaw):
        raise ValueError('native stage source/grid/baseline proof')
    values=array.array('f');values.frombytes(data[36:])
    if sys.byteorder!='little':values.byteswap()
    if not all(math.isfinite(v) for v in values):raise ValueError('native stage nonfinite output')
    return {'source_ordinal':source,'rows':rows,'steps':steps,'model_dt':dt,'initial_yaw':yaw,
            'actual_baseline_bit_exact_mask':mask,'row_identity':[(f,s) for f in FACTORS for s in STAGES],
            'blocks':{name:{'values':values[i*240:(i+1)*240],'shape':[8,30]} for i,name in enumerate(FIELDS)}}
