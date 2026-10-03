"""Strict private three-stage reader; original raw and auxiliary rows stay distinct."""
import array
import math
from pathlib import Path
import struct
import sys
from native_witness_io import FIELDS


def read_stages(path):
    with Path(path).open('rb') as stream:data=stream.read(995817)
    if len(data)!=995816 or data[:8]!=b'RMCISTO1':raise ValueError('individual stage exact schema/budget')
    count,batch,steps,dt=struct.unpack_from('<IIIf',data,8)
    if (count,batch,steps)!=(1,300,30) or dt!=struct.unpack('<f',struct.pack('<f',.1))[0]:raise ValueError('individual stage fixed grid')
    ordinal,mask=struct.unpack_from('<II',data,24)
    if ordinal!=0 or mask!=63:raise ValueError('individual stage original raw proof required')
    offset=32;stages=[]
    for expected,rows in enumerate([300,311,311]):
        kind,actual_rows=struct.unpack_from('<II',data,offset);offset+=8
        if (kind,actual_rows)!=(expected,rows):raise ValueError('individual stage kind/row mapping')
        blocks={};length=rows*steps
        for name in FIELDS:
            blob=data[offset:offset+length*4];offset+=length*4
            values=array.array('f');values.frombytes(blob)
            if sys.byteorder!='little':values.byteswap()
            if not all(math.isfinite(v) for v in values):raise ValueError('nonfinite individual stage tensor')
            blocks[name]={'values':values,'bytes':blob,'shape':[rows,steps]}
        stages.append({'stage':kind,'rows':rows,'raw_row_offset':0 if kind==0 else 11,'blocks':blocks})
    if offset!=len(data):raise ValueError('individual stage trailing bytes')
    return {'ordinal':ordinal,'raw_exact_mask':mask,'raw_rows':batch,'steps':steps,'model_dt':dt,'stages':stages}
