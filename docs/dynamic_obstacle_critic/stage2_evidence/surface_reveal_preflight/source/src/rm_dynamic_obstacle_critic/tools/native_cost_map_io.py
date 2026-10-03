"""Strict private frozen-map SDK query format, unrelated to ROS capture types."""
import math
from pathlib import Path
import struct


def read_input(path):
    with Path(path).open('rb') as stream:data=stream.read(689869)
    if len(data)!=689868 or data[:8]!=b'RMCOSTQ1':raise ValueError('native map exact private schema/budget')
    values=struct.unpack_from('<8If4d',data,8);dt=values[8];resolution,ox,oy,horizon=values[9:]
    if values[:8]!=(298,100,100,4,102,21165,2,30) or dt!=struct.unpack('<f',struct.pack('<f',.1))[0] or resolution!=.05 or horizon!=30*dt or not all(math.isfinite(v) for v in values[8:]):
        raise ValueError('native map fixed source/grid/time')
    fp=list(struct.iter_unpack('<2d',data[76:140]));raw=data[140:10140]
    if not all(math.isfinite(v) for p in fp for v in p):raise ValueError('native map nonfinite footprint')
    traces=[];offset=10140;first_ids=[]
    for i in range(102):
        kind,row,ei,count,start=struct.unpack_from('<4Id',data,offset);offset+=24
        if ei!=i//51 or count!=(208 if ei==0 else 207) or start!=(48.004 if ei==0 else 47.98) or (i%51<8 and (kind,row)!=(0,i%51)) or (i%51>=8 and (kind!=1 or not 0<=row<300)):
            raise ValueError('native map trace row/epoch mapping')
        if ei==0:
            if i%51>8 and row<=first_ids[-1][1]:raise ValueError('native map individual order/duplicate')
            first_ids.append((kind,row))
        elif (kind,row)!=first_ids[i%51]:raise ValueError('native map epoch row mapping')
        poses=list(struct.iter_unpack('<4d',data[offset:offset+count*32]));offset+=count*32
        if len(poses)!=count or not all(math.isfinite(v) for p in poses for v in p) or poses[0][0]!=0 or abs(poses[-1][0]-horizon)>1e-12 or any(b[0]<=a[0] or b[0]>horizon+1e-12 for a,b in zip(poses,poses[1:])):
            raise ValueError('native map complete finite sample grid')
        traces.append({'trace':i,'path_kind':kind,'path_row':row,'epoch_index':ei,'start':start,'poses':poses})
    if offset!=len(data):raise ValueError('native map trailing data')
    return {'dt':dt,'resolution':resolution,'origin':[ox,oy],'horizon':horizon,'size':[100,100],'footprint':fp,'raw':raw,'traces':traces}
