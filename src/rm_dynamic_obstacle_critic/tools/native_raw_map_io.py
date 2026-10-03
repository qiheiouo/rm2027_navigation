"""Strict original raw30/dense private query reader, isolated from old stages."""
import math
from pathlib import Path
import struct


def read_input(path):
    with Path(path).open('rb') as stream:data=stream.read(4598941)
    if len(data)!=4598940 or data[:8]!=b'RMCOSTR1':raise ValueError('native raw map exact private schema/budget')
    values=struct.unpack_from('<8If4d',data,8);dt=values[8];resolution,ox,oy,horizon=values[9:]
    if values[:8]!=(298,100,100,4,1200,142500,2,30) or dt!=struct.unpack('<f',struct.pack('<f',.1))[0] or resolution!=.05 or horizon!=30*dt or not all(math.isfinite(v) for v in values[8:]):raise ValueError('native raw map fixed source/grid/time')
    fp=list(struct.iter_unpack('<2d',data[76:140]));raw=data[140:10140]
    if not all(math.isfinite(v) for p in fp for v in p):raise ValueError('native raw map nonfinite footprint')
    traces=[];offset=10140
    for i in range(1200):
        kind,row,ei,count,start=struct.unpack_from('<4Id',data,offset);offset+=24;index=i%600;epoch=i//600
        if kind!=(2 if index<300 else 3) or row!=index%300 or ei!=epoch or count!=((208 if epoch==0 else 207) if kind==2 else 30) or start!=(48.004 if epoch==0 else 47.98):raise ValueError('native raw map trace row/epoch mapping')
        poses=list(struct.iter_unpack('<4d',data[offset:offset+count*32]));offset+=count*32
        if len(poses)!=count or not all(math.isfinite(v) for p in poses for v in p) or poses[0][0]!=(0 if kind==2 else dt) or abs(poses[-1][0]-horizon)>1e-12 or any(b[0]<=a[0] or b[0]>horizon+1e-12 for a,b in zip(poses,poses[1:])):raise ValueError('native raw map complete finite sample grid')
        if kind==3 and any(p[0]!=(k+1)*dt for k,p in enumerate(poses)):raise ValueError('native raw map exact original30 time grid')
        traces.append({'trace':i,'path_kind':kind,'path_row':row,'epoch_index':ei,'start':start,'poses':poses})
    if offset!=len(data):raise ValueError('native raw map trailing data')
    return {'dt':dt,'resolution':resolution,'origin':[ox,oy],'horizon':horizon,'size':[100,100],'footprint':fp,'raw':raw,'traces':traces}
