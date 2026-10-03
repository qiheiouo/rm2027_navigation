#!/usr/bin/env python3
"""Independent center/grid-line scalar checks of installed footprint queries."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
from native_cost_map_io import read_input


def raster_line(a,b):
    """Integer midpoint rasterization, including endpoints and >= ties."""
    x,y=a;dx,dy=abs(b[0]-x),abs(b[1]-y);sx=1 if b[0]>=x else -1;sy=1 if b[1]>=y else -1
    major=max(dx,dy);minor=min(dx,dy);error=major//2
    for _ in range(major+1):
        yield x,y
        error+=minor
        if error>=major:
            error-=major
            if dx>=dy:y+=sy
            else:x+=sx
        if dx>=dy:x+=sx
        else:y+=sy


def scalar_cost(context,pose):
    x,y,yaw=pose;res=context['resolution'];ox,oy=context['origin'];width,height=context['size'];raw=context['raw']
    def cell(px,py):
        if px<ox or py<oy:return None
        xx,yy=int((px-ox)/res),int((py-oy)/res)
        return (xx,yy) if xx<width and yy<height else None
    middle=cell(x,y);center=raw[middle[1]*width+middle[0]] if middle else -1
    c,s=math.cos(yaw),math.sin(yaw);points=[cell(x+c*px-s*py,y+s*px+c*py) for px,py in context['footprint']]
    if any(p is None for p in points):return center,254
    result=0
    for a,b in zip(points,points[1:]+points[:1]):
        edge=0
        for xx,yy in raster_line(a,b):
            value=raw[yy*width+xx]
            if value==254:return center,254
            edge=max(edge,value)
        result=max(result,edge)
    return center,result


def verify(prepared,cpp):
    context=read_input(prepared/'queries.bin');identity=json.loads((prepared/'input_identity.json').read_text())
    if identity['schema']!='native_map_query_input_v1' or identity['source_ordinal']!=298 or identity['trace_count']!=102 or identity['query_count']!=21165 or identity['model_dt']!=context['dt'] or identity['horizon']!=context['horizon'] or identity['map']['size']!=context['size'] or identity['map']['resolution']!=context['resolution'] or identity['map']['origin']!=context['origin'] or identity['padded_footprint']!=[list(p) for p in context['footprint']]:
        raise ValueError('private query identity metadata changed')
    if hashlib.sha256((prepared/'queries.bin').read_bytes()).hexdigest()!=identity['input_sha256'] or hashlib.sha256(context['raw']).hexdigest()!=identity['map_raw_bytes_sha256']:
        raise ValueError('original frozen query/map identities required')
    data=[json.loads(l) for l in cpp.read_text().splitlines()]
    if len(data)!=21166 or data[0]['schema']!='native_map_queries_v1' or data[0]['source_ordinal']!=298 or data[0]['trace_count']!=102 or data[0]['query_count']!=21165 or data[0]['actual_CostCritic_state']!='unknown' or data[0]['native_library']!='/opt/ros/humble/lib/libnav2_costmap_2d_core.so':
        raise ValueError('complete installed native checker output required')
    offset=1;traces=[];errors=[]
    for trace in context['traces']:
        rows=data[offset:offset+len(trace['poses'])];offset+=len(rows)
        for sample,(p,row) in enumerate(zip(trace['poses'],rows)):
            if any(row[key]!=trace[key] for key in ['trace','path_kind','path_row','epoch_index']) or row['sample']!=sample or struct.pack('<4d',row['future_seconds'],*row['pose'])!=struct.pack('<4d',*p):
                raise ValueError('native query private pose/row/time join changed')
            center,fp=scalar_cost(context,p[1:])
            if type(row['center_in_map']) is not bool or type(row['center_raw']) is not int or isinstance(row['native_footprint_cost'],bool) or row['center_in_map']!=(center>=0) or row['center_raw']!=center or row['native_footprint_cost']!=fp:
                errors.append({'trace':trace['trace'],'sample':sample,'native_center':row['center_raw'],'scalar_center':center,'native_footprint':row['native_footprint_cost'],'scalar_footprint':fp})
        traces.append({'trace':trace['trace'],'path_kind':trace['path_kind'],'path_row':trace['path_row'],'epoch_index':trace['epoch_index'],
            'samples':len(rows),'maximum_center_raw':max(r['center_raw'] for r in rows),
            'maximum_native_footprint_cost':max(r['native_footprint_cost'] for r in rows),
            'native_footprint_samples_ge203':sum(r['native_footprint_cost']>=203 for r in rows),
            'native_footprint_samples_eq254':sum(r['native_footprint_cost']==254 for r in rows),
            'first_native_footprint_ge203':next((r for r in rows if r['native_footprint_cost']>=203),None)})
    return {'verdict':'PASS' if not errors else 'FAILED','query_count':21165,'trace_count':102,'scalar_mismatch_count':len(errors),
        'scalar_mismatches':errors,'traces':traces,'actual_CostCritic_full_state':'unknown',
        'scope':'Exact independent center/raw and midpoint boundary-grid costs, not whole polygon/interpolation-reserve or actual CostCritic classification.'}


def verify_synthetic(path):
    data=[json.loads(line) for line in path.read_text().splitlines()]
    if len(data)!=17 or data[0]['schema']!='native_map_synthetic_v1' or data[0]['native_library']!='/opt/ros/humble/lib/libnav2_costmap_2d_core.so':
        raise ValueError('complete installed synthetic checker output required')
    checks={};index=1
    for placement in ['edge','interior','center']:
        for value in [203,216,253,254,255]:
            expected={'placement':placement,'cell_value':value,'center_raw':value if placement=='center' else 0,'native_footprint_cost':value if placement=='edge' else 0}
            checks[f'{placement}_{value}']=data[index]==expected;index+=1
    checks['outside_map']=data[-1]=={'placement':'outside_map','native_footprint_cost':254}
    return {'verdict':'PASS' if all(checks.values()) else 'FAILED','checks':checks,
        'scope':'Artificial uninflated maps: boundary-only helper and separate center query, not actual CostCritic selection/classification.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['prepared','cpp','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite native map verification')
    result=verify(args.prepared,args.cpp)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'verdict':result['verdict'],'queries':result['query_count'],'mismatches':result['scalar_mismatch_count']}),flush=True)
    raise SystemExit(0 if result['verdict']=='PASS' else 1)
