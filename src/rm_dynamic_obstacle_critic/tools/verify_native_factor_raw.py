#!/usr/bin/env python3
"""Independent scalar blocked-cell and interval-reserve audit, full three seconds."""
import argparse
import bisect
import json
import math
from pathlib import Path
from analyze_trial import load_truth_rows, rotation, distance, travel
from native_factor_io import read_paths
from native_snapshot_io import read_snapshot


def verify(source,path,report):
    native,blocks=read_snapshot(source/'native_cycles/cycle_298.json');record=read_paths(path)
    if report['native_source_ordinal']!=298 or len(report['rows'])!=4:raise ValueError('fixed full factor report required')
    policy=json.loads((source/'policy.json').read_text());threshold=policy['raw_costmap_threshold']
    if threshold!=203:raise ValueError('original raw203 threshold required')
    grid=native['map'];res=grid['resolution'];width,height=grid['size'];ox,oy=grid['origin']
    cells=[]
    for index,value in enumerate(blocks['raw_costmap']['values']):
        if value>=threshold:
            x,y=index%width,index//width;lo=(ox+x*res,oy+y*res)
            cells.append({'x':x,'y':y,'value':value,'lo':lo,'hi':(lo[0]+res,lo[1]+res),
                          'polygon':[lo,(lo[0]+res,lo[1]),(lo[0]+res,lo[1]+res),(lo[0],lo[1]+res)]})
    truth=load_truth_rows(source,json.loads((source/'execution.json').read_text()));times=[r['t'] for r in truth]
    footprint=native['padded_footprint'];radius=max(math.hypot(*p) for p in footprint);epochs={}
    for epoch,summary in report['epoch_summaries'].items():
        start=summary['epoch'];dt=record['model_dt'];knots=[start+k*dt for k in range(31)]
        samples=sorted(set(knots+[t for t in times if start<t<knots[-1]]));results=[]
        if len(samples)!=summary['samples']:raise ValueError('full scalar sample grid differs')
        for row,label in enumerate(report['rows']):
            coordinates=[]
            for axis,name in enumerate(['x','y','yaw']):
                initial=native['pose'][axis] if axis<2 else record['initial_yaw']
                values=[initial,*record['blocks'][name]['values'][row*30:(row+1)*30]]
                if axis==2:
                    values=[values[0],*[values[0]+sum(math.remainder(values[k]-values[k-1],2*math.pi) for k in range(1,j+1)) for j in range(1,31)]]
                coordinates.append(values)
            poses=[]
            for t in samples:
                j=min(30,max(1,bisect.bisect_right(knots,t)))
                fraction=(t-knots[j-1])/(knots[j]-knots[j-1])
                poses.append([v[j-1]+fraction*(v[j]-v[j-1]) for v in coordinates])
            reserves=[0.]*len(samples)
            for i in range(len(samples)-1):
                half=.5*travel(poses[i],poses[i+1],radius)
                reserves[i]=max(reserves[i],half);reserves[i+1]=max(reserves[i+1],half)
            lower=math.inf;first_contact=None;closest=None
            for i,(t,pose,reserve) in enumerate(zip(samples,poses,reserves)):
                polygon=rotation(pose,footprint)
                lo=[min(p[a] for p in polygon) for a in (0,1)];hi=[max(p[a] for p in polygon) for a in (0,1)]
                best=max(min(lo[0]-ox,lo[1]-oy,ox+width*res-hi[0],oy+height*res-hi[1]),0.)
                selected=None
                def aabb(cell):
                    return math.hypot(max(cell['lo'][0]-hi[0],lo[0]-cell['hi'][0],0),max(cell['lo'][1]-hi[1],lo[1]-cell['hi'][1],0))
                # AABB distance lower bounds true polygon-square distance. Once
                # larger than the best exact gap it cannot change the minimum.
                for bound,cell in sorted(((aabb(c),c) for c in cells),key=lambda pair:(pair[0],pair[1]['y'],pair[1]['x'])):
                    if bound>best+2e-12:break
                    gap=distance(polygon,cell['polygon'])
                    if gap<best or gap==best and selected is None:
                        best=gap;selected=cell
                    if best==0:break
                witness={'sample_index':i,'epoch':t,'future_seconds':t-start,'pose':pose,'reserve_m':reserve,
                         'exact_gap_to_blocked_cells_or_boundary_m':best,'gap_minus_reserve_m':best-reserve,
                         'cell':{k:selected[k] for k in ['x','y','value','lo','hi']} if selected else None}
                if best==0 and first_contact is None:first_contact=witness
                if best-reserve<lower:lower=best-reserve;closest=witness
            original=label['full_conditional_labels'][epoch]
            if original['raw_interval_margin_lower']>lower+2e-12:
                raise ValueError('vector raw conservative lower exceeds independent exact cell bound')
            passed=lower>1e-9
            if passed!=original['gates']['frozen_native_raw203_with_interval_reserve']:
                raise ValueError('independent full raw203 gate classification differs')
            results.append({'row':row,'factor':label['factor'],'scalar_exact_raw_interval_margin_m':lower,
                'captured_conservative_margin_m':original['raw_interval_margin_lower'],'raw_gate_pass':passed,
                'sampled_raw_cell_or_boundary_contact':first_contact is not None,'first_contact':first_contact,'minimum_margin_witness':closest})
        epochs[epoch]={'samples_per_path':len(samples),'blocked_cells':len(cells),'threshold':threshold,'rows':results}
    return {'verdict':'PASS','epochs':epochs,'numerical_comparison_only':2e-12,
            'scope':'Independent scalar polygon-square/boundary distances and source frozen raw203 reserve; cell contact is a raw-map label, not an executed physical collision.'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','paths','report','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=verify(args.source,args.paths,json.loads(args.report.read_text()))
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
