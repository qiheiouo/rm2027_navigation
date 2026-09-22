"""Recompute pilot evidence and isolate the first moving-clearance violation."""
import bisect
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import audit_dynamic
ex=audit_dynamic.ex
from dynamic_metrics import rows_from_transport, obstacle_polygon, geometry_metrics, placed, padded, polygon_distance
from simulation_geometry import OBSTACLES
from audit_endpoint_witness import audit_file
HERE=Path(__file__).resolve().parent
root=ex.WORK/'runs/dynamic_reference_pilot_v2'
m=json.loads((root/'inputs.json').read_text())
for n,h in m['files'].items():assert ex.sha(ex.REPO/n)==h,n
fig,axes=plt.subplots(2,3,figsize=(16,9));reviews=[]
for row,name in enumerate(('tdt_astar','tdt_qp')):
    p=root/f'{name}_1';s=json.loads((p/'dynamic_summary.json').read_text())
    assert audit_dynamic.inspect_dynamic_trial(p)['raw_summary_matches']
    for f,h in s['artifacts_sha256'].items():assert ex.sha(p/f)==h,f
    all_rows=rows_from_transport(p/'gazebo_poses.jsonl');times=[r['t'] for r in all_rows]
    start,end=s['navigation_interval_sim_s'];lo=max(0,bisect.bisect_right(times,start)-1);hi=bisect.bisect_left(times,end)
    rows=all_rows[lo:hi+1];poly=m['body_polygon_m'];obs=obstacle_polygon()
    assert json.loads(json.dumps(geometry_metrics(rows,poly,.03,obs)))==s['geometry']['new_car_reference']
    runtime_path=p/'runtime_geometry.json'
    if runtime_path.exists():
        runtime=json.loads(runtime_path.read_text())
        assert (runtime['map']['width'],runtime['map']['height'])==(260,240)
        for r in runtime['runtime'].values():assert r['expected']==r['actual']
        runtime_capture={'available':True,'footprint_parameters_verified':True}
    else:
        capture_log=(p/'geometry_capture.log').read_text()
        assert 'parameter timeout' in capture_log
        runtime_capture={'available':False,'footprint_parameters_verified':False,
            'reason':'Post-navigation parameter service timed out. No post-navigation full raw map captured; do not substitute a later run.'}
    published_maps=ex.read_rows(p/'observation/costmap.jsonl')
    assert published_maps and all((g['width'],g['height'])==(260,240) for g in published_maps)
    dist=[polygon_distance(placed(poly,r['robot']),placed(obs,r['obstacle'])) for r in rows]
    pdist=[polygon_distance(placed(padded(poly,.03),r['robot']),placed(obs,r['obstacle'])) for r in rows]
    index=next(i for i,d in enumerate(dist) if d<.05);critical=rows[index]
    overlap=next(i for i,d in enumerate(dist) if d==0.)
    cmd=ex.read_rows(p/'observation/commands.jsonl');events=ex.read_rows(p/'observation/events.jsonl')
    errors=[e for e in events if e.get('level',0)>=40]
    window={}
    for key,filename in [('global','costmap'),('local','local_costmap')]:
        maps=ex.read_rows(p/f'observation/{filename}.jsonl')
        candidates=[v for v in maps if v['t']<=critical['t']]
        if not candidates:window[key]={'covered':False};continue
        grid=candidates[-1];x,y,_=critical['obstacle'];r=grid['resolution'];ox,oy=grid['origin'];w=grid['width']
        pts=[(ox+(i%w+.5)*r,oy+(i//w+.5)*r) for i,c in enumerate(grid['data']) if c>=99]
        window[key]={'covered':True,'stamp':grid['t'],'age_s':critical['t']-grid['t'],'frame':grid['frame'],
            'lethal_centres_inside_actual_obstacle_box':sum(abs(px-x)<=.225 and abs(py-y)<=.275 for px,py in pts),
            'scope':'Published OccupancyGrid at/before violation; not planner raw snapshot; count alone is not marking/clearing acceptance.'}
    witness=s['geometry']['new_car_reference']['body']['moving_min_witness']
    robot_poly=placed(poly,witness['robot']);obstacle_poly=placed(obs,witness['obstacle'])
    xmin,xmax=min(v[0] for v in obstacle_poly),max(v[0] for v in obstacle_poly)
    ymin,ymax=min(v[1] for v in obstacle_poly),max(v[1] for v in obstacle_poly)
    inside=[v for v in robot_poly if xmin<v[0]<xmax and ymin<v[1]<ymax]
    endpoint=audit_file(p/'observation/events.jsonl');assert not endpoint['missing_witnesses']
    review={'planner':name,'summary':s,'first_below_005':{**critical,'body_gap_m':dist[index]},
        'first_sampled_body_overlap':rows[overlap],
        'command_before_first_violation':next(v for v in reversed(cmd) if v['t']<=critical['t']),
        'new_car_vertices_strictly_inside_obstacle_at_overlap':inside,
        'costmap_before_first_violation':window,'controller_errors':errors,'endpoint_audit':endpoint,
        'raw_summary_recomputed':True,'dynamic_geometry_recomputed':True,
        'post_navigation_runtime_geometry_capture':runtime_capture}
    reviews.append(review)
    ax=axes[row,0]
    for x0,y0,x1,y1 in OBSTACLES:ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='.4'))
    ax.plot([v['robot'][0] for v in rows],[v['robot'][1] for v in rows],label='Robot path')
    ax.plot([v['obstacle'][0] for v in rows],[v['obstacle'][1] for v in rows],color='tab:orange',label='Obstacle motion')
    ax.plot(5.6,0,'k*',ms=10);ax.set(title=f'{name}: action 4, recoveries {s["recoveries"]}',aspect='equal',xlabel='x (m)',ylabel='y (m)');ax.legend(fontsize=8);ax.grid(alpha=.2)
    ax=axes[row,1];ax.plot([v['t'] for v in rows],dist,label='New-car body');ax.plot([v['t'] for v in rows],pdist,label='Padded footprint');ax.axhline(.05,color='red',ls='--',label='Body gate 0.05 m');ax.set(title='Gap to actual moving obstacle',xlabel='Simulation time (s)',ylabel='Gap (m)',ylim=(-.02,.5));ax.legend(fontsize=8);ax.grid(alpha=.2)
    ax=axes[row,2]
    ax.add_patch(Polygon(robot_poly,fill=False,edgecolor='tab:blue',lw=2,label='New-car body'))
    ax.add_patch(Polygon(placed(padded(poly,.03),witness['robot']),fill=False,edgecolor='tab:blue',ls='--',label='Padding'))
    ax.add_patch(Polygon(obstacle_poly,facecolor='tab:orange',alpha=.4,label='Actual obstacle'))
    if inside:ax.scatter(*zip(*inside),color='red',s=30,label='Body vertex inside box')
    x,y,_=witness['robot'];ax.set(title=f'First sampled overlap: t={witness["t"]:.3f}',aspect='equal',xlim=(4.45,5.8),ylim=(y-.55,y+.8),xlabel='x (m)',ylabel='y (m)');ax.legend(fontsize=8);ax.grid(alpha=.2)
result={'schema':'rm_tdt_planner/dynamic_reference_pilot_review/v1','series':'dynamic_reference_pilot_v2',
 'consistent_and_audited':True,'full_evidence_complete':False,'recorded':2,'target_dynamic_pass':0,'complete_varied_phase_matrix':False,
 'accepted_for_deployment':False,'runtime_source_commit':m['runtime_source_commit'],'fixture_correction':m['fixture_correction'],
 'trials':reviews,'scope':'Two first-stage trials only; each group stopped after its first safety failure. Geometry uses actual synchronized Gazebo poses, not target joint commands.'}
ex.write_new(HERE/'aggregate.json',result)
fig.suptitle('Dynamic reference pilot: successful actions did not satisfy moving-clearance gates')
fig.tight_layout();fig.savefig(HERE/'dynamic_review.png',dpi=150);fig.savefig(HERE/'dynamic_review.svg')
print(json.dumps([{k:t[k] for k in ('planner','first_below_005','first_sampled_body_overlap','new_car_vertices_strictly_inside_obstacle_at_overlap','costmap_before_first_violation')} for t in reviews],indent=2))
