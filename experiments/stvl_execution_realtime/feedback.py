"""Reproduce fixed Gazebo6.18 velocity estimation from actual model poses.

The 10-sample average updates every physics step before publish throttling.
Do not confuse this state estimate with native limited wheel target velocity.
"""
import pathlib,csv,math,collections,json,sys,numpy as np
from mapping import rows
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_execution_realtime/evidence';CACHE=ROOT/'build/realtime_cache'
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));from analyze import stats
result=[]
for p in CACHE.glob('*/runs/*'):
    if not (p/'native_boundary.csv').exists() and not (p/'native_boundary.csv.gz').exists():continue
    native=rows(p/'native_boundary.csv');dq=collections.deque(maxlen=10);by={};last=None
    for r in native:
        if last is not None:
            dt=(int(r['sim_ns'])-int(last['sim_ns']))/1e9;dx=float(r['x'])-float(last['x']);dy=float(r['y'])-float(last['y']);yaw=float(r['yaw']);dyaw=math.atan2(math.sin(yaw-float(last['yaw'])),math.cos(yaw-float(last['yaw'])))
            c=float(np.cos(np.float32(yaw)));s=float(np.sin(np.float32(yaw)));dq.append([(c*dx+s*dy)/dt,(c*dy-s*dx)/dt,float(np.float32(dyaw))/dt]);by[int(r['sim_ns'])]=[sum(v[k] for v in dq)/len(dq) for k in range(3)]
        last=r
    errors=[[] for _ in range(3)]
    for r in rows(p/'odometry.csv'):
        q=by.get(int(r['source_ns']))
        if q:
            for k,f in enumerate(('vx','vy','wz')):errors[k].append(abs(q[k]-float(r[f])))
    result.append({'cohort':p.parents[1].name,'run':p.name,'physical_model_comparable':p.parents[1].name=='corrected_scene','component_errors':[stats(v) for v in errors],
        'units':['m/s','m/s','rad/s'],'basis':'1ms actual model-pose differences, current-yaw cosf/sinf, angular float32 and window10; publication50Hz. Source generation semantics confirmed, not an execution model.'})
(OUT/'odometry_reconstruction.json').write_text(json.dumps(result,indent=2)+'\n');print('reconstructed',len(result),'streams; max error',max(s['max'] for r in result for s in r['component_errors']))
