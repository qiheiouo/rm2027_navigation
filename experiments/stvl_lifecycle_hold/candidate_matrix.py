"""Separate native dynamic counter, nominal rate, joint geometry and optimization.
Read-only focused snapshots plus measured braking path. No candidate generator.
"""
import pathlib,json,sys,subprocess,math,bisect
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/lifecycle_cache';OUT=ROOT/'experiments/stvl_lifecycle_hold/evidence';BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014')
sys.path.insert(0,str(BASE/'experiments/stvl_c83_endtoend'));from causal_replay import lines,binary
sys.path.insert(0,str(BASE/'experiments/stvl_execution_realtime'));from mapping import previous_yaw_view,rows
from fixture_metrics import interp
p=BASE/'build/c83_cache/shadow_durable/runs/narrow_C_83_aligned';records=lines(p/'control.jsonl');zero=[r['cycle_id'] for r in records if r.get('prediction',{}).get('safe_candidates')==0];choose=zero+[next(r['cycle_id'] for r in records if r.get('prediction',{}).get('safe_candidates')==300)]
static=json.loads((p.parent.parent/'assets/narrow_static.json').read_text());fp=[[-.34,-.29],[.34,-.29],[.34,.29],[-.34,.29]];summary=[]
def execute(ctx):
 path=CACHE/'candidate_input.json';path.write_text(json.dumps(ctx,separators=(',',':'))+'\n');result=json.loads(subprocess.check_output([str(BASE/'build/c83_cache/plugin_build/replay_cycle'),str(path)],text=True));path.unlink();return result
for cycle,raw in binary(p/'paired_timing.jsonl.raw'):
 if cycle not in choose:continue
 c=records[cycle];tr=[dict(id=f'raw_{i}',poses=v[:,0,:].tolist(),velocities=v[:,1,:].tolist()) for i,v in enumerate(raw)];tr.append(dict(id='selected',poses=previous_yaw_view(c['pose'],c['selected_post_sg_shift_controls']),velocities=c['selected_post_sg_shift_controls']))
 ctx=dict(case=f'durable_{cycle}',pose=c['pose'],speed=c['speed'],pose_epoch_ns=c['pose_epoch_ns'],dt=.1,identity_frame_fixture=True,feedback_consistent=True,execution_mapping_known=False,prediction=c['prediction'],footprint=fp,static_map=static,trajectories=tr);result=execute(ctx);rr=result[:-1]
 count=lambda key:sum(x[key]=='Safe' for x in rr)
 summary.append(dict(case=ctx['case'],cycle_id=cycle,raw_candidates=len(raw),native_dynamic_safe=c['prediction']['safe_candidates'],nominal_rate_safe=count('nominal_rate'),diagnostic_static_wall_safe=count('static'),diagnostic_dynamic_swept_safe=count('dynamic'),diagnostic_joint_geometry_safe=count('joint_geometry'),diagnostic_joint_complete_safe=count('joint_complete'),optimization_scores=len(c['native_final_costs_after_gamma']),finite_scores=sum(math.isfinite(x) for x in c['native_final_costs_after_gamma']),selected=result[-1],definition='native safe discrete .02; independent wall/sweep/rate predicates; execution_mapping_known false'))
# Use actual fixture motion with unchanged original consumed prediction; input
# pose is the measured fixture prefix, so this is conditional, not exact old U.
original=lines(pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_execution_realtime_20261014/build/realtime_cache/corrected_scene/runs/narrow_C_83_aligned/control.jsonl'))[25];old_goal=4002000000
brakes=[]
for name in ('brake','brake_delayed'):
 p0=CACHE/name;e=json.loads((p0/'events.json').read_text());od=[x for x in rows(p0/'observations.csv') if x['topic']=='/physical_odom'];epoch=e['origin_ns']+original['pose_epoch_ns']-old_goal;initial=interp(od,epoch);future=[interp(od,epoch+(k+1)*100000000) for k in range(30)];assert initial is not None and all(x is not None for x in future)
 tr=[dict(id=name,poses=[x[:3].tolist() for x in future],velocities=[x[3:].tolist() for x in future])];ctx=dict(case=name,pose=initial[:3].tolist(),speed=initial[3:].tolist(),pose_epoch_ns=original['pose_epoch_ns'],dt=.1,identity_frame_fixture=True,feedback_consistent=True,execution_mapping_known=False,prediction=original['prediction'],footprint=fp,static_map=static,trajectories=tr);brakes.append(dict(case=name,actual_fixture_initial=initial.tolist(),original_input_pose=original['pose'],check=execute(ctx)[0],scope='measured physical braking future, original forecast, fixture prefix within predeclared tolerance; not online oracle or production action'))
(OUT/'candidate_semantics.json').write_text(json.dumps(dict(focused=summary,measured_brakes=brakes,original_raw='unavailable at c25; native safe0 means zero discrete clearances, not global invalid300'),indent=2)+'\n');print(summary);print(brakes)
