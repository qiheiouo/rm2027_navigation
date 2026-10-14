"""First changed command → limited target visibility, not torque or exact receipt.
Only changed stop trajectories with one input/output publisher, repeated zero
Twists are not falsely treated as uniquely identified applications.
"""
import pathlib,json,sys,bisect,math
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/lifecycle_cache';OUT=ROOT/'experiments/stvl_lifecycle_hold/evidence';BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014')
sys.path.insert(0,str(BASE/'experiments/stvl_execution_realtime'));from mapping import rows
sys.path.insert(0,str(ROOT/'experiments/stvl_lifecycle_hold'));from fixture_metrics import jl,interp
result=[]
for label in ('brake','brake_delayed'):
 p=CACHE/label;e=json.loads((p/'events.json').read_text());ins=jl(p/'inputs.jsonl');zero=next(x for x in ins if x['event']=='new_output' and x['cycle_id']==1000);cpp=rows(p/'command_receipts.csv');native=rows(p/'native.csv');obs=[x for x in rows(p/'observations.csv') if x['topic']=='/physical_odom'];origin=e['origin_ns'];start=origin+round(e['brake_trigger_s']*1e9);nsim=[int(x['sim_ns']) for x in native];nwall=[int(x['steady_ns']) for x in native]
 for x in cpp:x['pub_wall_ns']=int(x['steady_ns'])+int(x['DDS_source_ns'])-int(x['system_ns'])
 nav=next(x for x in cpp if x['topic']=='/cmd_vel_nav' and x['pub_wall_ns']>=zero['publish_wall_ns']-1000000 and max(abs(float(x[k])) for k in ('vx','vy','wz'))<1e-7)
 prior=[x for x in cpp if x['topic']=='/cmd_vel' and x['pub_wall_ns']<nav['pub_wall_ns']][-1];sm=next(x for x in cpp if x['topic']=='/cmd_vel' and x['pub_wall_ns']>=nav['pub_wall_ns'] and float(x['vx'])<float(prior['vx'])-.001)
 stub=next(x for x in cpp if x['topic']=='/simulation/chassis/cmd_vel' and x['pub_wall_ns']>=sm['pub_wall_ns']-100000 and max(abs(float(x[k])-float(sm[k])) for k in ('vx','vy','wz'))<1e-7)
 begin=bisect.bisect_left(nwall,stub['pub_wall_ns']);before=native[max(0,begin-1)];change=next(x for x in native[begin:] if abs(float(x['applied_vx'])-float(before['applied_vx']))>1e-6)
 st=interp(obs,start);en=interp(obs,zero['publish_sim_ns']);delaymove=None if st is None or en is None else float(np.linalg.norm(en[:2]-st[:2]))
 result.append(dict(label=label,decision_ready_to_actual_pub_ms=zero['actual_release_to_publish_ms'],nav_to_smoother_publication_ms=(sm['pub_wall_ns']-nav['pub_wall_ns'])/1e6,smoother_to_stub_publication_ms=(stub['pub_wall_ns']-sm['pub_wall_ns'])/1e6,stub_to_first_changed_limited_target_ms=(int(change['steady_ns'])-stub['pub_wall_ns'])/1e6,observed_component_sim_ns=int(change['sim_ns']),first_smoothed_command=[float(sm[k]) for k in ('vx','vy','wz')],first_limited_target=[float(change[k]) for k in ('applied_vx','applied_vy','applied_wz')],movement_before_stop_publication_m=delaymove,application_scope='PreUpdate component visibility,1ms/order uncertainty; physical pose separately observed; no MCU or force timestamp'))
(OUT/'stop_application.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
