"""Reuse original navigation metrics; add scoped prediction and paired timing."""
import pathlib,json,sys,collections,math,csv,gzip,bisect,xml.etree.ElementTree as ET,importlib.util
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014');CACHE=ROOT/'build/active_scope_cache';OUT=ROOT/'experiments/stvl_active_scope/evidence';OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(BASE/'experiments/stvl_temporal'));spec=importlib.util.spec_from_file_location('frozen_temporal_metrics',BASE/'experiments/stvl_temporal/analyze.py');legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
sys.path.insert(0,str(BASE/'experiments/stvl_temporal_roots'));from data import jsonlines
sys.path.insert(0,str(BASE/'experiments/stvl_execution_realtime'));from mapping import rows
legacy.jsonlines=jsonlines;legacy.csvrows=rows
def stats(v):return legacy.stats(v)
def run(p,native_required=True):
 legacy.CACHE=p.parent.parent;r=legacy.trial(p);r['input_scope']=p.parent.parent.name
 event=json.loads((p/'events.json').read_text());start=event['goal_ns'];end=event['end_ns']
 truth=jsonlines(p/'truth.jsonl');bytime={}
 for x in truth:
  if start is not None and start<=x['source_ns']<=end:bytime.setdefault(x['source_ns'],{})[x['model']]=x
 # Actual tilted base collision box, same original SDF half dimensions. Mechanical
 # bbox remains the primary original .047 gate; this separate .005 gate is unchanged.
 box=ET.parse(p/'scene.sdf').getroot().find("world/model[@name='rm_sentry_2027']/link[@name='base_link']/collision[@name='base_collision']");size=np.array(list(map(float,box.findtext('geometry/box/size').split())));offset=np.array(list(map(float,box.findtext('pose').split())));assert all(abs(x)<1e-12 for x in offset[3:]);half=size/2;gaps=[]
 for t,m in sorted(bytime.items()):
  if 'rm_sentry_2027' not in m or 'moving_obstacle' not in m:continue
  rob=m['rm_sentry_2027'];pos=np.array(rob['position']);rot=np.array(rob['rotation']);points=[(pos+rot@(np.array([x,y,z])+offset[:3]))[:2].tolist() for x in (-half[0],half[0]) for y in (-half[1],half[1]) for z in (-half[2],half[2])]
  # Convex hull from the frozen physical oracle, no instant-body substitution.
  sys.path.insert(0,'/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004/experiments/temporal_mpc/gazebo');from audit_run import hull
  gaps.append(legacy.polygon_distance(hull(points),m['moving_obstacle']['polygon']))
 r['base_collision_box_gap_m']=min(gaps) if gaps else None;r['base_collision_box_geometry']={'size':size.tolist(),'pose':offset.tolist(),'source':'actual frozen scene.sdf base_collision'}
 r['hard_gate_pass']=bool(r.get('safe_success') and gaps and min(gaps)>=.005 and r.get('output_single_owner') and not r.get('output_bounds_violations'))
 projection=jsonlines(p/'projection.jsonl');r['projection']={'packets':len(projection),'with_ignored_tentative':sum(bool(x['ignored_ids']) for x in projection),'active_empty_packets':sum(x['active_empty'] for x in projection),'fields_preserved_all':all(x['active_fields_unchanged'] and x['source_reconstructable'] for x in projection),'core_ms_before_logging':stats([x['projection_ms'] for x in projection])}
 control=[x for x in jsonlines(p/'control.jsonl') if start is not None and start<=x['epoch_ns']<=end];timing=[x for x in jsonlines(p/'paired_timing.jsonl') if start is not None and start<=x['epoch_ns']<=end];receipt=[x for x in rows(p/'command_receipts.csv') if x['topic']=='/cmd_vel_nav' and start is not None and start<=int(x['ROS_receipt_ns'])<=end];cs={x['cycle_id']:x for x in control};used=set();pubtimes=[]
 for x in timing:
  c=cs[x['cycle_id']];cmd=c.get('command')
  if cmd is None:continue
  hit=next(((i,z) for i,z in enumerate(receipt) if i not in used and x['entry_steady_ns']<=int(z['steady_ns'])<=x['entry_steady_ns']+500000000 and max(abs(float(z[k])-v) for k,v in zip(('vx','vy','wz'),cmd))<1e-7),None)
  if hit:
   i,z=hit;used.add(i);pub=int(z['steady_ns'])+int(z['DDS_source_ns'])-int(z['system_ns']);pubtimes.append((pub-x['entry_steady_ns'])/1e6)
 r['entry_to_DDS_publication_ms']=stats(pubtimes);r['entry_to_DDS_over100']=sum(x>100 for x in pubtimes)
 r['wrapper_ms_including_exports']=stats([x['T_actual_wrapper_pre_return_ms'] for x in timing]);r['candidate_export_ms']=stats([x['candidate_export_ms'] for x in timing]);r['native_safe0_fresh_nonzero']=sum(x.get('prediction',{}).get('safe_candidates')==0 and x.get('command') is not None and max(abs(v) for v in x['command'])>1e-6 for x in control)
 native=rows(p/'native_boundary.csv') if native_required else [];max_acc=[0.,0.,0.];bad=0;valid=0;outside_bad=0
 for a,b in zip(native,native[1:]):
  if not(start<=int(a['sim_ns'])<=end and start<=int(b['sim_ns'])<=end):
   if any(b.get(k) in (None,'') for k in ('applied_vx','applied_vy','applied_wz')):outside_bad+=1
   continue
  try:
   dt=(int(b['sim_ns'])-int(a['sim_ns']))/1e9
   if dt<=0:continue
   for j,k in enumerate(('applied_vx','applied_vy','applied_wz')):max_acc[j]=max(max_acc[j],abs(float(b[k])-float(a[k]))/dt)
   valid+=1
  except (KeyError,ValueError,TypeError):bad+=1
 r['native_limited_target_max_acceleration']=max_acc if native_required else None;r['native_valid_intervals']=valid if native_required else None;r['native_parse_bad']=bad if native_required else None;r['native_outside_navigation_incomplete_rows']=outside_bad if native_required else None;r['hard_gate_pass']=(r['hard_gate_pass'] and valid>0 and bad==0 and max(max_acc)<=2.01) if native_required else None
 r['rate_warnings']=(p/'launch.log').read_text().count('Control loop missed its desired rate');r['incomplete_original_lines']={f.name:sum(1 for l in gzip.open(f,'rt') if not legacy.valid_json(l)) for f in p.glob('*.jsonl.gz')}
 r['contacts_outcome']=bool(event['contacts']);r['first_contact_relative_s']=None if not event['contacts'] or start is None else (event['contacts'][0]['source_ns']-start)/1e9 if 'source_ns' in event['contacts'][0] else None
 # Full per-critic diagnostics remain in unique local original streams.
 r.pop('snapshots',None)
 (OUT/(r['input_scope']+'_'+r['run']+'.json')).write_text(json.dumps(r,indent=2)+'\n');return r
if __name__=='__main__':
 result=[run(p) for p in sorted(CACHE.glob('*/runs/*')) if (p/'events.json').exists() and (p/'supervisor.json').exists()]
 (OUT/'paired_summary.json').write_text(json.dumps(result,indent=2)+'\n')
 for r in result:print(json.dumps({k:r.get(k) for k in ('input_scope','scene','success','hard_gate_pass','arrival_s','body_clearance_m','base_collision_box_gap_m','native_errors','prediction_reason_counts','zero_safe_candidates','entry_to_DDS_publication_ms','entry_to_DDS_over100','rate_warnings')}))
