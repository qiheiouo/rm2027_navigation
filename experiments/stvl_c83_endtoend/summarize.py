"""Same-cycle durations, publication association and original gate accounting."""
import pathlib,json,sys,csv,gzip,math,bisect,collections
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/c83_cache';OUT=ROOT/'experiments/stvl_c83_endtoend/evidence'
sys.path.insert(0,str(ROOT/'experiments/stvl_execution_realtime'));from mapping import rows,previous_yaw_view
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
from causal_replay import lines

def stats(v):
 if not v:return None
 a=np.array(v,dtype=float);return dict(n=len(a),p50=float(np.quantile(a,.5)),p95=float(np.quantile(a,.95)),p99=float(np.quantile(a,.99)),max=float(a.max()),over100=int(sum(a>100)),over100_rate=float(np.mean(a>100)))
def run(p):
 event=json.loads((p/'events.json').read_text());goal=event['goal_ns'];end=event['end_ns'];tim=lines(p/'paired_timing.jsonl');control=lines(p/'control.jsonl');tim=[x for x in tim if goal<=x['epoch_ns']<=end];cs={x['cycle_id']:x for x in control};receipt=rows(p/'command_receipts.csv');nav=[x for x in receipt if x['topic']=='/cmd_vel_nav' and goal<=int(x['ROS_receipt_ns'])<=end];used=set();records=[]
 od=rows(p/'odometry.csv');ts=[int(x['source_ns']) for x in od]
 for x in tim:
  c=cs[x['cycle_id']];x=dict(x);x['relative_s']=(x['epoch_ns']-goal)/1e9
  # disjoint decomposition includes raw-read observer cost conservatively.
  x['T_input_disjoint_ms']=x['T_input_ms']+x['raw_read_ms'];x['T_mppi_disjoint_ms']=x['T_mppi_including_trace_ms']-x['raw_read_ms']
  assert abs(x['T_total_necessary_boundary_ms']-(x['T_input_disjoint_ms']+x['T_mppi_disjoint_ms']+x['T_check_ms']+x['T_other_prelog_ms']))<1e-6
  matches=[(i,r) for i,r in enumerate(nav) if i not in used and c.get('command') is not None and max(abs(float(r[k])-v) for k,v in zip(('vx','vy','wz'),c['command']))<1e-7 and int(r['steady_ns'])>=x['entry_steady_ns'] and int(r['steady_ns'])<=x['entry_steady_ns']+500000000]
  x['entry_to_DDS_publication_ms']=None;x['publisher']=None
  if matches:
   i,r=matches[0];used.add(i);pub=int(r['steady_ns'])+int(r['DDS_source_ns'])-int(r['system_ns']);x['entry_to_DDS_publication_ms']=(pub-x['entry_steady_ns'])/1e6;x['wrapper_to_DDS_publication_ms']=(pub-x['wrapper_pre_return_steady_ns'])/1e6;x['publisher']=r['publisher_gid']
  if c.get('command') is not None and c.get('aggregate'):
   target=c['pose_epoch_ns']+100000000;j=bisect.bisect_left(ts,target)
   if 0<j<len(ts):
    z=(target-ts[j-1])/(ts[j]-ts[j-1]);actual=[float(od[j-1][k])*(1-z)+float(od[j][k])*z for k in ('x','y','yaw','vx','vy','wz')];fixed=previous_yaw_view(c['pose'],c['selected_post_sg_shift_controls'])[0]
    x['nominal_first_to_actual_100ms_position_m']=math.dist(fixed[:2],actual[:2]);x['nominal_first_to_actual_100ms_yaw_rad']=abs(math.atan2(math.sin(fixed[2]-actual[2]),math.cos(fixed[2]-actual[2])));x['Twist_to_next_feedback_velocity_error']=[c['command'][q]-actual[q+3] for q in range(3)]
  x['native_safe_candidates']=c.get('prediction',{}).get('safe_candidates');x['prediction_reason']=c.get('prediction',{}).get('reason');x['shadow_selected']=next((q for q in c.get('shadow_checks',[]) if q['id']=='selected_previous_yaw'),None)
  records.append(x)
 keys=('T_input_disjoint_ms','T_mppi_disjoint_ms','T_check_ms','T_other_prelog_ms','T_total_necessary_boundary_ms','T_actual_wrapper_pre_return_ms','entry_to_DDS_publication_ms','control_log_ms','candidate_export_ms')
 res=dict(cohort=p.parent.parent.name,events=event,cycles=len(records),native_failed=sum(x['failed'] for x in records),timing={key:stats([x[key] for x in records if x.get(key) is not None]) for key in keys},warm_timing={key:stats([x[key] for x in records[1:] if x.get(key) is not None]) for key in keys},cold=records[0],entry_intervals_ms=stats([(b['entry_steady_ns']-a['entry_steady_ns'])/1e6 for a,b in zip(records,records[1:])]),publication_matches=sum(x['entry_to_DDS_publication_ms'] is not None for x in records),navigation_control_records=sum(goal<=x['epoch_ns']<=end for x in control),missing_timing_cycle_ids=[x['cycle_id'] for x in control if goal<=x['epoch_ns']<=end and x['cycle_id'] not in {q['cycle_id'] for q in records}],recovery_or_schedule_gaps_over150ms=[dict(previous_cycle_id=a['cycle_id'],next_cycle_id=b['cycle_id'],interval_ms=(b['entry_steady_ns']-a['entry_steady_ns'])/1e6) for a,b in zip(records,records[1:]) if b['entry_steady_ns']-a['entry_steady_ns']>150000000],nominal_to_actual_next_position_m=stats([x['nominal_first_to_actual_100ms_position_m'] for x in records if 'nominal_first_to_actual_100ms_position_m' in x]))
 (OUT/f'{p.parent.parent.name}_paired_cycles.json').write_text(json.dumps(records,separators=(',',':'))+'\n');return res
if __name__=='__main__':
 res=[]
 for p in sorted(CACHE.glob('*/runs/*')):
  if (p/'events.json').exists() and ((p/'paired_timing.jsonl').exists() or (p/'paired_timing.jsonl.gz').exists()):res.append(run(p))
 (OUT/'timing_summary.json').write_text(json.dumps(res,ensure_ascii=False,indent=2)+'\n');print([(x['cohort'],x['cycles'],x['timing']['T_total_necessary_boundary_ms']) for x in res])
