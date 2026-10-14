"""Native/control/physics boundary accounting; retrospective observations only.

DDS source timestamps are publication, not optimizer-entry or motor application.
Native wheel target application is not measured actuator force. Physics velocity
from 1ms pose differences is explicitly distinct from unavailable component data.
"""
import pathlib,json,csv,sys,math,bisect,collections,gzip
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_execution_realtime/evidence';CACHE=ROOT/'build/realtime_cache'
OLD=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_joint_execution_20261014/build/joint_cache')
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal_roots'));from data import jsonlines
sys.path.insert(0,str(ROOT/'experiments/stvl_joint_execution'));import audit
V=('vx','vy','wz')
def rows(p):
    actual=p if p.exists() else pathlib.Path(str(p)+'.gz');opener=gzip.open if actual.suffix=='.gz' else open
    with opener(actual,'rt') as f:return list(csv.DictReader(f))
def present(p):return p.exists() or pathlib.Path(str(p)+'.gz').exists()
def vector(r,fields=V):return [float(r[k]) for k in fields]
def clamp(v,b):return max(-b,min(b,v))
def wrap(v):return math.atan2(math.sin(v),math.cos(v))
def previous_yaw_view(p,commands,dt=.1):
    p=p[:];out=[]
    for vx,vy,wz in commands:
        yaw=p[2];p=[p[0]+(math.cos(yaw)*vx-math.sin(yaw)*vy)*dt,p[1]+(math.sin(yaw)*vx+math.cos(yaw)*vy)*dt,yaw+wz*dt];out.append(p)
    return out
def held(commands,t,topic):
    selected=[r for r in commands if r['topic']==topic and int(r['receipt_ns'])<=t]
    return selected[-1] if selected else None
def interpolate_scalar(ts,vals,t):
    i=bisect.bisect_left(ts,t)
    if i==0 or i>=len(ts):return None
    q=(t-ts[i-1])/(ts[i]-ts[i-1]);return vals[i-1]+q*(vals[i]-vals[i-1])
def run(p,cohort):
    e=json.loads((p/'events.json').read_text());start,end=e['goal_ns'],e['end_ns'];od=rows(p/'odometry.csv');ts=[int(r['source_ns']) for r in od];commands=rows(p/'commands.csv');control=[r for r in jsonlines(p/'control.jsonl') if start<=r['epoch_ns']<=end]
    records=[];captured=[];categories=collections.Counter();first={};native=[];receipts=[];gid_names={}
    for r in jsonlines(p/'execution_endpoints.jsonl'):
        for topic,endpoints in r['topics'].items():
            for ep in endpoints:gid_names[ep['gid']]=ep['name']
    if present(p/'native_boundary.csv'):native=rows(p/'native_boundary.csv')
    if present(p/'command_receipts.csv'):receipts=rows(p/'command_receipts.csv')
    nw=[int(r['steady_ns']) for r in native];nt=[int(r['sim_ns']) for r in native]
    nav=[r for r in receipts if r['topic']=='/cmd_vel_nav' and start<=int(r['ROS_receipt_ns'])<=end];used=set()
    for i,c in enumerate(control):
        if 'command' not in c or not c.get('aggregate'):continue
        epoch=c['pose_epoch_ns'];initial=audit.interpolate(od,ts,epoch);actual=audit.interpolate(od,ts,epoch+100000000)
        if initial is None or actual is None:continue
        velocity=c['speed'];acc=[(actual[j+3]-initial[j+3])/.1 for j in range(3)];plan=c['aggregate'][0];position_error=math.dist(plan[:2],actual[:2]);yaw_error=abs(wrap(plan[2]-actual[2]));actual_velocity_error=[c['command'][j]-actual[j+3] for j in range(3)]
        flags=[];t=(epoch-start)/1e9
        if t<=1.2:flags.append('A1')
        if abs(velocity[0])>=.2 and abs(velocity[1])<=.03 and abs(velocity[2])<=.05 and max(abs(a) for a in acc[:2])<=.1:flags.append('A2')
        if abs(acc[0])>.2 or (math.hypot(*velocity[:2])>=.02 and math.hypot(*actual[3:5])<.02):flags.append('A3')
        if abs(velocity[1])>=.08:flags.append('A4')
        if abs(velocity[2])>=.15 and math.hypot(*velocity[:2])>=.08:flags.append('A5')
        active=any(tr['state'] in (2,3) for tr in c.get('prediction',{}).get('source',{}).get('tracks',[]))
        if e['scene'] in ('narrow','wait_release') and active and math.hypot(*velocity[:2])<.08:flags.append('A6')
        categories.update(flags)
        record={'cycle':i,'relative_s':t,'entry_ns':c['epoch_ns'],'input_pose_ns':epoch,'input_pose':c['pose'],'feedback':velocity,'measured_feedback_at_input':initial[3:],
            'SDK_next_pose':plan,'actual_next_pose':actual[:3],'actual_next_velocity':actual[3:],'raw_Twist':c['command'],
            'held_smoother_or_behavior_at_entry':held(commands,c['epoch_ns'],'/cmd_vel'),'held_sim_output_at_entry':held(commands,c['epoch_ns'],'/simulation/chassis/cmd_vel'),
            'position_error_m':position_error,'yaw_error_rad':yaw_error,'velocity_error_components':actual_velocity_error,'categories':flags,'publication':None,
            'controller_reported_partial_compute_ms':c['compute_ms'],'captured_large_diagnostics':'raw' in c}
        # Match actual publication by Twist and chronology, never by minimum cost.
        choices=[(j,r) for j,r in enumerate(nav) if j not in used and c['epoch_ns']-5000000<=int(r['ROS_receipt_ns'])<=c['epoch_ns']+250000000 and max(abs(a-b) for a,b in zip(vector(r),c['command']))<1e-7]
        if choices:
            j,r=choices[0];used.add(j);source=int(r['DDS_source_ns']);pub_steady=int(r['steady_ns'])+source-int(r['system_ns']);pub_sim=interpolate_scalar(nw,nt,pub_steady) if nw else None
            record['publication']={'DDS_source_system_ns':source,'receipt_ROS_ns':int(r['ROS_receipt_ns']),'estimated_publication_sim_ns_from_native_steps':pub_sim,
               'input_pose_to_publication_ms':None if pub_sim is None else (pub_sim-epoch)/1e6,'entry_to_publication_ms':None if pub_sim is None else (pub_sim-c['epoch_ns'])/1e6,
               'DDS_publication_to_cpp_receipt_ms':(int(r['system_ns'])-source)/1e6,'publisher':gid_names.get(r['publisher_gid'],r['publisher_gid'])}
        records.append(record)
        for category in flags:
            if category not in first:first[category]=record
        if position_error>.001 and 'SDK_to_execution' not in first:first['SDK_to_execution']=record
        if max(abs(a-b) for a,b in zip(velocity,initial[3:]))>.001 and 'feedback_to_input' not in first:first['feedback_to_input']=record
        if 'raw' in c and 'selected_post_sg_shift_controls' in c:
            expected=previous_yaw_view(c['pose'],[c['speed']])[0];raw_error=max(math.dist(pose[0][:2],expected[:2]) for pose in c['raw']);fixed=previous_yaw_view(c['pose'],c['selected_post_sg_shift_controls']);view_errors=[math.dist(a[:2],b[:2]) for a,b in zip(fixed,c['aggregate'])]
            item={'cycle':i,'relative_s':t,'raw_first_pose_reference_max_error_m':raw_error,'returned_vs_actual_selected_sequence_max_error':max(abs(a-b) for a,b in zip(c['command'],c['selected_post_sg_shift_controls'][0])),
               'native_raw_velocity_first_equals_feedback_max_error':max(max(abs(a-b) for a,b in zip(v[0],c['speed'])) for v in c['root_diagnostics']['candidate_velocity']),
               'SDK_previous_yaw_view_max_error_m':max(view_errors),'SDK_first_material_view_step':next((k for k,x in enumerate(view_errors) if x>.001),None)}
            captured.append(item)
    stage_stats={cat:{key:analyze.stats([r[key] for r in records if cat in r['categories']]) for key in ('position_error_m','yaw_error_rad')} for cat in ('A1','A2','A3','A4','A5','A6')}
    publication=[r['publication'] for r in records if r['publication'] and r['publication']['input_pose_to_publication_ms'] is not None]
    result={'run':p.name,'cohort':cohort,'physical_model_comparable':cohort not in ('boundary_enabled','no_capture'),'categories':dict(categories),'case_stats':stage_stats,'control_records':len(records),'captured_records':captured,'first_boundaries':first,
        'publication_matches':len(publication),'input_pose_to_publication_ms':analyze.stats([r['input_pose_to_publication_ms'] for r in publication]),
        'DDS_publication_to_cpp_receipt_ms':analyze.stats([r['DDS_publication_to_cpp_receipt_ms'] for r in publication]),'native_boundary':None}
    result['captured_periods_publication']=[{'cycle':r['cycle'],'relative_s':r['relative_s'],'partial_compute_ms':r['controller_reported_partial_compute_ms'],'publication':r['publication']} for r in records if r['captured_large_diagnostics']]
    if native:
        ndt=[int(r['dt_ns']) for r in native];applied_acc=[[] for _ in range(3)];response=[];target_flags=[];unchanged=0;mean_errors=[];mean_component_errors=[[] for _ in range(3)];pose_agreement=[]
        for a,b in zip(native,native[1:]):
            dt=(int(b['sim_ns'])-int(a['sim_ns']))/1e9
            if dt<=0:continue
            target=vector(b,('applied_vx','applied_vy','applied_wz'));previous=vector(a,('applied_vx','applied_vy','applied_wz'))
            for j in range(3):applied_acc[j].append(abs(target[j]-previous[j])/dt)
            # Component-sampled pose differences are NOT certified instantaneous
            # physical velocity. Native WorldVelocity is absent in this fixture.
            yaw=float(b['yaw']);dx=(float(b['x'])-float(a['x']))/dt;dy=(float(b['y'])-float(a['y']))/dt;w=wrap(float(b['yaw'])-float(a['yaw']))/dt
            measured=[math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,w]
            unchanged+=math.hypot(dx,dy)<1e-12
            if start<=int(b['sim_ns'])<=end:response.append(math.dist(target,measured))
            if max(abs(x) for x in target)<1e-8:target_flags.append(int(b['sim_ns']))
        for a,b in zip(od,od[1:]):
            lo,hi=int(a['source_ns']),int(b['source_ns']);j,k=bisect.bisect_right(nt,lo),bisect.bisect_right(nt,hi)
            if k<=j or not start<=hi<=end:continue
            average=[sum(float(native[l][key]) for l in range(j,k))/(k-j) for key in ('applied_vx','applied_vy','applied_wz')]
            mean_errors.append(math.dist(average,vector(b)))
            for component in range(3):mean_component_errors[component].append(abs(average[component]-vector(b)[component]))
            last=native[k-1];pose_agreement.append(math.dist(vector(last,('x','y')),vector(b,('x','y'))))
        result['native_boundary']={'rows':len(native),'dt_ns_counts':dict(collections.Counter(ndt)),
            'direct_world_velocity_available_rows':sum(int(r['valid']) for r in native),'actual_velocity_basis':'existing 50Hz source-stamped /odometry/lio body velocity. 1ms component pose derivative is diagnostic only, not an instantaneous physical measurement',
            'native_applied_target_acceleration':[analyze.stats(v) for v in applied_acc],'native_acceleration_exceed_2p01':[sum(v>2.01 for v in values) for values in applied_acc],
            'wheel_target_vs_component_pose_derivative_error_diagnostic_only':analyze.stats(response),'unchanged_component_pose_intervals':unchanged,
            'wheel_target_interval_mean_vs_50Hz_body_feedback_error':analyze.stats(mean_errors),'native_pose_vs_same_time_odom_position_error_m':analyze.stats(pose_agreement),
            'target_vs_feedback_component_errors':[analyze.stats(v) for v in mean_component_errors],'component_units':['m/s','m/s','rad/s'],
            'mixed_norm_limit':'legacy combined norm is diagnostic only, mixes m/s and rad/s; use component errors for interpretation',
            'application_semantics':'PreUpdate native JointVelocityCmd target snapshot, state after same physical update; component application, not measured motor force or MCU'}
    if receipts:
        result['receipt_topics']={t:{'n':sum(r['topic']==t for r in receipts),'DDS_available':sum(r['topic']==t and int(r['DDS_source_ns'])>0 for r in receipts),'publishers':dict(collections.Counter(gid_names.get(r['publisher_gid'],r['publisher_gid']) for r in receipts if r['topic']==t))} for t in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel','/odometry/lio')}
    (CACHE/'mapped_cycles').mkdir(exist_ok=True);(CACHE/'mapped_cycles'/(cohort+'__'+p.name+'.json')).write_text(json.dumps(records,separators=(',',':'))+'\n')
    return result
def main():
    output=[]
    for cache,cohort in [(OLD/'fresh','prior_fresh'),(OLD/'observer_retry','prior_retry'),(CACHE/'fresh','receipt_only'),(CACHE/'boundary_enabled','boundary_enabled'),(CACHE/'no_capture','no_capture'),(CACHE/'corrected_scene','corrected_scene')]:
        for p in sorted((cache/'runs').glob('*')):
            if (p/'events.json').exists():output.append(run(p,cohort))
    (OUT/'mapping_results.json').write_text(json.dumps({'runs':output,'scope':'SDK X, actual candidate first step, and observed execution are separate contracts. Native WorldVelocity unavailable; no producer/actuator time fabricated.'},indent=2)+'\n')
    print('mapped',len(output),'old/new runs, categories',dict(sum((collections.Counter(r['categories']) for r in output),collections.Counter())))
if __name__=='__main__':main()
