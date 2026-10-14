"""Generate small update/view/retrospective inputs from immutable prior evidence."""
import pathlib,json,copy,csv,sys
from mapping import ROOT,CACHE,previous_yaw_view
sys.path.insert(0,str(ROOT/'experiments/stvl_joint_execution'));from audit import interpolate
prior=ROOT.parent/'stvl_joint_execution_20261014';old=prior/'build/joint_cache';e=prior/'experiments/stvl_joint_execution/evidence'
for name in ('updates','view_inputs','execution_inputs'):(CACHE/name).mkdir(exist_ok=True)
a=json.loads((old/'unit_inputs/known_safe_wait.json').read_text())
for name,change in [('new_map_occupied','occupied'),('new_map_unknown_after_zero','unknown'),('expired_new_source','expired'),('fresh_source_wrong_frame','frame'),('new_footprint','footprint')]:
    b=copy.deepcopy(a);b['case']=name
    if change=='occupied':b['static_map']['grid'][60*120+60]=254
    if change=='unknown':b['static_map']['grid'][60*120+60]=254;b['static_map']['grid'][60*120+61]=255
    if change=='expired':b['prediction']['consume_ns']+=401000000
    if change=='frame':b['prediction']['source']['frame']='odom'
    if change=='footprint':b['footprint']=[[-.4,-.32],[.4,-.32],[.4,.32],[-.4,.32]];b['static_map']['grid'][66*120+67]=254
    (CACHE/'updates'/f'{name}.json').write_text(json.dumps(b,separators=(',',':'))+'\n')
sources={pathlib.Path(m['input']).stem:pathlib.Path(m['source_control']).parent for m in json.loads((e/'history_index.json').read_text())}
sources.update({m['case']:pathlib.Path(m['source']).parent for m in json.loads((e/'fresh_snapshot_index.json').read_text())})
for directory in ('inputs','fresh_inputs'):
    for p in (old/directory).glob('*.json'):
        ctx=json.loads(p.read_text());case=ctx['case'];selected=ctx['trajectories'][-1]
        ctx['trajectories']=[dict(selected,id='original_sdk'),dict(selected,id='corrected_previous_yaw',poses=previous_yaw_view(ctx['pose'],selected['velocities']))];ctx['case']='view__'+case
        (CACHE/'view_inputs'/f"{ctx['case']}.json").write_text(json.dumps(ctx,separators=(',',':'))+'\n')
        od=list(csv.DictReader((sources[case]/'odometry.csv').open()));ts=[int(r['source_ns']) for r in od];future=[interpolate(od,ts,ctx['pose_epoch_ns']+(k+1)*100000000) for k in range(30)]
        if any(r is None for r in future):continue
        ctx['trajectories'].append({'id':'observed_future_execution','poses':[r[:3] for r in future],'velocities':None});ctx['case']='execution__'+case
        (CACHE/'execution_inputs'/f"{ctx['case']}.json").write_text(json.dumps(ctx,separators=(',',':'))+'\n')
print('prepared snapshot updates, selected-view and recorded-execution inputs')
