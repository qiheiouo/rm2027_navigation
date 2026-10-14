"""Post-trial read-only snapshot diagnosis; no holdout-driven code or tuning."""
import pathlib,json
from history_prepare import static_map,feedback_at,ROOT,jsonlines
cache=ROOT/'build/joint_cache';out=cache/'fresh_inputs';out.mkdir(exist_ok=True);index=[]
for cohort in ('fresh','observer_retry'):
    for p in sorted((cache/cohort/'runs').glob('*')):
        if not (p/'events.json').exists():continue
        e=json.loads((p/'events.json').read_text())
        for r in jsonlines(p/'control.jsonl'):
            ex=r.get('root_diagnostics',{});probe=ex.get('cost_probe')
            if not probe or 'raw' not in r:continue
            feed=feedback_at(p,r['pose_epoch_ns']);match=feed is not None and max(abs(a-b) for a,b in zip(feed['velocity'],r['speed']))<.005
            case=cohort+'__'+p.name+'__'+str(r['epoch_ns'])
            trajectories=[{'id':f'raw_{i}','poses':poses,'velocities':ex['candidate_velocity'][i]} for i,poses in enumerate(r['raw'])]
            trajectories.append({'id':'selected_sdk','poses':r['aggregate'],'velocities':r['selected_post_sg_shift_controls']})
            context={'case':case,'pose':r['pose'],'speed':r['speed'],'pose_epoch_ns':r['pose_epoch_ns'],'dt':.1,'identity_frame_fixture':True,'execution_mapping_known':False,'feedback_consistent':match,'measured_feedback':feed,'footprint':probe['footprint'],'prediction':r['prediction'],'static_map':static_map(p),'trajectories':trajectories}
            (out/(case+'.json')).write_text(json.dumps(context,separators=(',',':'))+'\n');index.append({'case':case,'relative_s':(r['epoch_ns']-e['goal_ns'])/1e9,'source':str(p/'control.jsonl'),'mode':e['mode']})
(ROOT/'experiments/stvl_joint_execution/evidence/fresh_snapshot_index.json').write_text(json.dumps(index,indent=2)+'\n');print('prepared',len(index),'bounded fresh snapshots')
