"""Read-only frozen snapshots; no new tracker or controller data source."""
import pathlib,json,csv,sys,math,bisect
import yaml
from PIL import Image
ROOT=pathlib.Path(__file__).resolve().parents[2];OLD=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_temporal_failure_roots_20261013')
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal_roots'));from data import jsonlines
OUT=ROOT/'build/joint_cache/inputs';OUT.mkdir(exist_ok=True)
def static_map(p):
 cfg=yaml.safe_load((p/'map.yaml').read_text());im=Image.open(p/'map.pgm');w,h=im.size;pixels=list(im.getdata());grid=[]
 for y in range(h):
  for x in range(w):
   v=pixels[(h-1-y)*w+x];occ=(v if cfg.get('negate',0) else 255-v)/255;grid.append(254 if occ>cfg['occupied_thresh'] else 0 if occ<cfg['free_thresh'] else 255)
 return {'width':w,'height':h,'resolution':cfg['resolution'],'origin':cfg['origin'][:2],'grid':grid,'scope':'original map-server occupancy; current STVL remains separate'}
def feedback_at(p,t):
 rows=list(csv.DictReader((p/'odometry.csv').open()));eligible=[r for r in rows if int(r['source_ns'])<=t];r=eligible[-1] if eligible else None
 return None if r is None else {'source_ns':int(r['source_ns']),'velocity':[float(r[k]) for k in ('vx','vy','wz')]}
def main():
 specs=[('diag','wait_release_C_29_aligned',2.8),('diag','narrow_D_29_aligned',4.1),('repair_odom','wait_release_C_13_aligned',3.2),('repair_odom','wait_release_C_29_aligned',4.1),('repair_odom','narrow_D_47_aligned',3.8),('repair_odom','narrow_C_29_aligned',3.2),('heldout_abi','narrow_D_61_aligned',4.2),('heldout_odom','narrow_D_61_aligned',4.2)]
 index=[]
 for cohort,name,relative in specs:
  p=OLD/'build/roots_cache'/cohort/'runs'/name;e=json.loads((p/'events.json').read_text());rows=[r for r in jsonlines(p/'control.jsonl') if r.get('root_diagnostics',{}).get('cost_probe') and 'raw' in r];r=min(rows,key=lambda r:abs((r['epoch_ns']-e['goal_ns'])/1e9-relative));extra=r['root_diagnostics'];probe=extra['cost_probe'];pred=r['prediction'];fp=probe['footprint'];feed=feedback_at(p,r['pose_epoch_ns']);speed_match=feed is not None and max(abs(a-b) for a,b in zip(feed['velocity'],r['speed']))<.005
  trajectories=[{'id':f'raw_{i}','poses':poses,'velocities':extra['candidate_velocity'][i],'control_targets':extra['candidate_noised_controls'][i]} for i,poses in enumerate(r['raw'])]
  trajectories.append({'id':'selected_sdk','poses':r['aggregate'],'velocities':r['selected_post_sg_shift_controls'],'control_targets':r['selected_post_sg_shift_controls']})
  context={'case':cohort+'__'+name,'pose':r['pose'],'speed':r['speed'],'pose_epoch_ns':r['pose_epoch_ns'],'dt':.1,'identity_frame_fixture':True,'execution_mapping_known':False,'feedback_consistent':speed_match,'measured_feedback':feed,'footprint':fp,'prediction':pred,'static_map':static_map(p),'trajectories':trajectories}
  path=OUT/(context['case']+'.json');path.write_text(json.dumps(context,separators=(',',':'))+'\n')
  index.append({'input':str(path),'source_control':str(p/'control.jsonl.gz'),'source_epoch_ns':r['epoch_ns'],'relative_s':(r['epoch_ns']-e['goal_ns'])/1e9,'native_master_footprint_hits':probe['full_native_footprint_hits'],'native_shortcut_hits':probe['native_shortcut_hits'],'native_dynamic_min_clearance':pred.get('candidate_min_clearance'),'native_costs_after_gamma':r['native_final_costs_after_gamma'],'native_temperature':r['native_temperature'],'feedback_consistent':speed_match,'source_group':e['mode']})
 (ROOT/'experiments/stvl_joint_execution/evidence/history_index.json').write_text(json.dumps(index,indent=2)+'\n');print('prepared',len(index),'bounded contexts')

if __name__=='__main__':main()
