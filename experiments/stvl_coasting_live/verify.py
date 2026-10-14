"""Recompute selected physical metrics; native 1ms gate requires full cache."""
import hashlib,importlib.util,json,pathlib,shutil,tarfile
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_coasting_live/evidence';DEST=ROOT/'build/coasting_live_cache/recovery_check';DEST.mkdir(exist_ok=True)
manifest=json.loads((OUT/'archive_manifest.json').read_text());known={x['path']:x['sha256'] for x in manifest['files']}
with tarfile.open(OUT/'paired_selected.tar.gz') as tf:
 for member in tf:
  assert member.name in known and member.isfile();data=tf.extractfile(member).read();assert hashlib.sha256(data).hexdigest()==known[member.name]
  path=DEST/member.name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
spec=importlib.util.spec_from_file_location('recovery_stage_metrics',ROOT/'experiments/stvl_active_scope/analyze.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.OUT=DEST/'derived';m.OUT.mkdir(exist_ok=True)
checks=[]
fields=('arrival_s','body_clearance_m','base_collision_box_gap_m','wall_clearance_m','wait_s','path_length_m')
for kind in ('strict','allow_equal'):
 p=DEST/kind/'runs/narrow_C_83_aligned';shutil.copy2(p/'control_selected.jsonl.gz',p/'control.jsonl.gz');r=m.run(p,native_required=False);original=json.loads((OUT/(kind+'.json')).read_text());errors={k:abs(r[k]-original[k]) for k in fields};assert max(errors.values())<=1e-9
 assert r['safe_success']==original['safe_success'] and r['output_single_owner']==original['output_single_owner'];checks.append({'variant':kind,'selected_physical_metric_absolute_errors':errors,'success_owner_preserved':True})
(OUT/'recovery_verification.json').write_text(json.dumps({'archive_member_hash_checks':len(known),'checks':checks,'not_recomputed':'1ms native acceleration/full cost vectors require preserved unique full cache; selected archive is transparently partial'},indent=2)+'\n')
print('selected physical metrics reproduced; full native cache still required for complete gate')
