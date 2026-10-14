"""Selected original physical data + transparent reduced control trace.
Full original streams/candidates stay in unique local cache, never deleted.
"""
import pathlib,json,gzip,tarfile,hashlib,shutil
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/active_scope_cache';OUT=ROOT/'experiments/stvl_active_scope/evidence';STAGE=CACHE/'selected'
# This folder contains only this script's copied/derived files, not raw sources.
if STAGE.exists():shutil.rmtree(STAGE)
STAGE.mkdir()
for p in CACHE.glob('*/runs/*'):
 if not (p/'supervisor.json').exists():continue
 dest=STAGE/p.parent.parent.name/'runs'/p.name;dest.mkdir(parents=True,exist_ok=True)
 for name in ('truth.jsonl.gz','odometry.csv.gz','commands.csv.gz','command_receipts.csv.gz','costmap.csv.gz','projection.jsonl.gz','paired_timing.jsonl.gz','events.json','manifest.json','supervisor.json','nav2.yaml','scene.sdf','scene_adapter.json','scenario.json','launch.log','output_endpoints.txt','controller_odom_contract.txt','compression_manifest.json'):
  if (p/name).exists():shutil.copy2(p/name,dest/name)
 records=[]
 with gzip.open(p/'control.jsonl.gz','rt') as f:
  for l in f:
   x=json.loads(l)
   for name in ('critic_deltas','native_final_costs_after_gamma','selected_post_sg_shift_controls','aggregate'):x.pop(name,None)
   records.append(json.dumps(x,separators=(',',':')))
 with gzip.open(dest/'control_selected.jsonl.gz','wt') as f:f.write('\n'.join(records)+'\n')
for name in ('preparation.json','prediction_adapter.py','full_crossing.log','active_crossing_resource_abort.txt'):
 if (CACHE/name).exists():shutil.copy2(CACHE/name,STAGE/name)
files=[p for p in STAGE.rglob('*') if p.is_file()]
with tarfile.open(OUT/'paired_selected.tar.gz','w:gz') as tf:
 for p in files:tf.add(p,arcname=str(p.relative_to(STAGE)))
(OUT/'archive_manifest.json').write_text(json.dumps(dict(files=[dict(path=str(p.relative_to(STAGE)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files],scope='complete physical observation/truth/commands/projection/timing; control retains pose/time/output/prediction but omits critic vectors and optimized future U/aggregate; full1ms native, raw prediction messages and all candidates/control remain unique local cache',unique_cache=str(CACHE)),indent=2)+'\n')
size=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file());assert size<3*1024**2,size;print(json.dumps(dict(selected_bytes=size,archive_bytes=(OUT/'paired_selected.tar.gz').stat().st_size)))
