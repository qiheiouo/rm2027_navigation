"""Small recovery subset; all full native/control streams remain locally."""
import gzip,hashlib,json,pathlib,shutil,tarfile
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/coasting_live_cache';OUT=ROOT/'experiments/stvl_coasting_live/evidence';STAGE=CACHE/'selected'
assert not STAGE.exists(),'preserve previous archive attempt'
STAGE.mkdir()
for p in CACHE.glob('*/runs/*'):
 dest=STAGE/p.parent.parent.name/'runs'/p.name;dest.mkdir(parents=True)
 for name in ('truth.jsonl.gz','odometry.csv.gz','commands.csv.gz','command_receipts.csv.gz','costmap.csv.gz','paired_timing.jsonl.gz','events.json','manifest.json','supervisor.json','nav2.yaml','scene.sdf','scene_adapter.json','scenario.json','launch.log','output_endpoints.txt','controller_odom_contract.txt','compression_manifest.json','supervisor_error.txt'):
  if (p/name).exists():shutil.copy2(p/name,dest/name)
 if (p/'control.jsonl.gz').exists():
  with gzip.open(p/'control.jsonl.gz','rt') as f,gzip.open(dest/'control_selected.jsonl.gz','wt') as g:
   for line in f:
    x=json.loads(line)
    for key in ('critic_deltas','native_final_costs_after_gamma','selected_post_sg_shift_controls','aggregate'):x.pop(key,None)
    g.write(json.dumps(x,separators=(',',':'))+'\n')
for name in ('preparation.json','strict.log','allow_equal.log','build.log','strict_resource_abort.txt','allow_equal_resource_abort.txt'):
 if (CACHE/name).exists():shutil.copy2(CACHE/name,STAGE/name)
files=[p for p in STAGE.rglob('*') if p.is_file()]
with tarfile.open(OUT/'paired_selected.tar.gz','w:gz') as tf:
 for p in files:tf.add(p,arcname=str(p.relative_to(STAGE)))
manifest={'files':[{'path':str(p.relative_to(STAGE)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in files],'scope':'full physical truth/odom/commands/timing and selected control without full cost/U vectors; full 1ms native/control remain unique local cache; no candidate raw was created','unique_cache':str(CACHE)}
(OUT/'archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');assert sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())<3*1024**2
print('selected recovery archive created; full unique evidence retained')
