"""Small owned runtime overlay, no edits to frozen checkout or installed modules."""
import pathlib,json,subprocess,hashlib,shutil
ROOT=pathlib.Path(__file__).resolve().parents[2];BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014');CACHE=ROOT/'build/active_scope_cache';CACHE.mkdir(exist_ok=True)
def source(path):return subprocess.check_output(['git','show','a880c7dc01cb424ea865e1fa0f33732396078670:'+path],cwd=BASE,text=True)
text=source('experiments/stvl_temporal/prediction_adapter.py')
text=text.replace('import rclpy\n','import rclpy\nimport os\nsys.path.insert(0,"/study")\nfrom projection import project\n',1)
anchor="oracle=json.loads(oracle_file.read_text()) if mode=='D' and oracle_file.exists() else None"
assert text.count(anchor)==1;text=text.replace(anchor,anchor+'\nprojection_log=(out/"projection.jsonl").open("w",buffering=1)')
anchor=' publisher.publish(String(data=json.dumps(payload,separators=(\',\',\':\'))))'
assert text.count(anchor)==1;text=text.replace(anchor,' payload=project(payload,os.environ["PROJECTION_KIND"],projection_log)\n'+anchor)
(CACHE/'prediction_adapter.py').write_text(text)
assets_source=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_temporal_failure_roots_20261013/build/roots_cache/assets')
for kind in ('full','active'):
 assets=CACHE/kind/'assets';assets.mkdir(parents=True,exist_ok=True)
 for name in ['bridge.yaml','scenario.json','common_nav2.yaml']+[s+ext for s in ('crossing','narrow') for ext in ('.sdf','.pgm','.yaml')]:
  shutil.copy2(assets_source/name,assets/name)
 shutil.copy2(BASE/'build/c83_cache/shadow_durable/assets/narrow_static.json',assets/'narrow_static.json')
 shutil.copy2(BASE/'build/c83_cache/shadow_durable/assets/crossing_static.json',assets/'crossing_static.json')
manifest=dict(base='63faca0dbafc74ac8d4e44814f499a0b12d88027',readonly_runtime='a880c7dc01cb424ea865e1fa0f33732396078670',adapter_sha256=hashlib.sha256(text.encode()).hexdigest(),only_change='adapter scope and equal diagnostic logging; no original file/optimizer changes',assets={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (CACHE/'full/assets').iterdir()})
(CACHE/'preparation.json').write_text(json.dumps(manifest,indent=2)+'\n');print(manifest['adapter_sha256'])
