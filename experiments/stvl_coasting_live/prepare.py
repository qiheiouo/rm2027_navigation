"""Build only owned copies of existing research plugins; full prediction input."""
import hashlib, importlib.util, json, pathlib, shutil, subprocess
ROOT=pathlib.Path(__file__).resolve().parents[2]
BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014')
REV='a880c7dc01cb424ea865e1fa0f33732396078670'
CACHE=ROOT/'build/coasting_live_cache'
def source(path):return subprocess.check_output(['git','show',REV+':'+path],cwd=BASE,text=True)
def change_once(s,a,b):assert s.count(a)==1,a;return s.replace(a,b)
def main():
 CACHE.mkdir(exist_ok=True);assert shutil.disk_usage(CACHE).free>3.08*1024**3
 module_path=ROOT/'experiments/stvl_observer_export/prepare.py';spec=importlib.util.spec_from_file_location('observer_prepare',module_path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);_,shadow=mod.header()
 plugins=change_once(source('experiments/stvl_c83_endtoend/plugins.cpp'),'#include "../stvl_temporal/vendor/model.hpp"','#include "model.hpp"')
 cmake=source('experiments/stvl_c83_endtoend/CMakeLists.txt');begin=cmake.index('add_executable(replay_cycle');end=cmake.index('ament_package()');cmake=cmake[:begin]+cmake[end:]
 supervisor=source('experiments/stvl_c83_endtoend/run_trial.py');supervisor=change_once(supervisor,"parameters['research_crash_trace']=False","parameters['research_crash_trace']=False\nparameters['candidate_export_enabled']=False")
 supervisor=change_once(supervisor,"control_intervention=False,study=", "control_intervention=False,candidate_export_enabled=False,coasting_relation=os.environ['COAST_KIND'],study=")
 (CACHE/'run_trial.py').write_text(supervisor)
 original_model=source('experiments/stvl_temporal/vendor/model.hpp');model=change_once(original_model,'#include "geometry.hpp"','#include "'+str(BASE/'experiments/stvl_temporal/vendor/geometry.hpp')+'"')
 assets_source=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_temporal_failure_roots_20261013/build/roots_cache/assets');identity={}
 for kind in ('strict','allow_equal'):
  project=CACHE/'projects'/kind;project.mkdir(parents=True,exist_ok=True)
  (project/'plugins.cpp').write_text(plugins);(project/'shadow.hpp').write_text(shadow);(project/'CMakeLists.txt').write_text(cmake)
  variant=model if kind=='strict' else change_once(model,'observed_ns >= source_ns','observed_ns > source_ns')
  (project/'model.hpp').write_text(variant)
  for f in ('controller.xml','critics.xml','package.xml'):(project/f).write_text(source('experiments/stvl_c83_endtoend/'+f))
  assets=CACHE/kind/'assets';assets.mkdir(parents=True,exist_ok=True)
  for f in ('bridge.yaml','scenario.json','common_nav2.yaml','narrow.sdf','narrow.pgm','narrow.yaml'):shutil.copy2(assets_source/f,assets/f)
  shutil.copy2(BASE/'build/c83_cache/shadow_durable/assets/narrow_static.json',assets/'narrow_static.json')
  identity[kind]={'files':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in project.iterdir()},'assets':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in assets.iterdir()}}
 assert identity['strict']['assets']==identity['allow_equal']['assets']
 assert all(identity['strict']['files'][f]==identity['allow_equal']['files'][f] for f in identity['strict']['files'] if f!='model.hpp')
 (CACHE/'preparation.json').write_text(json.dumps({'frozen_commit':REV,'original_model_sha256':hashlib.sha256(original_model.encode()).hexdigest(),'variants':identity,'original_sources_untouched':True,'supervisor_changes':'equal explicit export=false and provenance metadata only'},indent=2)+'\n')
 print('two identical projects/assets except one relational operator; full input, no controller change')
if __name__=='__main__':main()
