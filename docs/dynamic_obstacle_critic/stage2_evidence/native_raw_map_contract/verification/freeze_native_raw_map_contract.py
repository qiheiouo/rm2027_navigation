import ast
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys
import matplotlib
import numpy
import yaml

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1');out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_raw_map_contract'
for folder,count in [('native_raw_map_tests',72),('native_raw_map_evidence_tests',13),('native_raw_map_old_query_regression',79)]:
    result=json.loads((base/folder/'verification.json').read_text())
    if result['verdict']!='PASS' or len(result['checks'])!=count:raise ValueError('all compiled/evidence/default probes required')
if not (base/'native_raw_map_binaries.txt').read_text().startswith((base/'native_cost_map_binaries_final.txt').read_text()):raise ValueError('prior16 identities changed')
out.mkdir()
def copy(source,target,compress=False):
    target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():raise FileExistsError(target)
    if compress:
        with target.open('xb') as stream:
            with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as writer:writer.write(source.read_bytes())
    else:shutil.copyfile(source,target)
tools=repo/'src/rm_dynamic_obstacle_critic/tools';pending=['prepare_native_raw_map_queries','native_raw_map_io','verify_native_raw_map_queries',
    'analyze_native_raw_map_contract','verify_native_raw_map_subset','verify_native_raw_map_report','test_native_raw_map_queries',
    'test_native_raw_map_evidence','plot_native_raw_map_contract','verify_raw_map_mutation_recipes','replay_native_raw_map_contract','test_native_cost_map_queries'];included=set()
while pending:
    name=pending.pop()
    if name in included:continue
    included.add(name);path=tools/(name+'.py')
    for node in ast.walk(ast.parse(path.read_text())):
        modules=[node.module] if isinstance(node,ast.ImportFrom) else [n.name for n in node.names] if isinstance(node,ast.Import) else []
        for module in modules:
            if module and (tools/(module.split('.')[0]+'.py')).is_file():pending.append(module.split('.')[0])
    copy(path,out/'source/tools'/path.name)
copy(tools/'native_cost_map_queries.cpp',out/'source/tools/native_cost_map_queries.cpp');copy(repo/'src/rm_dynamic_obstacle_critic/CMakeLists.txt',out/'source/CMakeLists.txt')
for target in ['native_cost_map_queries','native_raw_map_queries']:
    for name in ['flags.make','link.txt']:copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles'/(target+'.dir')/name,out/'verification'/(target+'_'+name))
for path in (base/'native_raw_map_prepared').iterdir():copy(path,out/'prepared'/path.name)
for source,target in [('native_raw_map_queries.jsonl','native_queries.jsonl.gz'),('native_raw_map_scalar.json','scalar_verification.json.gz'),
    ('native_raw_map_contract_report.json','report.json.gz'),('native_raw_map_subset_final.json','cell_subset.json.gz')]:copy(base/source,out/target,True)
copy(base/'native_raw_map_report_verification.json',out/'verification.json')
for folder,target in [('native_raw_map_tests','native_tests'),('native_raw_map_evidence_tests','evidence_tests')]:
    for path in sorted((base/folder).rglob('*')):
        if not path.is_file():continue
        # Full bases plus lossless mutation recipes reproduce the omitted large
        # ephemeral matrices/native JSONL, all of which remain in /tmp.
        if path.suffix in ['.bin','.jsonl']:continue
        relative=path.relative_to(base/folder);compress=path.suffix=='.json' and path.name not in ['verification.json','positive.json','mutation.json'] and not path.name.endswith('.mutation.json')
        copy(path,out/'verification'/target/(str(relative)+('.gz' if compress else '')),compress)
copy(base/'native_raw_map_old_query_regression/verification.json',out/'verification/default79_verification.json')
for folder,target in [('native_raw_map_figures_final','figures'),('native_raw_map_figures','verification/initial_three_figures')]:
    for path in (base/folder).iterdir():copy(path,out/target/path.name)
copy(base/'native_raw_map_subset.json',out/'verification/initial_three_subset.json.gz',True)
copy(base/'native_raw_map_initial_three_subset_source.py',out/'verification/initial_three_subset_source.py')
for name in ['native_raw_map_prepare.log','native_raw_map_build.log','native_raw_map_execute.log','native_raw_map_scalar.log','native_raw_map_tests.log',
    'native_raw_map_analysis.log','native_raw_map_subset.log','native_raw_map_subset_final.log','native_raw_map_report_verification.log',
    'native_raw_map_evidence_tests.log','native_raw_map_plot.log','native_raw_map_plot_final.log','native_raw_map_binaries.log','native_raw_map_binaries.txt']:
    copy(base/name,out/'verification'/name)
provenance={'preregistration_checkpoint':'0852a4a','native_source_ordinal':298,'original_raw_rows':300,'raw30_exact_queries':18000,
    'dense_queries':124500,'total_queries':142500,'traces':1200,'native_probe_checks':72,'evidence_checks':13,'default_probe_checks':79,
    'exact_polygon_cell_subset':[2,41,42,73,82,127,160,161,185,244],'source_modules':len(included),
    'prior16_ELF_and_core_identities_unchanged':True,'runtime_TF_public_contract_and_profiles_changed':False,
    'actual_CostCritic_full_state_and_cell_source':'unknown','original_task_raw203_age':'FAILED','full_witness_assessment':'NOT ESTABLISHED',
    'physical_support_bounds':'unknown','visual_review':'Final PNG inspected: original raw73 contact, raw185 positive exact map margin and all300 predicate counts are legible.',
    'mutation_storage':'Full frozen base query/native bytes plus 38 lossless patches/JSON-row recipes and SHA256; every large ephemeral negative retained in trial /tmp.',
    'image_identity':'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
    'limits':'All300 raw vector map labels are conservative; exact polygon-square scalar labels cover ten selected paths. Raw controls are not SG-accepted commands.'}
for label,folder in [('source','gazebo_dynamic_consumption_evidence'),('witness','dynamic_consumption_witness'),('raw','raw_rollout_objective'),('previous_map','native_cost_map_contract')]:
    provenance[label+'_archive']='../'+folder;provenance[label+'_manifest_sha256']=hashlib.sha256((out.parent/folder/'manifest.json').read_bytes()).hexdigest()
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python':sys.version,'numpy':numpy.__version__,'pyyaml':yaml.__version__,'matplotlib':matplotlib.__version__,
    'native_build':'Existing Release/O3, no fast-math; separate RM_NATIVE_RAW_MAP=1 target; unchanged installed SDK, no ROS initialization or network.'},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_native_raw_map_contract.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Same original raw paths: original grids, dense queries, conservative all300 vector and exact ten-path geometry; actual critic unknown.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'source_modules':len(included)}))
