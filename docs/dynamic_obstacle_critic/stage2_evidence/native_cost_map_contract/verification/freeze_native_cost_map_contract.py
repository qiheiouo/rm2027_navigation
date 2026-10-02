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

repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_dynamic_critic_v1')
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_cost_map_contract';out.mkdir()
def copy(source,target,compress=False):
    target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():raise FileExistsError(target)
    if compress:
        with target.open('xb') as stream:
            with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as writer:writer.write(source.read_bytes())
    else:shutil.copyfile(source,target)
tools=repo/'src/rm_dynamic_obstacle_critic/tools';pending=['prepare_native_cost_map_queries','native_cost_map_io','verify_native_cost_map_queries',
    'test_native_cost_map_queries','analyze_native_cost_map_contract','test_native_cost_map_contract','plot_native_cost_map_contract','replay_native_cost_map_contract'];included=set()
while pending:
    name=pending.pop()
    if name in included:continue
    included.add(name);path=tools/(name+'.py')
    for node in ast.walk(ast.parse(path.read_text())):
        modules=[node.module] if isinstance(node,ast.ImportFrom) else [n.name for n in node.names] if isinstance(node,ast.Import) else []
        for module in modules:
            if module and (tools/(module.split('.')[0]+'.py')).is_file():pending.append(module.split('.')[0])
    copy(path,out/'source/tools'/path.name)
copy(tools/'native_cost_map_queries.cpp',out/'source/tools/native_cost_map_queries.cpp')
copy(repo/'src/rm_dynamic_obstacle_critic/CMakeLists.txt',out/'source/CMakeLists.txt')
copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles/native_cost_map_queries.dir/flags.make',out/'verification/native_cost_map_queries_flags.make')
copy(base/'build/rm_dynamic_obstacle_critic/CMakeFiles/native_cost_map_queries.dir/link.txt',out/'verification/native_cost_map_queries_link.txt')
for path in (base/'native_cost_map_query_input').iterdir():copy(path,out/'prepared'/path.name)
for source,target,compress in [('native_cost_map_queries.jsonl','native_queries.jsonl.gz',True),('native_cost_map_synthetic.jsonl','native_synthetic.jsonl',False),
    ('native_cost_map_contract_report.json','report.json',False),('native_cost_map_scalar_final.json','scalar_verification.json',False)]:copy(base/source,out/target,compress)
if (base/'native_cost_map_scalar_final.json').read_bytes()!=(base/'native_cost_map_scalar_initial.json').read_bytes():raise ValueError('scalar metadata strengthening changed numerical result')
for folder,target in [('native_cost_map_tests','native_tests'),('native_cost_map_contract_tests','contract_tests')]:
    folder='native_cost_map_contract_tests_final' if target=='contract_tests' else folder
    result=json.loads((base/folder/'verification.json').read_text());expected=79 if target=='native_tests' else 9
    if result['verdict']!='PASS' or len(result['checks'])!=expected:raise ValueError('all format/native/provenance probes required')
    for path in sorted((base/folder).rglob('*')):
        if not path.is_file():continue
        relative=path.relative_to(base/folder);compress=path.suffix in ['.bin','.jsonl']
        copy(path,out/'verification'/target/(str(relative)+('.gz' if compress else '')),compress)
for path in (base/'native_cost_map_contract_figures').iterdir():copy(path,out/'figures'/path.name)
for name in ['native_cost_map_headers.txt','native_cost_map_source_inventory.txt','native_cost_map_checker_header_and_symbols.txt',
    'native_cost_map_method_inspection.txt','native_cost_map_line_iterator_header.txt','native_cost_map_line_and_footprint_inspection.txt']:
    copy(base/name,out/'verification/SDK_inspection'/(name+'.gz'),True)
for name in ['native_cost_map_query_build.log','native_cost_map_query_prepare.log','native_cost_map_execute.log','native_cost_map_scalar_initial.log',
    'native_cost_map_scalar_final.log','native_cost_map_tests.log','native_cost_map_contract_analysis.log','native_cost_map_contract_analysis_failed_source.py',
    'native_cost_map_contract_analysis_final.log','native_cost_map_contract_tests.log','native_cost_map_contract_plot.log',
    'native_cost_map_contract_tests_final.log',
    'native_cost_map_binaries_initial.txt','native_cost_map_binaries.log','native_cost_map_binaries_final.log','native_cost_map_binaries_final.txt']:
    copy(base/name,out/'verification'/name)
copy(base/'native_cost_map_contract_tests/verification.json',out/'verification/initial_eight_provenance_probes.json')
old=(base/'native_sg_stage_binaries_final.txt').read_text();actual=(base/'native_cost_map_binaries_final.txt').read_text()
if not actual.startswith(old) or len(actual.splitlines())!=16:raise ValueError('all prior14 ELF identities required')
if actual.splitlines()[-2:]!=(base/'native_cost_map_binaries_initial.txt').read_text().splitlines():raise ValueError('new helper/map core identities changed')
report=json.loads((base/'native_cost_map_contract_report.json').read_text())
if report['scalar_mismatch_count']!=0 or report['query_count']!=21165 or report['actual_CostCritic_full_state']!='unknown':raise ValueError('complete frozen helper proof required')
provenance={'preregistration_checkpoint':'40d9a32','native_source_ordinal':298,'queries':21165,'traces':102,'model_paths':51,
    'runtime_TF_public_contract_or_native_libraries_changed':False,'prior_fourteen_ELFs_unchanged':True,
    'native_checker_library':'/opt/ros/humble/lib/libnav2_costmap_2d_core.so','native_checker_library_sha256':actual.splitlines()[-1].split()[0],
    'original_task_raw203_age':'FAILED','full_witness_assessment':'NOT ESTABLISHED','physical_support_bounds':'unknown',
    'actual_CostCritic_state_and_cell_source':'unknown','critical_cost_semantics':'not established by this helper experiment',
    'visual_review':'PNG inspected: center zero, boundary202/216/229 and whole polygon first contacts are readable.',
    'preserved_setup_failures':['First analysis passed relative paths to an absolute-root manifest verifier; retained source and log; original archives verified intact.',
        'First Docker identity attempt omitted stdin forwarding and produced an empty log; actual -i execution generated and verified 16 identity lines.'],
    'image_identity':'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
    'limits':'Helper boundary costs differ from whole polygon plus continuous reserve; actual complete CostCritic scoring/classification not replayed.'}
for label,folder in [('source','gazebo_dynamic_consumption_evidence'),('witness','dynamic_consumption_witness'),('stages','native_sg_stages'),('mass','native_factor_safe_mass')]:
    provenance[label+'_archive']='../'+folder;provenance[label+'_manifest_sha256']=hashlib.sha256((out.parent/folder/'manifest.json').read_bytes()).hexdigest()
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python':sys.version,'numpy':numpy.__version__,'pyyaml':yaml.__version__,'matplotlib':matplotlib.__version__,
    'native_probe_python':'Existing Humble image Python3; standard library only; no ROS initialization, network or hardware.',
    'native_build':'Existing Release/O3; no fast-math flags; existing SDK declarations/library only, no upstream source vendoring.'},indent=2)+'\n')
copy(Path(__file__),out/'verification/freeze_native_cost_map_contract.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Native frozen-map boundary helper versus whole polygon/raw203; actual critic unknown and all original failures retained.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes':sum(p.stat().st_size for p in out.rglob('*') if p.is_file()),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest(),'source_modules':len(included)}))
