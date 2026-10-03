import ast,gzip,hashlib,json,shutil,sys
from pathlib import Path
import matplotlib,numpy,yaml
repo=Path('/home/qihei/rm2027_navigation');base=Path('/tmp/rm_native_individual_stages');out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_individual_stages';tools=repo/'src/rm_dynamic_obstacle_critic/tools';sys.path.insert(0,str(tools))
from prepare_processed_cv import verify
v=json.loads((base/'tests/verification.json').read_text());assert v['verdict']=='PASS' and len(v['checks'])==63
r=json.loads((base/'analysis/report.json').read_text());assert r['original_raw_nine_tensor_bytes_exact'] and r['all300_afterSG_full_labels_exact'] and r['all311_source_pose_full_labels_exact']
assert hashlib.sha256((base/'sdk_build/native_safe_control_witness').read_bytes()).hexdigest()=='d798fbd5050a943a6e9a5d4fb5d2e4cc0d2883891723a5e94b72f3c808abec01'
assert hashlib.sha256((base/'sdk_build/native_individual_stages').read_bytes()).hexdigest()=='ae89bcf377117db0beb71ba840dd23f61f72e3a33029c9fae661331e89d0bdb7'
refs={}
for name,folder in [('source','gazebo_dynamic_consumption_evidence'),('witness','dynamic_consumption_witness'),('raw_map','native_raw_map_contract'),('mass','native_factor_safe_mass')]:
 p=out.parent/folder;verify(p.resolve());refs[name]=(folder,hashlib.sha256((p/'manifest.json').read_bytes()).hexdigest())
out.mkdir()
def copy(src,rel,compress=False):
 dst=out/(rel+('.gz' if compress else ''));dst.parent.mkdir(parents=True,exist_ok=True)
 if compress:
  with dst.open('xb') as stream:
   with gzip.GzipFile(filename='',mode='wb',fileobj=stream,mtime=0) as writer:writer.write(src.read_bytes())
 else:shutil.copyfile(src,dst)
pending=['analyze_native_individual_stages','native_individual_stage_io','test_native_individual_stages','plot_native_individual_stages','replay_native_individual_stages','prepare_native_witness'];included=set()
while pending:
 name=pending.pop()
 if name in included:continue
 included.add(name);path=tools/(name+'.py');copy(path,'source/tools/'+path.name)
 for node in ast.walk(ast.parse(path.read_text())):
  modules=[node.module] if isinstance(node,ast.ImportFrom) else [n.name for n in node.names] if isinstance(node,ast.Import) else []
  for module in modules:
   if module and (tools/(module.split('.')[0]+'.py')).is_file():pending.append(module.split('.')[0])
copy(tools/'native_safe_control_witness.cpp','source/tools/native_safe_control_witness.cpp');copy(repo/'src/rm_dynamic_obstacle_critic/CMakeLists.txt','source/CMakeLists.txt')
copy(base/'prepared/input.bin','prepared/input.bin',True);copy(base/'prepared/input_identity.json','prepared/input_identity.json')
for src,dst in [('tests/positive.stages.bin','stages.bin'),('tests/positive.witness.bin','witness.bin'),('tests/positive.jsonl','witness.jsonl'),('analysis/report.json','analysis/report.json'),('analysis/raw185_scalar.json','analysis/raw185_scalar.json')]:copy(base/src,dst,True)
for path in sorted((base/'tests').rglob('*')):
 if path.is_file():copy(path,'verification/tests/'+str(path.relative_to(base/'tests')),path.suffix in ['.bin','.jsonl'])
for path in (base/'figures_reviewed').iterdir():copy(path,'figures/'+path.name)
for path in (base/'figures_final').iterdir():copy(path,'verification/first_successful_figures/'+path.name)
for name in ['build.py','build.log','initial_native_safe_control_witness.cpp','initial_binary_identities.txt','initial_plot_native_individual_stages.py','first_successful_plot_source.py','prepare.log','sdk_build.log','sdk_build_final.log','execute.log','analysis.log','tests.log','plot.log','plot_final.log','plot_reviewed.log']:
 copy(base/name,'verification/'+name)
copy(base/'offline/CMakeLists.txt','verification/offline_CMakeLists.txt');copy(base/'sdk_build/compile_commands.json','verification/compile_commands.json')
for target in ['native_safe_control_witness','native_individual_stages']:
 for src,dst in [('flags.make','flags.make'),('link.txt','link.txt')]:copy(base/'sdk_build/CMakeFiles'/(target+'.dir')/src,'verification/'+target+'_'+dst)
copy(Path(__file__),'verification/freeze.py')
provenance={'preregistration_checkpoint':'9626742','native_source_ordinal':298,'stage_rows':[300,311,311],'raw_row_offsets':[0,11,11],
 'source_modules':len(included),'native_probe_checks':63,'old_default_ELF_unchanged':True,'old311_binary_JSON_and_afterSG_nine_blocks_exact':True,
 'new_stage_ELF_sha256':'ae89bcf377117db0beb71ba840dd23f61f72e3a33029c9fae661331e89d0bdb7','stage_output_sha256':v['stage_sha256'],
 'registered_constraints':[0.1,-0.5,0.8,0.5,1.2],'initial_build_revision':'Initial stage ELF7153... preceded strict registered-parameter check; final compiled positive output is byte exact with initial stage/legacy outputs. Both initial source/identity and final flags are retained.',
 'preserved_failures':['Initial direct build assumed a missing archived link.txt; compiler output/script retained; SDK-only ament/CMake build succeeded.','Initial plot attempted numpy.asarray on raw bytes and failed to reshape; source/log retained. First successful figure legend obscured counts; its source/figures retained and final PNG visually checked.'],
 'exact_cell_scope':'Three stages of original raw185 only; all300 stage map labels remain conservative vector checks.',
 'legacy_gate_name':'SG_full_sequence_and_return_bounds checks numerical limits in every supplied stage; it does not establish that raw/pre-SG was filtered or accepted for publication.',
 'image_identity':'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
 'runtime_TF_profile_public_contracts_or_original_SDKs_changed':False,'actual_CostCritic_state_and_individual_increments':'unknown','physical_support_bounds':'unknown','original_task_raw203_age':'FAILED','composite':'NOT ESTABLISHED'}
for label,(folder,digest) in refs.items():provenance[label+'_archive']='../'+folder;provenance[label+'_manifest_sha256']=digest
(out/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python':sys.version,'numpy':numpy.__version__,'matplotlib':matplotlib.__version__,'pyyaml':yaml.__version__,'native_tests':'Existing offline Humble image Python3; no ROS initialization/network/hardware; no new packages.'},indent=2)+'\n')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope':'Raw/bounds/SG individual stages with exact original raw/311 joins and original strict conditional gates; no selected or published controls.','files':files},indent=2)+'\n')
print(json.dumps({'files':len(files),'bytes_excluding_manifest':sum((out/f).stat().st_size for f in files),'source_modules':len(included),'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest()}),flush=True)
