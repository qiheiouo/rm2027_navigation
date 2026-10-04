import hashlib,json,subprocess,tarfile,shutil
from pathlib import Path
work=Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004'); original=Path('/home/qihei/rm2027_navigation')
root=work/'build/temporal_mpc_candidates_20261004'
names=['crossing_b0_01','crossing_single_01','crossing_portfolio_01','head_on_b0_01','head_on_b0_02','head_on_single_02','head_on_portfolio_02']
def git(cwd,*args):return subprocess.check_output(['git',*args],cwd=cwd,text=True).strip()
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
files={}
for name in names:
 with tarfile.open(root/name/'source_snapshot.tar.gz') as tar:
  files[name]={m.name:hashlib.sha256(tar.extractfile(m).read()).hexdigest() for m in tar.getmembers() if m.isfile()}
left=files[names[0]];right=files[names[4]]
differences={k:{'before':left.get(k),'after':right.get(k)} for k in sorted(left.keys()|right.keys()) if left.get(k)!=right.get(k)}
expected={'experiments/temporal_mpc/gazebo/record_run.py','experiments/temporal_mpc/gazebo/verify_static_capture.py'}
identities={n:json.loads((root/n/'runtime_identity.json').read_text()) for n in names}
result={'source_hashes':files,'same_crossing_and_failed_baseline_sources':all(files[n]==left for n in names[:4]),'same_corrected_head_on_sources':all(files[n]==right for n in names[4:]),'cross_group_differences':differences,'only_registered_recorder_changes':set(differences)==expected,'same_prestart_binaries_and_dependencies':all((v['binaries'],v['dependencies'])==(identities[names[0]]['binaries'],identities[names[0]]['dependencies']) for v in identities.values()),'all_identities_prestart':all(v['captured_before_process_start'] for v in identities.values()),'current_branch':git(work,'branch','--show-current'),'main_base':git(work,'merge-base','HEAD','main'),'formal_src_diff':git(work,'diff','main','--name-only','--','src'),'post_physics_runtime_changes':False,'post_physics_plot_change':'Read scene.goal instead of treating fixture_profile string as a mapping; plotting only.'}
assert all(result[k] for k in ['same_crossing_and_failed_baseline_sources','same_corrected_head_on_sources','only_registered_recorder_changes','same_prestart_binaries_and_dependencies','all_identities_prestart'])
assert not result['formal_src_diff']
(root/'source_identity.json').write_text(json.dumps(result,indent=2)+'\n')
(root/'binaries').mkdir(exist_ok=True)
for relative,item in identities[names[0]]['binaries'].items():
 p=work/relative;assert digest(p)==item['sha256'];shutil.copy2(p,root/'binaries'/p.name)
base=json.loads(Path('/tmp/temporal_mpc_preserved_20261004.json').read_text());preserved={}
for relative,item in base['files'].items():
 p=original/relative;observed={'size':p.stat().st_size,'sha256':digest(p) if item['sha256'] is not None else None}
 preserved[relative]={'expected':item,'observed':observed,'pass':item==observed}
refs={ref:{'expected':value,'observed':git(original,'rev-parse',ref)} for ref,value in base['refs'].items()}
for item in refs.values():item['pass']=item['expected']==item['observed']
p={'files':preserved,'refs':refs,'all_preserved':all(v['pass'] for v in preserved.values()) and all(v['pass'] for v in refs.values()),'large_original_core':'size checked; no huge core read or changed'}
assert p['all_preserved'];(root/'preservation_check.json').write_text(json.dumps(p,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='source_hashes'},indent=2));print('original state preserved',p['all_preserved'])
