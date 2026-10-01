import hashlib,json,shutil,sys
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');tools=repo/'src/rm_dynamic_obstacle_critic/tools';sys.path.insert(0,str(tools))
from analyze_trial import analyze
from audit_contact_witness import analyze as contact
from audit_mechanical_footprint import analyze as mechanical
root=repo/'docs/dynamic_obstacle_critic/stage2_evidence';checks=[]
for name in ['gazebo_soft_map_clearance','gazebo_soft_clearance_performance']:
 p=root/name
 for filename,func in [('analysis.json',analyze),('contact_audit.json',contact),('mechanical_footprint_audit.json',mechanical)]:
  expected=json.loads((p/filename).read_text());actual=json.loads(json.dumps(func(p)));assert actual==expected,(name,filename)
  checks.append({'trial':name,'report':filename,'semantic_exact':True})
 pmanifest=json.loads((p/'manifest.json').read_text())
 for fn,digest in pmanifest['files'].items():assert hashlib.sha256((p/fn).read_bytes()).hexdigest()==digest,(name,fn)
checks.append({'all_historical_manifests_unchanged':True})
probe=Path('/tmp/rm_dynamic_critic_v1/mechanical_policy_probe');shutil.copytree(root/'gazebo_soft_clearance_performance',probe,dirs_exist_ok=True)
policy=json.loads((probe/'policy.json').read_text());policy['full_mechanical_body_required']=True;(probe/'policy.json').write_text(json.dumps(policy))
shutil.copyfile(repo/'src/rm_dynamic_obstacle_critic/config/nav2_cv_mechanical_footprint.yaml',probe/'profile.yaml')
shutil.copyfile(repo/'src/rm_dynamic_obstacle_critic/config/guard_mechanical_footprint.yaml',probe/'installed_inputs/guard.yaml')
actual=json.loads(json.dumps(analyze(probe)));original=json.loads((root/'gazebo_soft_clearance_performance/analysis.json').read_text())
assert actual['geometry']['body']==original['geometry']['body']
assert not actual['gates']['full_mechanical_body_clearance']
assert actual['mechanical_geometry']['static_linear_interpolation_bound']>.05
(probe/'mechanical_footprint_audit.json').write_text(json.dumps(actual['mechanical_geometry']))
checks.append({'offline_policy_probe_only':True,'physical_base_report_retained_exact':True,'full_mechanical_gate':actual['gates']['full_mechanical_body_clearance'],'mechanical_static_bound':actual['mechanical_geometry']['static_linear_interpolation_bound'],'mechanical_dynamic_bound':actual['mechanical_geometry']['dynamic_linear_interpolation_bound'],'verdict':actual['verdict']})
out=Path('/tmp/rm_dynamic_critic_v1/mechanical_auditor_regression.json');out.write_text(json.dumps(checks,indent=2)+'\n');print(out.read_text())
