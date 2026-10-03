import hashlib,json,shutil,subprocess,sys
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');temp=Path('/tmp/rm_surface_reveal_initial');replay=Path('/tmp/rm_surface_reveal_reexecution')
out=repo/'docs/dynamic_obstacle_critic/stage2_evidence/surface_reveal_preflight';out.mkdir()
checkpoint=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip();sources={}
files=subprocess.check_output(['git','ls-files','src/rm_dynamic_obstacle_critic/tools','docs/dynamic_obstacle_critic/dynamic_surface_reveal_experiment.md','docs/dynamic_obstacle_critic/stage2_progress.md','docs/contracts/tf_contract_2027.md','docs/development_workflow.md','docs/project_structure_and_documentation_index.md','src/rm_dynamic_obstacle_critic/README.md'],cwd=repo,text=True).splitlines()
for name in files:
    data=subprocess.check_output(['git','show',checkpoint+':'+name],cwd=repo);target=out/'source'/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data);sources[name]=hashlib.sha256(data).hexdigest()
for source,prefix in [(temp,'initial_execution'),(replay,'independent_reexecution')]:
    for p in source.iterdir():
        if p.is_file():target=out/prefix/p.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
for p in [Path('/tmp/reveal_visible_initial.json'),Path('/tmp/reveal_visible_initial.log'),Path('/tmp/reveal_unit_initial.log'),Path('/tmp/reveal_brake_units.log'),Path('/tmp/reveal_brake_units_final.log'),Path('/tmp/reveal_brake_units_checkpoint.log'),Path('/tmp/run_reveal_pair.py'),Path('/tmp/rm_surface_reveal_pair_driver.log'),Path('/tmp/rm_surface_reveal_reexecution_driver.log'),Path('/tmp/rm_surface_reveal_plot_driver.log')]:
    target=out/'verification'/p.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
(out/'source_identity.json').write_text(json.dumps({'checkpoint':checkpoint,'files':sources},indent=2)+'\n')
(out/'audit_identity.json').write_text(json.dumps({'source_checkpoint':checkpoint,'source_files':len(sources),'inputs':{'visible':'3199b3b3c93ee7ba89b3b23a01b4ecb44a21ebd5c10b9b11c94e71fb59bb7d84','nominal':'9317daadf03cf11fb2b13110c633b8cce127f9943c0db8310f4d2bf47aa2cf5f'},'unit_checks':18,'independent_report_figure_checks':8,'python':sys.version,'matplotlib':__import__('matplotlib').__version__,'limits':'Early partial initial report predates added contact view and is not the final source-reexecution baseline. Relative-path failure retained and repaired. Guaranteed real braking/response and C trial remain unknown; large original trials are referenced by full manifests.'},indent=2)+'\n')
shutil.copyfile(Path(__file__),out/'verification/freeze_surface_audits.py')
files={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()};size=sum((out/p).stat().st_size for p in files)
(out/'manifest.json').write_text(json.dumps({'scope':'Source-time reveal proxy and empirical simulated stop observations; no certified stopping or moving-surface trial','source_checkpoint':checkpoint,'files':files},indent=2)+'\n')
assert all(hashlib.sha256((out/name).read_bytes()).hexdigest()==sha for name,sha in files.items())
print(json.dumps({'files':len(files),'bytes_excluding_manifest':size,'manifest_sha256':hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest()}))
