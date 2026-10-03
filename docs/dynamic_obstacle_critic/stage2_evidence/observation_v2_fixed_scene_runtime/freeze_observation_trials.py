"""Freeze regular evidence; never traverse corruption-probe source links."""
import gzip,hashlib,json,os,shutil,subprocess
from pathlib import Path
repo=Path('/home/qihei/rm2027_navigation');temp=Path('/tmp/rm_observation_diameter_gazebo')
base=repo/'docs/dynamic_obstacle_critic/stage2_evidence';summaries={}
post=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
runtime=json.loads((temp/'source_identity.json').read_text())

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def copy_tree(source,out,compress=False):
    transfers={};links={}
    for folder,dirs,files in os.walk(source,followlinks=False):
        folder=Path(folder)
        for name in list(dirs):
            p=folder/name
            if p.is_symlink():links[str(p.relative_to(source))]=os.readlink(p);dirs.remove(name)
            elif name in ('__pycache__','.pytest_cache'):dirs.remove(name)
        for name in sorted(files):
            p=folder/name;relative=p.relative_to(source)
            if p.is_symlink():links[str(relative)]=os.readlink(p);continue
            if not p.is_file():continue
            zipped=compress and (p.suffix in ('.bin','.jsonl') or p.name=='launch.log')
            target=out/(str(relative)+('.gz' if zipped else ''));target.parent.mkdir(parents=True,exist_ok=True)
            before=digest(p)
            if zipped:
                with p.open('rb') as src,target.open('xb') as dest:
                    with gzip.GzipFile(filename='',mode='wb',fileobj=dest,mtime=0) as z:shutil.copyfileobj(src,z)
            else:shutil.copyfile(p,target)
            if digest(p)!=before:raise ValueError('source changed while freezing '+str(p))
            transfers[str(relative)]={'archived_path':str(target.relative_to(out)),'original_sha256':before,'original_bytes':p.stat().st_size,'encoding':'gzip-mtime0' if zipped else 'identity'}
    return transfers,links

def finish(out,scope):
    files={str(p.relative_to(out)):digest(p) for p in sorted(out.rglob('*')) if p.is_file()}
    (out/'manifest.json').write_text(json.dumps({'scope':scope,'runtime_source_checkpoint':runtime['commit'],'post_trial_auditor_checkpoint':post,'files':files},indent=2)+'\n')
    # A second full pass checks both digests and the enumerated file set.
    assert set(files)=={str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name!='manifest.json'}
    assert all(digest(out/name)==sha for name,sha in files.items())
    result={'files':len(files),'bytes_excluding_manifest':sum((out/name).stat().st_size for name in files),'manifest_sha256':digest(out/'manifest.json')}
    summaries[out.name]=result;print(out.name,json.dumps(result),flush=True)

shared=base/'observation_v2_fixed_scene_runtime';shared.mkdir()
for relative,sha in runtime['files'].items():
    source=temp/'source'/relative
    data=subprocess.check_output(['git','show',runtime['commit']+':'+relative],cwd=repo)
    assert hashlib.sha256(data).hexdigest()==sha and digest(source)==sha
    target=shared/'runtime_source'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
copy_tree(repo/'src/rm_dynamic_obstacle_critic/tools',shared/'auditors')
for p in sorted(temp.iterdir()):
    if p.is_file():shutil.copyfile(p,shared/p.name)
for folder in ('runtime_buildlogs','runtime_build'):
    source=temp/folder
    if source.exists():
        for p in source.rglob('*'):
            if p.is_file() and not p.is_symlink() and (folder.endswith('logs') or p.name in ('flags.make','link.txt','compile_commands.json','CMakeCache.txt')):
                target=shared/'verification'/folder/p.relative_to(source);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
pre=Path('/tmp/rm_observation_diameter_replay/build/rm_dynamic_obstacle_critic')
helpers={}
for name in ('replay_observation_consumption','replay_native_optimizer','native_safe_control_witness','evaluate_observation_processed_cv'):
    helpers[name]=digest(pre/name)
    for leaf in ('flags.make','link.txt'):
        p=pre/'CMakeFiles'/(name+'.dir')/leaf;target=shared/'verification/sdk_build'/(name+'_'+leaf);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
(shared/'archive_identity.json').write_text(json.dumps({'runtime_source_checkpoint':runtime['commit'],'runtime_source_files_verified':len(runtime['files']),'post_trial_auditor_checkpoint':post,'sdk_helper_elf_sha256':helpers,'image_identity':'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3','runtime_preflight_manifest_sha256':'7f24451295349607ff7e9529e9a77666dfea471284a4adb665e9b4704919473d','scope':'Frozen eight-package runtime sources and actual install identities; offline auditors separated; failures retained; no acceptance'},indent=2)+'\n')
finish(shared,'Frozen paired v2 runtime/source/helper closure; both task verdicts FAILED')
for mode in ('visible','nominal'):
    name='gazebo_observation_v2_'+mode;source=temp/name;out=base/name;out.mkdir()
    transfers,links=copy_tree(source,out,True)
    (out/'compression_identity.json').write_text(json.dumps(transfers,indent=2)+'\n')
    (out/'source_link_references.json').write_text(json.dumps({'scope':'Corruption probes own only regular changed files. Linked originals are logical read-only sources, not traversed or copied. Recreate probes against a fresh decoded working copy.','links':links},indent=2)+'\n')
    (out/'archive_identity.json').write_text(json.dumps({'mode':mode,'runtime_source_checkpoint':runtime['commit'],'post_trial_auditor_checkpoint':post,'shared_runtime_archive':shared.name,'shared_runtime_manifest_sha256':summaries[shared.name]['manifest_sha256'],'physical_raw_task_overall':'FAILED','scope':'Complete original regular trial files, changed corruption inputs and reference metadata; missing after/context remains UNKNOWN; full 3s processed path witnesses pending'},indent=2)+'\n')
    (out/'replay_policy.json').write_text(json.dumps({'association_window_seconds':.05,'plot_title':f'V2 {mode}: fixed-scene task FAILED','cost_plot_title':f'V2 {mode}: observed costs; fixed-scene task FAILED','scope':'Retain original fail gates; fresh SDK outputs and manifest-backed inputs required'},indent=2)+'\n')
    finish(out,'Paired v2 '+mode+' fixed-scene regression: task FAILED; future support not established')
(temp/'frozen_trials_summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
