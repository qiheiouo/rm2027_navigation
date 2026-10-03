from pathlib import Path
import hashlib,shlex,subprocess,json
repo=Path('/repo');root=repo/'docs/dynamic_obstacle_critic/stage2_evidence/native_factor_aggregation/verification';base='evaluate_native_factor_aggregation'
flags={}
for line in (root/(base+'_flags.make')).read_text().splitlines():
    if line.startswith('CXX_'):key,value=line.split(' = ',1);flags[key]=shlex.split(value)
assert not any('RM_' in x for x in flags['CXX_DEFINES'])
build=Path('/out/build/rm_dynamic_obstacle_critic');build.mkdir(parents=True,exist_ok=True);ids={}
for target in ['native_safe_control_witness','native_individual_stages']:
    definitions=flags['CXX_DEFINES']+(['-DRM_NATIVE_INDIVIDUAL_STAGES=1'] if target=='native_individual_stages' else [])
    obj=Path('CMakeFiles')/f'{target}.dir/tools/native_safe_control_witness.cpp.o';(build/obj.parent).mkdir(parents=True,exist_ok=True)
    command=['/usr/bin/c++',*definitions,*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-MD','-MT',str(obj),'-MF',str(obj)+'.d','-o',str(obj),'-c',str(repo/'src/rm_dynamic_obstacle_critic/tools/native_safe_control_witness.cpp')]
    subprocess.run(command,cwd=build,check=True)
    link=shlex.split((root/(base+'_link.txt')).read_text());link=[str(obj) if x==f'CMakeFiles/{base}.dir/tools/{base}.cpp.o' else target if x==base else x for x in link]
    subprocess.run(link,cwd=build,check=True)
    ids[target]=hashlib.sha256((build/target).read_bytes()).hexdigest()
    (Path('/out')/(target+'_compile_command.json')).write_text(json.dumps(command,indent=2)+'\n')
    (Path('/out')/(target+'_link_command.json')).write_text(json.dumps(link,indent=2)+'\n')
Path('/out/binary_identities.json').write_text(json.dumps(ids,indent=2)+'\n');print(json.dumps(ids),flush=True)
