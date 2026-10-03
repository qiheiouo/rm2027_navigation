from pathlib import Path
import hashlib,shlex,subprocess,json
root=Path('/repo/docs/dynamic_obstacle_critic/stage2_evidence/native_raw_map_contract')
source=Path('/repo/src/rm_dynamic_obstacle_critic/tools/native_cost_map_queries.cpp')
expected=json.loads((root/'manifest.json').read_text())['files']['source/tools/native_cost_map_queries.cpp']
assert hashlib.sha256(source.read_bytes()).hexdigest()==expected
build=Path('/out/build/rm_dynamic_obstacle_critic');build.mkdir(parents=True,exist_ok=True)
identities={}
for target in ['native_cost_map_queries','native_raw_map_queries']:
    flags={}
    for line in (root/'verification'/f'{target}_flags.make').read_text().splitlines():
        if line.startswith('CXX_'):key,value=line.split(' = ',1);flags[key]=shlex.split(value)
    obj=Path('CMakeFiles')/f'{target}.dir/tools/native_cost_map_queries.cpp.o'
    (build/obj.parent).mkdir(parents=True,exist_ok=True)
    compile_cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-MD','-MT',str(obj),'-MF',str(obj)+'.d','-o',str(obj),'-c',str(source)]
    subprocess.run(compile_cmd,cwd=build,check=True)
    subprocess.run(shlex.split((root/'verification'/f'{target}_link.txt').read_text()),cwd=build,check=True)
    identities[target]=hashlib.sha256((build/target).read_bytes()).hexdigest()
for name in ['libnav2_costmap_2d_core.so','libmppi_controller.so','libmppi_critics.so']:
    identities[name]=hashlib.sha256((Path('/opt/ros/humble/lib')/name).read_bytes()).hexdigest()
Path('/out/rebuilt_identities.json').write_text(json.dumps(identities,indent=2)+'\n');print(json.dumps(identities),flush=True)
