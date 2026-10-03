from pathlib import Path
import hashlib, json, shutil, subprocess, xml.etree.ElementTree as ET
repo=Path('/home/qihei/rm2027_navigation');work=Path('/tmp/rm_observation_anchor_portability')
base=repo/'docs/dynamic_obstacle_critic/stage2_evidence/observation_anchor_preflight'
target=repo/'docs/dynamic_obstacle_critic/stage2_evidence/observation_anchor_portability'
target.mkdir(exist_ok=False)
def copy(p,name):
    q=target/name;q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,q)
for name in ['build.log','tests.log','test_result.log','source_identity.json']:
    copy(work/name,'verification/'+name)
for name in ['src/rm_dynamic_obstacle_critic/CMakeLists.txt','src/rm_dynamic_obstacle_critic/test/test_model.cpp']:
    copy(repo/name,'amended_source/'+name)
    assert (repo/name).read_bytes()==(work/'source'/name).read_bytes()
    copy(repo/name,'verification/current_'+Path(name).name)
identity=json.loads((work/'source_identity.json').read_text())['source_sha256']
original=json.loads((base/'manifest.json').read_text())['files']
changed=[name for name,digest in identity.items() if digest!=original['source/'+name]]
assert sorted(changed)==['src/rm_dynamic_obstacle_critic/CMakeLists.txt','src/rm_dynamic_obstacle_critic/test/test_model.cpp']
prefix=b'// Generated test namespace adapter; frozen body follows unchanged.\n'
generated=work/'build/rm_dynamic_obstacle_critic/test_reference/static_map_2622371.hpp'
frozen=repo/'src/rm_dynamic_obstacle_critic/test/reference/static_map_2622371.hpp'
assert generated.read_bytes()==prefix+frozen.read_bytes()
copy(generated,'verification/generated_static_map_2622371.hpp')
runtime=json.loads((base/'provenance.json').read_text())['runtime_elf_sha256']
checks={name:hashlib.sha256((work/'install'/name).read_bytes()).hexdigest()==digest for name,digest in runtime.items()}
assert all(checks.values())
count=0
for p in sorted((work/'build/rm_dynamic_obstacle_critic/test_results').rglob('*.xml')):
    root=ET.parse(p).getroot();assert root.get('failures','0')=='0' and root.get('errors','0')=='0'
    count+=int(root.get('tests','0'));copy(p,'verification/native_xml/'+p.name)
assert count==53
for name in ['test_model','rm_dynamic_obstacle_critic','dynamic_safety_guard']:
    p=work/'build/rm_dynamic_obstacle_critic/CMakeFiles'/(name+'.dir')/'flags.make'
    copy(p,'verification/build_flags/'+name+'.flags.make')
copy(Path(__file__),'verification/freeze.py')
provenance=dict(schema=1,status='PASS',commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),source_archive='../observation_anchor_preflight',source_manifest_sha256=hashlib.sha256((base/'manifest.json').read_bytes()).hexdigest(),source_file_count=len(identity),source_mtime_ns=1700000000000000000,source_byte_changes=changed,native_tests=count,generated_frozen_body_byte_exact=True,runtime_elf_matches=checks,sdk_image='sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',scope='Same-mtime clean-source test build portability; frozen body and four runtime ELFs unchanged. Physical/task failures remain.')
(target/'provenance.json').write_text(json.dumps(provenance,indent=2,sort_keys=True)+'\n')
files={str(p.relative_to(target)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(target.rglob('*')) if p.is_file()}
(target/'manifest.json').write_text(json.dumps(dict(schema=1,files=files,scope=provenance['scope']),indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(files=len(files),bytes=sum((target/name).stat().st_size for name in files),manifest_sha256=hashlib.sha256((target/'manifest.json').read_bytes()).hexdigest(),runtime_elf_matches=checks),sort_keys=True))
