from pathlib import Path
import hashlib
import json
import shutil
import subprocess

repo = Path('/home/qihei/rm2027_navigation')
work = Path('/tmp/rm_observation_anchor_preflight')
target = repo/'docs/dynamic_obstacle_critic/stage2_evidence/observation_anchor_preflight'
target.mkdir(exist_ok=False)
def copy(source, name):
    destination = target/name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)
for name in subprocess.check_output(['git', 'ls-files', 'src/rm_competition_interfaces',
        'src/rm_dynamic_obstacle_tracking', 'src/rm_dynamic_obstacle_critic'], cwd=repo, text=True).splitlines():
    copy(repo/name, 'source/'+name)
for name in ['docs/dynamic_obstacle_critic/observation_anchor_cv_experiment.md',
             'docs/contracts/dynamic_obstacle_geometry_contract_2027.md',
             'docs/contracts/topic_contract_2027.md', 'docs/contracts/tf_contract_2027.md',
             'docs/runtime_profiles.md', 'docs/development_workflow.md']:
    copy(repo/name, 'source/'+name)
for name in ['core_41e96399.py', 'node_41e96399.py']:
    copy(work/name, 'reference/'+name)
for path in sorted(work.glob('*.log')):
    copy(path, 'verification/'+path.name)
for path in sorted(work.glob('*.xml')):
    copy(path, 'verification/'+path.name)
copy(work/'integer_default_core_regression.json', 'verification/default_core_regression.json')
copy(work/'default_wire_regression.json', 'verification/default_wire_regression.json')
copy(work/'environment.json', 'verification/environment.json')
for name in ['initial_ros_fixture_failure', 'initial_guard_fixture_failure', 'initial_wire_failure',
             'default_wire_regression.raw', 'default_guard', 'observation_guard', 'epoch_guard']:
    for path in sorted((work/name).rglob('*')):
        if path.is_file(): copy(path, 'verification/'+str(path.relative_to(work)))
for path in sorted((work/'integer_ros_fixtures').rglob('*')):
    if path.is_file(): copy(path, 'verification/ros_fixtures/'+str(path.relative_to(work/'integer_ros_fixtures')))
for path in sorted((work/'build/rm_dynamic_obstacle_critic/test_results').rglob('*.xml')):
    copy(path, 'verification/native_xml/'+path.name)
for path in sorted((work/'build/rm_dynamic_obstacle_critic/CMakeFiles').rglob('*')):
    if path.name in ('flags.make', 'link.txt'):
        copy(path, 'verification/build_flags/'+str(path.relative_to(work/'build/rm_dynamic_obstacle_critic/CMakeFiles')))
copy(Path(__file__), 'verification/freeze.py')
runtime = {}
for name in ['lib/librm_dynamic_obstacle_critic.so', 'lib/librm_native_cost_accumulator_observer.so',
             'lib/rm_dynamic_obstacle_critic/dynamic_safety_guard',
             'lib/rm_dynamic_obstacle_critic/native_command_observer']:
    name = 'rm_dynamic_obstacle_critic/'+name
    runtime[name] = hashlib.sha256((work/'install'/name).read_bytes()).hexdigest()
for name in ['core.py', 'dynamic_obstacle_tracker_node.py']:
    relative = 'rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/'+name
    installed = work/'install/rm_dynamic_obstacle_tracking/lib/python3.10/site-packages/rm_dynamic_obstacle_tracking'/name
    assert installed.read_bytes() == (repo/'src'/relative).read_bytes()
def revision(name):
    return subprocess.check_output(['git', 'rev-parse', name], cwd=repo, text=True).strip()
assert revision('HEAD').startswith('14a015b5')
assert revision('main') == 'd735ee12bd950dca0e691cdf2f2c61f35cef8ffc'
assert revision('codex/dynamic-differential-risk') == 'e680b1430db6efd8dc601d5585ee207e5f9b42a4'
provenance = dict(schema=1, implementation_commit=revision('HEAD'), reference_commit='41e96399',
    preregistration_commit='356df7ca', sdk_image='sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
    native_tests=53, tracker_tests=54, default_guard_tests=7, observation_guard_tests=7,
    source_math_tests=8, observation_math_tests=6, runtime_elf_sha256=runtime,
    main_commit=revision('main'), research_commit=revision('codex/dynamic-differential-risk'),
    frozen_research_tag=revision('research/dynamic-differential-risk-frozen-20261001^{}'),
    scope='Interface preflight only; no simulation or hardware trial, full physical geometry or goal acceptance. Integer ROS epoch tests and all initial fixture/CDR padding failures retained.')
(target/'provenance.json').write_text(json.dumps(provenance, indent=2, sort_keys=True)+'\n')
files = {str(path.relative_to(target)): hashlib.sha256(path.read_bytes()).hexdigest()
         for path in sorted(target.rglob('*')) if path.is_file()}
(target/'manifest.json').write_text(json.dumps(dict(schema=1, files=files, scope=provenance['scope']),
                                             indent=2, sort_keys=True)+'\n')
print(json.dumps(dict(files=len(files), bytes=sum((target/name).stat().st_size for name in files),
    manifest_sha256=hashlib.sha256((target/'manifest.json').read_bytes()).hexdigest(), runtime_elf_sha256=runtime), sort_keys=True))
