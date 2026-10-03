import gzip, hashlib, json, subprocess, sys
from pathlib import Path

repo = Path(__file__).resolve().parents[2]
work = Path(__file__).parent
out = work / 'actual_sdk_reexecution'
out.mkdir(exist_ok=False)
tools = repo / 'docs/dynamic_obstacle_critic/stage2_evidence/surface_reveal_preflight/source/src/rm_dynamic_obstacle_critic/tools'
checks = {}
for mode in ('visible', 'nominal'):
    trial = repo / 'docs/dynamic_obstacle_critic/stage2_evidence' / ('gazebo_observation_v2_' + mode)
    dest = out / mode
    dest.mkdir()
    for script, args, name in [
        ('prepare_dynamic_consumption_replay.py', [trial / 'dynamic_scores', dest / 'dynamic'], 'prepare_dynamic'),
        ('prepare_native_optimizer_replay.py', [trial, dest / 'optimizer', '--association-window', '.05'], 'prepare_optimizer'),
        ('prepare_native_witness.py', [trial, dest / 'witness'], 'prepare_witness'),
    ]:
        with (dest / (name + '.log')).open('x') as log:
            subprocess.run([sys.executable, '-B', str(tools / script), *map(str, args)], stdout=log, stderr=subprocess.STDOUT, check=True)
    for part, original in [('dynamic', 'dynamic_replay'), ('optimizer', 'native_optimizer')]:
        if (dest / part / 'input.bin').read_bytes() != gzip.decompress((trial / original / 'input.bin.gz').read_bytes()):
            raise ValueError(mode + ' prepared ' + part + ' input differs')

commands = ['set -e', 'source /opt/ros/humble/setup.bash', 'source /sdk/install/setup.bash']
for mode in ('visible', 'nominal'):
    prefix = '/out/' + mode
    helper = '/sdk/build/rm_dynamic_obstacle_critic/'
    commands.append(f'{helper}replay_observation_consumption {prefix}/dynamic/input.bin {prefix}/dynamic/risk.jsonl')
    for name, flag in [('sdk_final_output', ''), ('zero_history_output', '--zero-history-each-cycle'), ('weights_output', '--weights-audit'), ('uniform_output', '--uniform-weight-probe')]:
        commands.append(f'{helper}replay_native_optimizer {prefix}/optimizer/input.bin {prefix}/optimizer/{name}.jsonl {flag}')
    commands.append(f'{helper}native_safe_control_witness {prefix}/witness/input.bin {prefix}/witness/output.bin {prefix}/witness/output.jsonl')
(out / 'run_helpers.sh').write_text('\n'.join(commands) + '\n')
with (out / 'docker_driver.log').open('x') as log:
    subprocess.run(['docker', 'run', '--rm', '--network', 'none', '--ulimit', 'core=0', '--user', f'{__import__("os").getuid()}:{__import__("os").getgid()}', '-v', f'{work / "sdk_build"}:/sdk:ro', '-v', f'{out}:/out', 'sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3', 'bash', '/out/run_helpers.sh'], stdout=log, stderr=subprocess.STDOUT, check=True)
for mode in ('visible', 'nominal'):
    trial = repo / 'docs/dynamic_obstacle_critic/stage2_evidence' / ('gazebo_observation_v2_' + mode)
    dest = out / mode
    checks[mode + ' risk_exact'] = (dest / 'dynamic/risk.jsonl').read_bytes() == gzip.decompress((trial / 'dynamic_replay/risk.jsonl.gz').read_bytes())
    for name in ('sdk_final_output', 'zero_history_output', 'weights_output', 'uniform_output'):
        checks[mode + ' ' + name + '_exact'] = (dest / 'optimizer' / (name + '.jsonl')).read_bytes() == gzip.decompress((trial / 'native_optimizer' / (name + '.jsonl.gz')).read_bytes())
    witness = [json.loads(line) for line in (dest / 'witness/output.jsonl').read_text().splitlines()][1:]
    identity = json.loads((dest / 'witness/input_identity.json').read_text())
    checks[mode + ' all_witness_integrity'] = all(r['raw_velocity_pose_exact'] and r['aggregate_SG_exact'] and r['actual_command_double_bit_exact'] for r in witness)
    checks[mode + ' witness_complete'] = len(witness) == identity['cycles'] == identity['source_cycles']
helpers = {name: hashlib.sha256((work / 'sdk_build/build/rm_dynamic_obstacle_critic' / name).read_bytes()).hexdigest() for name in ('replay_observation_consumption', 'replay_native_optimizer', 'native_safe_control_witness')}
report = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'checks': checks, 'helper_elf_sha256': helpers, 'scope': 'Fresh frozen-source actual SDK execution; raw states, risk, costs, histories, SG and MPPI returned command exact. Geometry/control acceptance separate.'}
(out / 'sdk_reexecution.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report), flush=True)
if not all(checks.values()):
    raise SystemExit(1)
for mode in ('visible', 'nominal'):
    trial = repo / 'docs/dynamic_obstacle_critic/stage2_evidence' / ('gazebo_observation_v2_' + mode)
    witness = out / mode / 'witness'
    with (witness / 'geometry.log').open('x') as log:
        subprocess.run([sys.executable, '-B', str(tools / 'audit_observation_native_witness.py'), str(trial), str(witness / 'output.bin'), str(witness / 'input_identity.json'), str(out / 'sdk_reexecution.json'), str(witness / 'geometry.json'), '--mode', mode], stdout=log, stderr=subprocess.STDOUT, check=True)
    print(mode + ' full 3s conditional geometry complete', flush=True)
