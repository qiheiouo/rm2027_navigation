import gzip, hashlib, json, subprocess, sys
from pathlib import Path

repo = Path(__file__).resolve().parents[2]
work = Path(__file__).parent
out = work / 'actual_sdk_reexecution'
tools = repo / 'docs/dynamic_obstacle_critic/stage2_evidence/surface_reveal_preflight/source/src/rm_dynamic_obstacle_critic/tools'
checks = {}
for mode in ('visible', 'nominal'):
    trial = repo / 'docs/dynamic_obstacle_critic/stage2_evidence' / ('gazebo_observation_v2_' + mode)
    dest = out / mode
    checks[mode + ' risk_exact'] = (dest / 'dynamic/risk.jsonl').read_bytes() == gzip.decompress((trial / 'dynamic_replay/native_risk.jsonl.gz').read_bytes())
    for name in ('sdk_final_output', 'zero_history_output', 'weights_output', 'uniform_output'):
        checks[mode + ' ' + name + '_exact'] = (dest / 'optimizer' / (name + '.jsonl')).read_bytes() == gzip.decompress((trial / 'native_optimizer' / (name + '.jsonl.gz')).read_bytes())
    witness = [json.loads(line) for line in (dest / 'witness/output.jsonl').read_text().splitlines()][1:]
    identity = json.loads((dest / 'witness/input_identity.json').read_text())
    checks[mode + ' all_witness_integrity'] = all(r['raw_velocity_pose_exact_mask'] == 63 and r['aggregate_SG_exact'] and r['actual_command_double_bit_exact'] for r in witness)
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
