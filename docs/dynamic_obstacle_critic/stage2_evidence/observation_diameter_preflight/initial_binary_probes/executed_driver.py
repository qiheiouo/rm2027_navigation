from pathlib import Path
import hashlib,json,struct,subprocess
root=Path('/out')
tools=root/'build/rm_dynamic_obstacle_critic'
inputs=root/'diameter_replay_inputs'
output=root/'diameter_binary_probes';output.mkdir(exist_ok=False)
nominal=(inputs/'V2NominalActualScoreAndBudgetArePreserved/input.bin').read_bytes()
v1=Path('/checks/old_v1_replay/input.bin')
rows=[]
def probe(label, executable, data, expected):
    path=output/(label+'.bin');path.write_bytes(data)
    target=output/(label+'.jsonl')
    result=subprocess.run([str(tools/executable),str(path),str(target)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (output/(label+'.log')).write_bytes(result.stdout)
    rows.append(dict(case=label,returncode=result.returncode,expected=expected,passed=result.returncode==expected,input_sha256=hashlib.sha256(data).hexdigest()))
probe('v1_helper_refuses_v2','replay_dynamic_consumption',nominal,1)
probe('v2_helper_refuses_v1','replay_observation_consumption',v1.read_bytes(),1)
changed=bytearray(nominal);struct.pack_into('<I',changed,112,2)
probe('unknown_radius_enum','replay_observation_consumption',bytes(changed),1)
changed=bytearray(nominal);struct.pack_into('<d',changed,116,.4)
probe('half_diameter_cannot_match_actual_nominal_risk','replay_observation_consumption',bytes(changed),2)
changed=bytearray(nominal);struct.pack_into('<I',changed,124,4097)
probe('oversize_prior','replay_observation_consumption',bytes(changed),1)
probe('truncated_input','replay_observation_consumption',nominal[:-1],1)
probe('trailing_input','replay_observation_consumption',nominal+b'\x00',1)
report=dict(status='PASS' if all(row['passed'] for row in rows) else 'FAILED',cases=rows,scope='Actual helper protocol and nominal-risk mismatch probes only; not physical negative trials.')
(output/'summary.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
print(json.dumps(report,sort_keys=True));raise SystemExit(0 if report['status']=='PASS' else 1)
