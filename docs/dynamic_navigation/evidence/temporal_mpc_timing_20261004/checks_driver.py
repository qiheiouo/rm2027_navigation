import json,xml.etree.ElementTree as ET
from pathlib import Path
work=Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004');root=work/'build/temporal_mpc_timing_20261004'
tests={}
for environment in ['host','humble']:
 paths=[root/f'tests_{environment}.xml',root/f'timing_audit_{environment}.xml']
 cases=[c for p in paths for c in ET.parse(p).getroot().iter('testcase')]
 ids={(c.get('classname'),c.get('name')) for c in cases}
 tests[environment]=dict(unique_test_cases=len(ids),failures=sum(c.find('failure') is not None or c.find('error') is not None for c in cases),skipped=sum(c.find('skipped') is not None for c in cases))
 assert len(ids)==130 and not tests[environment]['failures'] and not tests[environment]['skipped']
clock=json.loads((root/'worker_clock_01/summary.json').read_text());nav2=json.loads((root/'nav2_portfolio_01/summary.json').read_text())
assert clock['pass_all'] and nav2['all_engineering_gates_pass']
runs={}
for name in ['head_on_b0_01','head_on_portfolio_01']:
 load=lambda f:json.loads((root/name/f).read_text())
 capture=load('ros_capture_audit.json');execution=load('execution_audit.json');inputs=load('reanchor_inputs_audit.json');timing=load('worker_timing_audit.json');age=load('proposal_age_audit.json')
 valid=bool(not capture['serialized_json_mismatches'] and not capture['source_tf_replay_failures'] and execution['all_model_replays_pass'] and not inputs['errors'] and not inputs['missing'] and inputs['actual_native_margins_match'] and not timing['errors'] and age['all_actual_replays_match'] and not age['errors'])
 assert valid
 runs[name]=dict(engineering_replays_pass=valid,cdr=capture['serialized_messages_checked'],source_tf=capture['source_tf_replay_available'],guard_replays=execution['guard_exact_input_replays'],native_state_and_input_joins=inputs['worker_input_joins'],dynamic_margin_and_age_replays=len(age['records']),request_timing_joins=timing['native_request_joins'],proposal_receipt_joins=timing['native_proposal_receipt_joins'],clock_statuses=timing['clock_statuses'])
pair=json.loads((root/'paired_summary.json').read_text())['pairs'][0]
analysis=json.loads((root/'timing_analysis.json').read_text())
result=dict(tests=tests,real_clock_checks=len(clock['checks']),nav2_fault_cases=len(nav2['fault_cases']),all_engineering_and_input_replays_pass=True,
 runs=runs,totals={k:sum(d[k] for d in runs.values()) for k in ['cdr','source_tf','guard_replays','native_state_and_input_joins','dynamic_margin_and_age_replays','request_timing_joins','proposal_receipt_joins']},
 observed_physical_continuity=dict(b0_controller=pair['b0']['logged_controller_75ms_continuity_gate'],b0_final_cmd=pair['b0']['logged_actuator_75ms_continuity_gate'],candidate_controller=pair['candidate']['logged_controller_75ms_continuity_gate'],candidate_final_cmd=pair['candidate']['logged_actuator_75ms_continuity_gate']),
 observed_worker_core_40ms_gate={n:d['core_40ms_rejections']==0 for n,d in analysis.items()},
 physical_acceptance=False,dynamic_acceptance=False,candidate_frozen_for_deployment=False,
 scope='Engineering and replay validity explicitly separate from physical success, continuous safety and real-time acceptance. Retain failed continuity and core deadline observations.')
(root/'phase_checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
