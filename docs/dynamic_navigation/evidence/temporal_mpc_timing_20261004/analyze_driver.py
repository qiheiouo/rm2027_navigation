import json,sys
from pathlib import Path
import numpy as np
work=Path('/home/qihei/rm2027_navigation/build/temporal_mpc_main_20261004');root=work/'build/temporal_mpc_timing_20261004'
sys.path.insert(0,str(work/'experiments/temporal_mpc/gazebo'))
from audit_run import stats
output={}
for name in ['head_on_b0_01','head_on_portfolio_01']:
 es=[json.loads(line) for line in (root/name/'events.jsonl').open()]
 def topic(t):return [e for e in es if e['topic']==t]
 timings=[json.loads(e['data']['data']) for e in topic('/temporal_mpc/worker_timing')]
 requests=[t for t in timings if t['kind']=='request']
 diags=[json.loads(e['data']['data']) for e in topic('/temporal_mpc/solver_diagnostic')]
 h=[json.loads(e['data']['data']) for e in topic('/temporal_mpc/execution_health')]
 commands=[e for e in topic('/cmd_vel') if 10<=e['receipt_sim_ns']/1e9<=50]
 left,right=max(zip(commands,commands[1:]),key=lambda pair:pair[1]['receipt_monotonic_ns']-pair[0]['receipt_monotonic_ns'])
 near=[d for d in h if left['receipt_monotonic_ns']-100_000_000<=d['producer_output_ns']<=right['receipt_monotonic_ns']+100_000_000]
 age=json.loads((root/name/'proposal_age_audit.json').read_text());reanchor=json.loads((root/name/'reanchor_inputs_audit.json').read_text())
 data=dict(request_signed_age_s=stats([r['request_signed_age_ns']*1e-9 for r in requests if 'request_signed_age_ns' in r]),
  proposal_callback_body_exceeds_40ms=sum((r['callback_body_end_monotonic_ns']-r['callback_start_monotonic_ns'])>40_000_000 for r in requests),
  core_40ms_rejections=sum(d['reason']=='worker cycle deadline' for d in diags),
  worker_max_iterations=max(d['iterations'] for d in diags),solver_s=stats([d['solver_s'] for d in diags]),
  request_publish_call_s=stats([(d['request_publish_return_monotonic_ns']-d['request_publish_monotonic_ns'])*1e-9 for d in [json.loads(e['data']['data']) for e in topic('/temporal_mpc/health')] if d.get('request_publish_monotonic_ns',-1)>=0]),
  frontend_overlap_cases=[r for r in [json.loads(line) for line in (root/name/'worker_timing_joined.jsonl').open()] if r.get('frontend_overlap_s',0)>0],
  worst_logged_command_gap=dict(gap_s=(right['receipt_monotonic_ns']-left['receipt_monotonic_ns'])*1e-9,left=left,right=right,nearby_guard_producer_records=near,
   scope='Observer commands have no producer sequence id. Nearby production records cannot uniquely match missing/late command delivery; no latency attribution.'),
  first_dynamic_rejection_age=age['records'][0] if age['records'] else None,
  first_dynamic_rejection_factorial=(reanchor['executed_dynamic_factorials']+reanchor['shadow_dynamic_factorials'])[0] if reanchor['native_dynamic_margins_checked'] else None,
  age_record_count=len(age['records']),age_actual_replay_matches=age['all_actual_replays_match'],age_errors=age['errors'],
  worker_input_joins=reanchor['worker_input_joins'],worker_input_errors=reanchor['errors'])
 output[name]=data
(root/'timing_analysis.json').write_text(json.dumps(output,allow_nan=False,indent=2)+'\n')
for name,d in output.items():
 print(name,json.dumps({k:v for k,v in d.items() if k not in ['frontend_overlap_cases','worst_logged_command_gap','first_dynamic_rejection_factorial','first_dynamic_rejection_age']}))
 print('worst cmd gap',d['worst_logged_command_gap']['gap_s'],'first dynamic margin',d['first_dynamic_rejection_age']['recorded_slack_m'])
