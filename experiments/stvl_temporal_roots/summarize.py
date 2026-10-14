"""All attempts, runtime exclusions and velocity-input contract; frozen gates unchanged."""
import json,pathlib,sys,re,csv,math,gzip
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
from data import jsonlines,COHORTS
analyze.jsonlines=jsonlines
OUT=ROOT/'experiments/stvl_temporal_roots/evidence';rows=[]
for cohort in COHORTS:
 cache=ROOT/'build/roots_cache'/cohort;analyze.CACHE=cache
 for p in sorted((cache/'runs').glob('*')):
  if not (p/'events.json').exists():continue
  row=analyze.trial(p);row['cohort']=cohort;log=(p/'launch.log').read_text();teardown=log.find('user interrupted with ctrl-c');crashes=[m for m in re.finditer('exit code (-11|139)',log) if 'component_container_isolated' in log[max(0,m.start()-150):m.start()]];early=any(teardown<0 or m.start()<teardown for m in crashes)
  row['runtime_exit_before_teardown']=early;row['eligible_controller_outcome']=not row['startup'] and row['group']!='CAL' and not early
  if early:row['corrected_classification']='runtime_controller_loss; preserve original recorded outcome, not a valid normal-planner timeout/contact'
  controls=jsonlines(p/'control.jsonl');row['speed_input_nonzero_cycles']=sum(any(abs(v)>1e-9 for v in r['speed']) for r in controls);row['speed_input_total_cycles']=len(controls)
  row['storage_preserved_capture_count']=sum(r.get('root_diagnostics',{}).get('cost_storage',{}).get('preserved',False) for r in controls)
  row['controller_odom_topic_explicit']=json.loads((p/'manifest.json').read_text()).get('mode') and '/odometry/lio' if cohort in ('repair_odom','heldout_odom') else 'native default odom; omitted controller param'
  row['malformed_owned_streams']={}
  for stream in list(p.glob('*.jsonl'))+list(p.glob('*.jsonl.gz')):
   op=gzip.open if stream.suffix=='.gz' else open
   with op(stream,'rt') as f:row['malformed_owned_streams'][stream.name]=sum(not analyze.valid_json(line) for line in f)
  rows.append(row)
summary={'attempts':len(rows),'started_trial_epochs':sum(not r['startup'] for r in rows),'accepted_goals':sum(not r['startup'] and r['group']!='CAL' for r in rows),'calibration_trials':sum(r['group']=='CAL' for r in rows),'early_controller_crash_trials':sum(r['runtime_exit_before_teardown'] for r in rows),'eligible_controller_trials':sum(r['eligible_controller_outcome'] for r in rows),'startup_failures':sum(r['startup'] for r in rows),'trials':rows,'gate':'unchanged goal, zero actor contact, full bbox gap>=.047m, zero sampled wall bbox overlap; acceleration independently reported'}
(OUT/'all_attempts.json').write_text(json.dumps(summary,indent=2)+'\n')
lines=['| cohort | case | goal | frozen safe | actor contact | wall samples | dynamic gap m | arrival s | compute p99 ms | runtime |','|---|---|---:|---:|---:|---:|---:|---:|---:|---|']
def fmt(x):return '—' if x is None else f'{x:.3f}'
for r in rows:lines.append(f"| {r['cohort']} | {r['run']} | {int(r.get('success',False))} | {int(r.get('safe_success',False))} | {r['contact_messages']} | {r.get('wall_overlap_samples','—')} | {fmt(r.get('body_clearance_m'))} | {fmt(r.get('arrival_s'))} | {fmt((r.get('compute_ms') or {}).get('p99'))} | {'controller lost' if r['runtime_exit_before_teardown'] else 'not started' if r['startup'] else 'complete'} |")
(OUT/'all_attempts.md').write_text('\n'.join(lines)+'\n');print({k:v for k,v in summary.items() if k not in ('trials','gate')})
