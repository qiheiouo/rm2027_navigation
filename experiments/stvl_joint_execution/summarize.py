"""All fresh attempts, no outcome selection, original gate and own outputs only."""
import pathlib,json,sys,csv,collections,gzip,hashlib,tarfile,io
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/joint_cache';OUT=ROOT/'experiments/stvl_joint_execution/evidence'
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal_roots'));from data import jsonlines
import audit
analyze.jsonlines=jsonlines
def tap_summary(p,start,end):
    logs=(p/'tap.log').read_text() if (p/'tap.log').exists() else ''
    rows=[r for r in jsonlines(p/'execution_receipts.jsonl') if start is not None and start<=r['ROS_receipt_ns']<=end]
    graph=jsonlines(p/'execution_endpoints.jsonl');names={}
    for r in graph:
        for t,eps in r['topics'].items():names.setdefault(t,set()).update(e['name'] for e in eps)
    # Different tap and record callbacks can have different queue lag.
    paired=[]
    for topic in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel'):
        messages=[r for r in rows if r['topic']==topic]
        for a,b in zip(messages,messages[1:]):
            wall=(b['monotonic_receipt_ns']-a['monotonic_receipt_ns'])/1e6;sim=(b['ROS_receipt_ns']-a['ROS_receipt_ns'])/1e6
            if wall>0:paired.append({'topic':topic,'wall_ms':wall,'sim_ms':sim,'RTF':sim/wall})
    return {'error':'Traceback' in logs,'error_tail':logs[-600:] if 'Traceback' in logs else None,'receipt_count':len(rows),'publishers':{k:sorted(v) for k,v in names.items()},
            'per_message_producer':'unavailable in fixed Humble rclpy; /cmd_vel has multiple native sources',
            'application_time':'Unknown; no actuator callback stamp',
            'periods':{topic:{k:analyze.stats([r[k] for r in paired if r['topic']==topic]) for k in ('wall_ms','sim_ms','RTF')} for topic in ('/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel')}}
def main():
    results=[];executions=[]
    for cohort in ('fresh','observer_retry'):
        analyze.CACHE=CACHE/cohort
        for p in sorted((analyze.CACHE/'runs').glob('*')):
            if not (p/'events.json').exists():results.append({'run':p.name,'cohort':cohort,'incomplete':True});continue
            row=analyze.trial(p);row['cohort']=cohort;row['heldout']=row['seed']==83
            row['tap']=tap_summary(p,json.loads((p/'events.json').read_text())['goal_ns'],json.loads((p/'events.json').read_text())['end_ns']);results.append(row)
            exe=audit.execution(p);exe['cohort']=cohort;executions.append(exe)
    primary=[r for r in results if r['cohort']=='fresh'];summary={'attempts':len(results),'valid_closed_loops':sum(not r.get('startup',True) and not r.get('incomplete',False) for r in results),'primary_n':len(primary),'observer_failures':sum(r.get('tap',{}).get('error',False) for r in results),
        'primary_goal':sum(r.get('success',False) for r in primary),'primary_safe_gate':sum(r.get('safe_success',False) for r in primary),'trials':results,'Cprime':'not run: execution rollout/application delay/phase bounds missing, full-batch joint diagnosis exceeds budget; no control mutation',
        'gate':'original goal/contact/.047 bbox dynamic/sampled wall gate; acceleration and continuous sweep reported separately',
        'statistical_scope':'one run per fixed case, paired by seed; observer retry is separate, not pooled into group averages'}
    (OUT/'fresh_results.json').write_text(json.dumps(summary,indent=2)+'\n');(OUT/'execution_fresh.json').write_text(json.dumps({'runs':executions},indent=2)+'\n')
    lines=['| cohort | scene | group/seed | goal / gate | elapsed s | dynamic gap m | wall hits | wait s | starts | p95/p99 ms | >100ms | observer |','|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|']
    for r in results:
        if r.get('incomplete'):lines.append(f"| {r['cohort']} | {r['run']} | incomplete | — | — | — | — | — | — | — | — | — |");continue
        f=lambda v:'—' if v is None else f'{v:.3f}'
        compute=r.get('compute_ms') or {};lines.append(f"| {r['cohort']} | {r['scene']} | {r['group']}/{r['seed']} | {int(r.get('success',False))}/{int(r.get('safe_success',False))} | {f(r.get('elapsed_s'))} | {f(r.get('body_clearance_m'))} | {r.get('wall_overlap_samples','—')} | {f(r.get('wait_s'))} | {r.get('start_count','—')} | {f(compute.get('p95'))}/{f(compute.get('p99'))} | {r.get('compute_over100','—')} | {'failed' if r['tap']['error'] else 'recorded'} |")
    (OUT/'fresh_table.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines))
    protocols={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'experiments/stvl_joint_execution').glob('*contract.json')};protocols['protocol.json']=hashlib.sha256((ROOT/'experiments/stvl_joint_execution/protocol.json').read_bytes()).hexdigest()
    (OUT/'protocol_hashes.json').write_text(json.dumps(protocols,indent=2)+'\n')
if __name__=='__main__':main()
