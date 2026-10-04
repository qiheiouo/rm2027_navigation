#!/usr/bin/env python3
"""Exact producer/callback joins; never subtract different clock domains."""
import argparse
from collections import Counter
import json
from pathlib import Path
import math
import numpy as np
from audit_run import stats
from temporal_mpc.timing import request_clock_status


def evaluate(events):
    records=[];errors=[];producers={};receipts={};diagnostics={}
    timings=[]
    for event in events:
        topic=event['topic']
        if topic=='/temporal_mpc/health':
            d=json.loads(event['data']['data'])
            if d.get('request_publish_monotonic_ns',-1)>=0:
                producers.setdefault((d['request_epoch_ns'],d['generation']),[]).append(d)
            if d.get('proposal_receipt_monotonic_ns',-1)>=0:
                receipts.setdefault((d['proposal_ns'],d['generation']),[]).append(d['proposal_receipt_monotonic_ns'])
        elif topic=='/temporal_mpc/solver_diagnostic':
            d=json.loads(event['data']['data'])
            if 'timing_callback_id' in d:diagnostics[d['timing_callback_id']]=d
        elif topic=='/temporal_mpc/worker_timing':timings.append(json.loads(event['data']['data']))
    frontends=[d for d in timings if d['kind']=='frontend'];joins=0;proposal_joins=0;clock_checks=0
    for t in timings:
        if t['kind']!='request':continue
        key=(t['request_epoch_ns'],t['generation']);row=dict(t)
        if 'decision_clock_ns' in t:
            clock_checks+=1
            expected=request_clock_status(t['request_epoch_ns'],t['request_frame'],t['decision_clock_ns'])
            if expected!=t['request_clock_status'] or t['request_signed_age_ns']!=t['decision_clock_ns']-t['request_epoch_ns']:
                errors.append('request clock classification mismatch')
        parents=[d for d in producers.get(key,[]) if d['request_publish_monotonic_ns']<=t['callback_start_monotonic_ns']]
        if parents:
            d=max(parents,key=lambda d:d['request_publish_monotonic_ns']);joins+=1
            begin=d['request_publish_monotonic_ns'];end=t['callback_start_monotonic_ns']
            row['native_request_publish_monotonic_ns']=begin
            row['native_request_publish_clock_ns']=d['request_publish_clock_ns']
            row['publish_start_to_callback_s']=(end-begin)*1e-9
            row['frontend_overlap_s']=sum(max(0,min(end,f['callback_body_end_monotonic_ns'])-
                                                max(begin,f['callback_start_monotonic_ns'])) for f in frontends)*1e-9
            raw=t['raw_request_fields'];actual=d['initial_state']
            measured=[raw[0],raw[1],2*math.atan2(raw[5],raw[6]),raw[7],raw[8],raw[12]] if not t['raw_request_invalid_indices'] else None
            if measured is not None and all(v is not None for v in actual) and np.max(np.abs(np.array(measured)-actual))>1e-8:
                errors.append('raw request/native measured state mismatch')
        if t['proposal_published']:
            diag=diagnostics.get(t['callback_id'])
            if diag is None:errors.append('published proposal solver diagnostic missing')
            elif (diag['epoch_ns'],diag['generation'])!=key:errors.append('callback/solver identity mismatch')
            relevant=[ns for ns in receipts.get(key,[]) if ns>=t['proposal_publish_monotonic_ns']]
            if relevant:
                proposal_joins+=1;row['proposal_publish_start_to_native_receipt_s']=(min(relevant)-t['proposal_publish_monotonic_ns'])*1e-9
        row['callback_body_s']=(t['callback_body_end_monotonic_ns']-t['callback_start_monotonic_ns'])*1e-9
        if 'solver_start_monotonic_ns' in t:
            row['mpc_solve_call_s']=(t['solver_end_monotonic_ns']-t['solver_start_monotonic_ns'])*1e-9
        records.append(row)
    return dict(request_callbacks=len(records),native_request_joins=joins,native_proposal_receipt_joins=proposal_joins,
                clock_classifications_checked=clock_checks,
                dispositions=dict(Counter(d['disposition'] for d in records)),
                clock_statuses=dict(Counter(d.get('request_clock_status','not_checked') for d in records)),
                publish_start_to_callback_s=stats([d['publish_start_to_callback_s'] for d in records if 'publish_start_to_callback_s' in d]),
                proposal_publish_start_to_native_receipt_s=stats([d['proposal_publish_start_to_native_receipt_s'] for d in records if 'proposal_publish_start_to_native_receipt_s' in d]),
                callback_body_s=stats([d['callback_body_s'] for d in records]),
                frontend_body_s=stats([(d['callback_body_end_monotonic_ns']-d['callback_start_monotonic_ns'])*1e-9 for d in frontends]),
                frontend_overlap_callbacks=sum(d.get('frontend_overlap_s',0)>0 for d in records),
                future_request_cases=[d for d in records if d.get('request_clock_status')=='future'],
                errors=errors,records=records,
                scope='Exact same-host Linux monotonic joins and signed local ROS clock decisions. Publish-start to callback includes transport/executor/clock-callback order; no unique latency allocation. Timing emission itself follows callback_body end. DDS system timestamps unavailable, never inferred.',dynamic_acceptance=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args();root=Path(a.directory)
    if (root/'worker_timing_audit.json').exists():raise SystemExit('refuse timing audit overwrite')
    with (root/'events.jsonl').open() as stream:d=evaluate([json.loads(line) for line in stream])
    rows=d.pop('records');(root/'worker_timing_audit.json').write_text(json.dumps(d,allow_nan=False,indent=2)+'\n')
    (root/'worker_timing_joined.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))
    print(json.dumps({k:v for k,v in d.items() if k!='future_request_cases'},indent=2))
