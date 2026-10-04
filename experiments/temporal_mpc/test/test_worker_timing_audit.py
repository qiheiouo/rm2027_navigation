from pathlib import Path
import sys
import json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'gazebo'))
from audit_worker_timing import evaluate


def fixture():
    t=dict(kind='request',callback_id=1,request_epoch_ns=1_000_000_000,generation=2,
           callback_start_monotonic_ns=120,callback_body_end_monotonic_ns=200,
           decision_clock_ns=999_000_000,request_signed_age_ns=-1_000_000,request_clock_status='future',
           request_frame='map',disposition='proposal',proposal_published=True,
           proposal_publish_monotonic_ns=180,raw_request_fields=[0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.],raw_request_invalid_indices=[])
    h=dict(request_epoch_ns=1_000_000_000,generation=2,request_publish_monotonic_ns=100,
           request_publish_clock_ns=1_001_000_000,proposal_receipt_monotonic_ns=190,proposal_ns=1_000_000_000,initial_state=[0.]*6)
    d=dict(timing_callback_id=1,epoch_ns=1_000_000_000,generation=2)
    return [dict(topic=topic,data=dict(data=json.dumps(value))) for topic,value in [('/temporal_mpc/worker_timing',t),('/temporal_mpc/health',h),('/temporal_mpc/solver_diagnostic',d)]]


def test_exact_clock_and_monotonic_join_do_not_use_observer_receipt_time():
    events=fixture();r=evaluate(events)
    assert not r['errors'] and r['native_request_joins']==r['native_proposal_receipt_joins']==1
    assert r['future_request_cases'][0]['native_request_publish_clock_ns']==1_001_000_000
    assert r['publish_start_to_callback_s']['max']==20e-9
    events.reverse();assert evaluate(events)['publish_start_to_callback_s']==r['publish_start_to_callback_s']


def test_clock_mismatch_or_missing_identity_is_not_silently_repaired():
    events=fixture();t=json.loads(events[0]['data']['data']);t['request_clock_status']='current'
    events[0]['data']['data']=json.dumps(t);r=evaluate(events[:2])
    assert 'request clock classification mismatch' in r['errors']
    assert 'published proposal solver diagnostic missing' in r['errors']
