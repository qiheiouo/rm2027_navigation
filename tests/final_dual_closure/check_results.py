"""Compare actual state traces and ROS receipts; no synthetic pass substitution."""
import argparse,json,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output_root');p.add_argument('--reference',default='after_reference_fixed');p.add_argument('--bundle',default='after_bundle_retry');args=p.parse_args();b=Path(args.output_root)
def load(path):return json.loads((b/path).read_text())
def trace(prefix,mode):
    return [json.loads(line.removeprefix('TRACE ')) for line in (b/f'{prefix}_trace_{mode}.log').read_text().splitlines() if line.startswith('TRACE ')]
def state(r):return (r['updates'],r['last_end'],r['state'],r['cov_trace'],r['cov_max'])
bc=trace('before','control');bb=trace('before','backfill');ac=trace('after','control');ab=trace('after','backfill')
assert state(bc[-1])==state(bb[-1])==state(ac[-1])==state(ab[-1])
assert ac[-1]['updates']==ac[0]['updates']+1 and ab[-1]['updates']==ab[0]['updates']+1
faults={}
for mode in ['gap','rewind','imu_rewind','lidar_restart']:
    a=trace('after',mode);assert all(state(r)==state(a[0]) for r in a)
    faults[mode]={'updates_before':a[0]['updates'],'updates_after':a[-1]['updates'],'state_unchanged':True}
br=trace('before','rewind');assert br[-1]['updates']>br[0]['updates'] and br[-1]['cov_trace']>br[0]['cov_trace']*1000
new=trace('after','new_epoch');assert new[0]['state']==ac[0]['state'] and new[0]['cov_trace']==ac[0]['cov_trace']
assert trace('after','duplicate')[-1]['state']==ac[-1]['state']
for name in ['imu','measurement']:
    assert (b/f'before_{name}.stdout').read_bytes()==(b/f'after_{name}.stdout').read_bytes()
r=load(args.reference+'/result.json');assert len(r['cases'])==26 and len(r['anchors'])==2 and len(r['faults'])==14
assert all(f['new_odoms']==0 for f in r['faults'])
assert all(x['position_error_m']<1e-7 and x['rotation_matrix_error']<1e-7 for x in r['cases'])
before=load('before_bundle/result.json');after=load(args.bundle+'/result.json');max_diff=0.
for x,y in zip(before['cases'],after['cases']):
    assert x['name']==y['name'] and len(x['samples'])==len(y['samples'])
    for left,right in zip(x['samples'],y['samples']):
        assert left['stamp']==right['stamp']
        max_diff=max(max_diff,max(abs(v-w) for v,w in zip(left['pose'],right['pose'])))
assert max_diff<1e-7,max_diff
ros=load('after_epoch_ros/result.json');assert len(ros['cases'])==7
summary={'normal_backend_ros_max_pose_component_change':max_diff,
  'native_normal_backfill_before_after_state_and_covariance_identical':True,
  'original_rollback_failure':{'y_m':br[-1]['state'][1],'covariance_trace':br[-1]['cov_trace'],'updates':br[-1]['updates']},
  'repaired_faults':faults,'new_process_epoch_matches_normal_relative_state':True,
  'aud038_aud046_native_receipts_identical':True,
  'source_reference':{'normal_samples':len(r['cases']),'anchors':len(r['anchors']),
    'fault_checks':len(r['faults']),'max_position_error_m':max(x['position_error_m'] for x in r['cases']),
    'max_rotation_matrix_error':max(x['rotation_matrix_error'] for x in r['cases']),
    'tf_owner_counts_by_epoch':r['tf_owner_counts_by_epoch']},
  'actual_ros_epoch_counts':[{'case':x['name'],'warm':x['warm_odoms'],'final':x['final_odoms']} for x in ros['cases']],
  'limits':['software fixtures only','legacy fixed_mount dynamic negative intentionally preserved',
    'first full ROS rerun timed out on source_skew shutdown; unchanged retry passed, root cause unresolved',
    'first new adapter fixture wrongly expected first-pose twist publication; expectation corrected, production unchanged']}
(b/'verified_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
