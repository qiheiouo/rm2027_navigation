"""Restore selected execution data, reproduce physical metrics, no live run."""
import pathlib,json,tarfile,hashlib,tempfile,sys
import fixture_metrics as fm
ROOT=pathlib.Path(__file__).resolve().parents[2];P=ROOT/'experiments/stvl_lifecycle_hold/evidence'
with tempfile.TemporaryDirectory(prefix='stvl_hold_review_') as temp:
 dst=pathlib.Path(temp)
 with tarfile.open(P/'execution_selected.tar.gz') as tf:tf.extractall(dst,filter='data')
 m=json.loads((P/'archive_manifest.json').read_text())
 for x in m['files']:assert hashlib.sha256((dst/x['path']).read_bytes()).hexdigest()==x['sha256'],x['path']
 fm.OUT=dst/'metrics';fm.OUT.mkdir()
 checks=[]
 for label in ('timing','timing_retry','reference','brake','brake_delayed'):
  r=fm.run(dst/label);old=json.loads((P/(label+'_metrics.json')).read_text())
  for key in ('contact_count','bbox_overlap_samples','wall_overlap_samples','dynamic_min_gap_m'):assert r[key]==old[key],(label,key)
  for a,b in zip(r['profiles'],old['profiles']):
   for key in ('case','first_zero_publication_s','body_stop_s','displacement_request_to_stop_m','injected_output_delays_ms'):assert a[key]==b[key],(label,key)
  if label.startswith('brake'):assert r['native_max_target_acceleration']==old['native_max_target_acceleration'];assert r['prefix_comparison']['prefix_all_within_frozen_tolerance']
  checks.append(label)
 print(json.dumps(dict(archive_hashes=len(m['files']),physical_metrics_reproduced=checks,scope='no new control run; timing/reference native1ms not archived')))
