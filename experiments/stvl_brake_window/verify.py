"""Recompute both complete physical runs from the selected archive, no live run."""
import tempfile,pathlib,json,hashlib,tarfile,sys
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_brake_window/evidence'
sys.path.insert(0,str(ROOT/'experiments/stvl_lifecycle_hold'));import fixture_metrics as fm
with tempfile.TemporaryDirectory(prefix='brake_window_review_') as temp:
 dst=pathlib.Path(temp)
 with tarfile.open(OUT/'execution_complete.tar.gz') as tf:tf.extractall(dst,filter='data')
 for x in json.loads((OUT/'manifest.json').read_text()):assert hashlib.sha256((dst/x['path']).read_bytes()).hexdigest()==x['sha256']
 fm.OUT=dst/'metrics';fm.OUT.mkdir()
 for label in ('brake_late04','brake_late08'):
  a=fm.run(dst/label);b=json.loads((OUT/(label+'_metrics.json')).read_text())
  for key in ('profiles','prefix_comparison','native_max_target_acceleration','native_valid_intervals','native_parse_bad','dynamic_min_gap_m','contact_count','wall_overlap_samples'):assert a[key]==b[key],(label,key)
 print('two complete physical metric records reproduced; no new live run')
