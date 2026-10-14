"""Restore selected archive and reproduce physical/temporal primary metrics."""
import pathlib,tempfile,tarfile,json,hashlib,gzip
import analyze as metrics
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_active_scope/evidence'
with tempfile.TemporaryDirectory(prefix='active_scope_review_') as temp:
 dst=pathlib.Path(temp)
 with tarfile.open(OUT/'paired_selected.tar.gz') as tf:tf.extractall(dst,filter='data')
 manifest=json.loads((OUT/'archive_manifest.json').read_text())
 for x in manifest['files']:assert hashlib.sha256((dst/x['path']).read_bytes()).hexdigest()==x['sha256'],x['path']
 metrics.OUT=dst/'metrics';metrics.OUT.mkdir()
 checks=[]
 for p in sorted(dst.glob('*/runs/*')):
  # Selected control is explicitly labelled; restore a derived comparison view.
  (p/'control.jsonl.gz').write_bytes((p/'control_selected.jsonl.gz').read_bytes())
  result=metrics.run(p,native_required=False);original=json.loads((OUT/(result['input_scope']+'_'+result['run']+'.json')).read_text())
  for key in ('success','safe_success','arrival_s','body_clearance_m','base_collision_box_gap_m','wall_overlap_samples','output_single_owner','output_bounds_violations','path_length_m','wait_s','projection','native_errors','prediction_reason_counts','zero_safe_candidates','entry_to_DDS_publication_ms'):
   assert result[key]==original[key],(p.name,key)
  checks.append(result['input_scope']+'/'+p.name)
 print(json.dumps(dict(verified_files=len(manifest['files']),reproduced_runs=checks,scope='full1ms native limit check not part of selected archive; original local cache retained')))
