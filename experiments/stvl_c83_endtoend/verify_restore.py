"""Recompute original gates from selected archives; no new outcome definition."""
import pathlib,sys,json
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
review=pathlib.Path(sys.argv[1]);fields=['safe_success','success','elapsed_s','body_clearance_m','wall_overlap_samples','wait_s','start_count','path_length_m','native_errors','zero_safe_candidates','output_publisher_count'];checks=[]
for p in sorted(review.iterdir()):
 if not (p/'events.json').exists():continue
 original=json.loads((p/'metrics.json').read_text());fresh=analyze.trial(p)
 for k in fields:assert fresh[k]==original[k],(p.name,k,fresh[k],original[k])
 checks.append(dict(cohort=p.name,fields=len(fields),exact=True))
assert len(checks)==4
(ROOT/'experiments/stvl_c83_endtoend/evidence/restore_check.json').write_text(json.dumps(checks,indent=2)+'\n');print('4/4 original gates and metrics exactly reproduced')
