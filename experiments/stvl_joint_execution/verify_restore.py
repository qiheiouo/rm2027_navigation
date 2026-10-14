"""Recompute original gate from compact archive truth; do not touch old trials."""
import pathlib,json,sys,math
ROOT=pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal_roots'));from data import jsonlines
analyze.jsonlines=jsonlines;dest=pathlib.Path(sys.argv[1]).resolve();analyze.CACHE=dest;results=[]
for cohort in ('fresh','observer_retry'):
    for p in sorted((dest/cohort).glob('*')):
        old=json.loads((p/'metrics.json').read_text());new=analyze.trial(p)
        fields=('success','safe_success','elapsed_s','body_clearance_m','wall_overlap_samples','wait_s','start_count','path_length_m','native_events','zero_safe_candidates','actual_max_acceleration','compute_over100','output_bounds_violations')
        assert all(old[k]==new[k] for k in fields),(p.name,{k:[old[k],new[k]] for k in fields if old[k]!=new[k]})
        results.append({'cohort':cohort,'run':p.name,'matched':True,'fields':list(fields)})
assert len(results)==9
(ROOT/'experiments/stvl_joint_execution/evidence/restore_check.json').write_text(json.dumps(results,indent=2)+'\n');print('9/9 restored original safety/efficiency/timing metrics match exactly')
