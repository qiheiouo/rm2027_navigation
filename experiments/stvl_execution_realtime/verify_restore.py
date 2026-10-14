import pathlib,json,sys
ROOT=pathlib.Path(__file__).resolve().parents[2];dest=pathlib.Path(sys.argv[1]);sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal_roots'));from data import jsonlines
analyze.jsonlines=jsonlines;analyze.CACHE=dest;results=[]
for cohort in ('fresh','boundary_enabled','no_capture','corrected_scene'):
    for p in sorted((dest/cohort).glob('*')):
        old=json.loads((p/'metrics.json').read_text());new=analyze.trial(p)
        fields=('success','safe_success','elapsed_s','body_clearance_m','wall_overlap_samples','wait_s','start_count','path_length_m','native_events','zero_safe_candidates','actual_max_acceleration','compute_over100','output_bounds_violations')
        assert all(old[k]==new[k] for k in fields),p.name;results.append({'cohort':cohort,'run':p.name,'match':True})
assert len(results)==8
(ROOT/'experiments/stvl_execution_realtime/evidence/restore_check.json').write_text(json.dumps(results,indent=2)+'\n');print('8/8 original metrics exactly reproduced from archive')
