import json,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[2];CACHE=ROOT/'build/realtime_cache';OUT=ROOT/'experiments/stvl_execution_realtime/evidence'
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));from analyze import stats
perf=json.loads((OUT/'performance_summary.json').read_text());index={r['case']:r for r in perf['runs']};result=[]
for p in sorted((CACHE/'classification').glob('*.json')):
    r=json.loads(p.read_text());assert r['equality'];base=index[r['case']];time=stats([v['total_ms'] for v in r['runs']]);budget=None
    if base['additive_screen']:
        n=base['additive_screen']['native_compute_ms'];budget={'p99_sum_ms':n['p99']+time['p99'],'max_sum_ms':n['max']+time['max'],'partial_native_basis':True}
    result.append({'case':r['case'],'trajectories':base['trajectories'],'cold_ms':r['cold']['total_ms'],'time_ms':time,'preparation_ms':stats([v['snapshot_prepare_ms'] for v in r['runs']]),'allocation_count':stats([v['allocation_count'] for v in r['runs']]),'visited_samples':r['cold']['visited_samples'],'potential_samples':base['trajectories']*30*11,'same_geometry_and_complete':True,'additive_screen':budget})
assert len(result)==33
(OUT/'classification_summary.json').write_text(json.dumps({'runs':result,'scope':'exact same predicates, not full diagnostic telemetry; only Unsafe witness early termination; no execution/optimizer certificate'},indent=2)+'\n')
for r in result:
    if r['trajectories']>1:print(r['case'],round(r['time_ms']['p99'],2),r['visited_samples'],r['additive_screen'])
views=[]
for p in sorted((CACHE/'views').glob('*.json')):
    r=json.loads(p.read_text());a,b=r['predicate_results'];views.append({'case':r['case'],'original':a,'previous_yaw_correction':b,'geometry_changed':a['joint_geometry']!=b['joint_geometry']})
assert len(views)==20
(OUT/'SDK_view_safety.json').write_text(json.dumps({'pairs':views,'changed_geometry_flags':sum(r['geometry_changed'] for r in views),'scope':'corrected read-only selected view, no actual controller/plant modification'},indent=2)+'\n')
