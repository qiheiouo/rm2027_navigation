"""Plain-call timing used for budgets; instrumented clocks/counters are separate."""
import pathlib,json,sys,collections
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_execution_realtime/evidence';CACHE=ROOT/'build/realtime_cache'
sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));from analyze import stats
prior=ROOT.parent/'stvl_joint_execution_20261014/experiments/stvl_joint_execution/evidence'
sources={pathlib.Path(r['input']).stem:pathlib.Path(r['source_control']).parent for r in json.loads((prior/'history_index.json').read_text())}
sources.update({r['case']:pathlib.Path(r['source']).parent for r in json.loads((prior/'fresh_snapshot_index.json').read_text())})
def main():
    summaries=[]
    for p in sorted((CACHE/'benchmark').glob('*.json')):
        r=json.loads(p.read_text());row={'case':r['case'],'trajectories':r['trajectories'],'steps':r['steps'],'vertices':r['vertices'],'map_cells':r['map_cells'],'parse_ms':r['parse_ms'],'RSS_kB':r['peak_RSS_kB'],'equality':r['optimization_equality'],
            'cold_reference_ms':r['cold_reference']['total_ms'],'cold_optimized_ms':r['cold_optimized']['total_ms'],'native_offline_comparison':r['native_offline_comparison'],'serialization_ms':r['serialization_ms'],
            'timing':{k:stats([v['total_ms'] for v in r[k]]) for k in ('reference','profiled','optimized','optimized_profiled','selected_only')},
            'stages':{k:{stage:stats([v['stages'][stage] for v in r[k]]) for stage in ('prepare_ms','sweep_ms','static_ms','dynamic_ms','rate_ms','result_ms')} for k in ('profiled','optimized_profiled')},
            'allocation':{k:{a:stats([v[a] for v in r[k]]) for a in ('allocation_count','allocated_bytes')} for k in ('reference','optimized','selected_only')},
            'flags':{k:dict(collections.Counter(x[k] for x in r['predicate_results'])) for k in ('static','dynamic','nominal_rate','joint_geometry','joint_complete')},'additive_screen':None}
        if r['case'] in sources:
            native=json.loads((sources[r['case']]/'metrics.json').read_text())['compute_ms'];extra=row['timing']['optimized'];single=row['timing']['selected_only']
            row['additive_screen']={'basis':'historical native call + isolated plain snapshot/checker; not simultaneous runtime proof, no serialization/acquisition/dispatch bound','native_compute_ms':native,
                'p99_sum_ms':native['p99']+extra['p99'],'max_sum_ms':native['max']+extra['max'],
                'p99_selected_sum_ms':native['p99']+single['p99'],'full_p99_under100':native['p99']+extra['p99']<=100,'full_max_under100':native['max']+extra['max']<=100}
        summaries.append(row)
    assert len(summaries)==33 and all(r['equality'] for r in summaries)
    (OUT/'performance_summary.json').write_text(json.dumps({'clock':'steady_clock','repeats_per_input':31,'inputs':33,'full_real_snapshots':20,'fixture_update_inputs':13,'scope':'full in-memory preparation+301 trajectory kernel vs selected-only; read/JSON and native comparison are offline extras; each call rebuilds snapshot','runs':summaries},indent=2)+'\n')
    lines=['| case | ref p50/p95/p99 ms | optimized p50/p95/p99/max ms | selected p99 ms | allocations ref→opt | native+opt p99/max ms |','|---|---|---|---|---|---|']
    for r in summaries:
        if r['trajectories']==1:continue
        fmt=lambda x:'/'.join(f'{x[k]:.2f}' for k in ('p50','p95','p99'))
        a=r['allocation'];screen=r['additive_screen'];budget=f"{screen['p99_sum_ms']:.2f}/{screen['max_sum_ms']:.2f}" if screen else '—'
        lines.append(f"| {r['case']} | {fmt(r['timing']['reference'])} | {fmt(r['timing']['optimized'])}/{r['timing']['optimized']['max']:.2f} | {r['timing']['selected_only']['p99']:.2f} | {int(a['reference']['allocation_count']['p50'])}→{int(a['optimized']['allocation_count']['p50'])} | {budget} |")
    (OUT/'performance_table.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines))
if __name__=='__main__':main()
