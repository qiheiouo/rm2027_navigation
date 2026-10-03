import hashlib,json,sys
from pathlib import Path
root=Path('/tmp/rm_surface_reveal_initial');out=Path('/tmp/rm_observation_v2_actual_reexecution')
summary={'source_trace_kind':'Frozen single-layer instantaneous LaserScan / planar SDF only','moving_surface_C_trial':'NOT RUN','modes':{}}
for mode in ('visible','nominal'):
    reveal=json.loads((root/(mode+'.json')).read_text());brake=json.loads((root/(mode+'_brake.json')).read_text());witness=json.loads((out/mode/'witness/geometry.json').read_text())
    actual=[next(p for p in r['proposals'] if p['name']=='actual_aggregate') for r in witness['records']]
    summary['modes'][mode]={'revealed_material_bins':reveal['reliable_bins'],'never_reliably_revealed_bins':reveal['never_reliably_revealed_bins'],
      'first_reveal_full_mechanical_gap_range_m':[reveal['non_left_censored_first_full_mechanical_gap_m'][key] for key in ('min','max')],
      'observed_brake_stop_episodes':brake['observed_stop_episodes'],'guaranteed_brake_bound':None,'total_response_delay':None,
      'all_native_cycles':witness['cycles'],'actual_aggregate_full_3s_gate_counts':{key:sum(p['gates'][key] for p in actual) for key in actual[0]['gates']},
      'actual_aggregate_safe_context_count':witness['counts']['actual_aggregate:safe_context'],
      'actual_aggregate_safe_progress_count':witness['counts']['actual_aggregate:safe_progress'],
      'raw300_full_3s_geometry':'PENDING; SDK state/poses exact only','original_overall_verdict':'FAILED'}
(root/'checkpoint_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
