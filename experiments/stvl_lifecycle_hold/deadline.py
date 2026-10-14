"""Measured hold intervals vs narrowly defined post100ms deadline extension.
No attribution of an entire normal calculation interval to an overrun.
"""
import json,pathlib,sys,math
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parents[2];OUT=ROOT/'experiments/stvl_lifecycle_hold/evidence';BASE=pathlib.Path('/home/qihei/rm2027_navigation/build/stvl_c83_endtoend_20261014')
sys.path.insert(0,str(BASE/'experiments/stvl_execution_realtime'));from mapping import rows
sys.path.insert(0,str(ROOT/'experiments/stvl_lifecycle_hold'));from fixture_metrics import interp
p=BASE/'build/c83_cache/shadow/runs/narrow_C_83_aligned';n=rows(p/'native_boundary.csv');nt=[int(x['steady_ns']) for x in n];sim=[int(x['sim_ns']) for x in n];od=rows(p/'odometry.csv')
h=json.loads((OUT/'overrun_history_lifecycle.json').read_text());a=[]
for i,c in enumerate(h['cycles']):
 if not c['timing'] or not c['publication']:continue
 entry=c['timing']['entry_steady_ns'];pub=c['publication']['pub_steady_ns'];total=(pub-entry)/1e6
 if total<=100:continue
 old=h['cycles'][i-1];oldpub=old['publication']['pub_steady_ns'];t0=float(np.interp(oldpub,nt,sim));t1=float(np.interp(pub,nt,sim));td=float(np.interp(entry+100000000,nt,sim));body0=interp(od,t0);body1=interp(od,t1);bodyd=interp(od,td)
 a.append(dict(cycle_id=c['cycle_id'],relative_s=c['relative_s'],dynamic_safe=c['dynamic_safe'],entry_to_pub_ms=total,excess_after_entry100ms_ms=total-100,previous_decision_publication_hold_ms=(pub-oldpub)/1e6,body_displacement_during_whole_hold_m=float(np.linalg.norm(body1[:2]-body0[:2])),interpolated_body_displacement_only_post_deadline_m=float(np.linalg.norm(body1[:2]-bodyd[:2])),post_deadline_displacement_scope='50Hz physical feedback interpolation;0.59ms below raw sampling, not exact causal counterfactual',first_downstream=c['first_downstream_cmd'],next_interval=c['publication']['until_next_nav_publication_ms']))
(OUT/'historical_overrun.json').write_text(json.dumps(a,indent=2)+'\n');print(a)
