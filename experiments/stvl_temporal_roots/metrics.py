import json,pathlib,sys,os
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'experiments/stvl_temporal'));import analyze
from data import jsonlines
analyze.jsonlines=jsonlines
cohort=sys.argv[1];cache=ROOT/'build/roots_cache'/cohort;analyze.CACHE=cache
rows=[analyze.trial(p) for p in sorted((cache/'runs').iterdir()) if (p/'events.json').exists()]
(ROOT/'experiments/stvl_temporal_roots/evidence'/f'{cohort}_outcomes.json').write_text(json.dumps(rows,indent=2)+'\n')
for r in rows:print(r['run'],'goal',r.get('success'),'safe',r.get('safe_success'),'contact',r['contact_messages'],'wall',r.get('wall_overlap_samples'),'gap',r.get('body_clearance_m'),'time',r.get('arrival_s'))
