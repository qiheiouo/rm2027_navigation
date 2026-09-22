#!/usr/bin/env python3
"""Sequential fresh trials, stop each group at its first valid gate failure."""
import json
import subprocess
import sys
from reference_experiment import REPO, ROOT, HERE, PLANNERS

logs=REPO/'build/tdt_p2b/snapshot_revalidation_logs'
for planner in PLANNERS:
    for trial in range(1,6):
        with (logs/f'{planner}_{trial}.log').open('x') as log:
            status=subprocess.call([sys.executable,str(HERE/'reference_experiment.py'),'run',planner,str(trial)],stdout=log,stderr=subprocess.STDOUT)
        path=ROOT/f'{planner}_{trial}'/'target_geometry_summary.json'
        if not path.exists():raise RuntimeError(f'{planner}_{trial}: missing target evidence, stop orchestration (exit {status})')
        s=json.loads(path.read_text())
        print(json.dumps({k:s[k] for k in ('planner','trial','action_status','recoveries','final_xy_error_m','target_static_pass','snapshot_admissions')}),flush=True)
        if status or not s['target_static_pass']:
            print(f'Stop {planner} group at first failed trial {trial}.',flush=True)
            break
