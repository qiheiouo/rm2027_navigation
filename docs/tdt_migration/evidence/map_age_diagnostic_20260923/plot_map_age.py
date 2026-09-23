#!/usr/bin/env python3
"""Visual aid for the seven recorded cycles around each dynamic clearance failure."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/'map_age_analysis_v2.json').read_text())['trials']
fig,axes=plt.subplots(2,2,figsize=(10,6),sharex='row')
for i,trial in enumerate(rows):
    t0=trial['first_body_below_005_sim_s']
    w=trial['window']
    x=[r['sim_s']-t0 for r in w]
    a,b=axes[i]
    a.plot(x,[r['actual_robot_body_to_box_gap_m'] for r in w],'-o',label='actual moving box')
    a.plot(x,[r['raw_254_body_gap_m'] for r in w],'-s',label='locked raw lethal cells')
    a.axhline(.05,color='tab:red',linestyle='--',label='body clearance gate')
    a.axvline(0,color='black',linewidth=.8)
    a.set_ylabel(f"{trial['planner']} gap (m)")
    a.grid(alpha=.25)
    if i==0:a.legend(fontsize=8)
    b.plot(x,[r['map_age_ms'] for r in w],'-o',label='master map age (wall ms)')
    b.plot(x,[r['selected_scan_age_sim_s']*1000 for r in w],'-s',label='selected scan age (sim ms)')
    b.axvline(0,color='black',linewidth=.8)
    b.set_ylabel('age (ms; separate clocks)')
    b.grid(alpha=.25)
    if i==0:b.legend(fontsize=8)
    a.set_xlabel('time from first body gap < 0.05 m (sim s)')
    b.set_xlabel('time from first body gap < 0.05 m (sim s)')
fig.suptitle('Fixed-phase diagnostic first cases: local map and observation age')
fig.tight_layout()
fig.savefig(HERE/'map_age_window.png',dpi=160)
fig.savefig(HERE/'map_age_window.svg')
