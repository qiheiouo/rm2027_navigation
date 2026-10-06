"""Decision figure for recorded proposals and the explicitly ideal feedback probe."""
import csv
import json
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root, output = map(Path, sys.argv[1:3])
with (output/'replay.csv').open() as stream:
    replay = list(csv.DictReader(stream))
with (output/'probe.csv').open() as stream:
    probes = list(csv.DictReader(stream))
fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), constrained_layout=True)
for ax, scene in zip(axes[:2], ('S1', 'S2')):
    events = json.loads((root/f'experiments/r4_corrected_runtime_shadow/evidence/{scene}_events.json').read_text())['events']
    begin = next(e['ROS_ns'] for e in events if e['kind'] == 'goal_accepted')
    clear = (next(e['ROS_ns'] for e in events if e['kind'] == 'clear_target')-begin)*1e-9
    ax.axvspan(1., clear, color='#eeeeee', label='original dynamic window')
    for label, style in (('observed', '-'), ('no_observed_dynamic', '--')):
        selected = [r for r in replay if r['scene'] == scene and r['condition'] == label]
        x = [(int(r['epoch_ns'])-begin)*1e-9 for r in selected]
        y = [float(r['forward']) if r['valid'] == '1' else float('nan') for r in selected]
        ax.plot(x, y, style, lw=1.5, label=label)
    failed = [r for r in replay if r['scene'] == scene and r['condition'] == 'observed' and r['reason'] == 'solver_not_solved']
    ax.scatter([(int(r['epoch_ns'])-begin)*1e-9 for r in failed], [0. for r in failed], marker='x', c='red', label='unavailable solve' if failed else None)
    ax.axvline(clear, ls=':', color='black')
    ax.axhline(0., lw=.7, color='gray')
    ax.set(title=f'{scene}: world XY proposals at native MPPI poses (not executed)', ylabel='path forward (m/s)', xlabel='time since goal (s)')
    ax.legend(fontsize=8, loc='upper right');ax.grid(alpha=.2)
ax = axes[2]
selected = [r for r in probes if r['case'] == 'feedback_long_hold']
ax.plot([int(r['cycle'])*.05 for r in selected], [float(r['vx']) if r['valid'] == '1' else float('nan') for r in selected], label='world vx', color='#1f77b4')
ax.axvspan(0., 6., color='#eeeeee', label='held obstacle')
ax.axhspan(-.02, .02, color='#87c9b5', alpha=.25, label='WAIT threshold')
ax.axvline(6., ls=':', color='black', label='clear')
ax.scatter([int(r['cycle'])*.05 for r in selected if r['valid'] == '0'], [0. for r in selected if r['valid'] == '0'], marker='x', c='red', label='unavailable solve')
ax.set(title='Same fixture: ideal world-velocity feedback while yaw spins at 0.6 rad/s', ylabel='world vx (m/s)', xlabel='virtual elapsed time (s)')
ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper left')
fig.savefig(output/'decision.png', dpi=150)
