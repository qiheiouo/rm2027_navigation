#!/usr/bin/env python3
"""Plot independently labelled physical trajectory and source-matched CV disk, offline."""
import argparse
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt'] = 'rm2027_contact_witness_v1'
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import yaml
from analyze_trial import load_truth_rows, rotation
from audit_scan_geometry import fixture


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); args = parser.parse_args(); root = args.trial
    witness = json.loads((root / 'contact_audit.json').read_text())['witnesses']['first_body_margin_violation']
    execution = json.loads((root / 'execution.json').read_text())
    physical = load_truth_rows(root, execution)
    cm = yaml.safe_load((root / 'profile.yaml').read_text())['local_costmap']['local_costmap']['ros__parameters']
    body = yaml.safe_load(cm['footprint']); pad = cm['footprint_padding']
    padded = [tuple(v + (pad if v > 0 else -pad if v < 0 else 0) for v in p) for p in body]
    scene = fixture(root / 'scene_inputs'); ax, ay = scene['actor_dimensions']; sx, sy = scene['static_dimensions']
    actor_shape = [(-ax/2,-ay/2),(ax/2,-ay/2),(ax/2,ay/2),(-ax/2,ay/2)]
    fig, (wide, near) = plt.subplots(1, 2, figsize=(11.5, 4.8))
    wide.plot([r['robot'][0] for r in physical], [r['robot'][1] for r in physical], color='#286e9d', label='Robot path (physical truth)')
    wide.add_patch(Polygon(rotation(scene['static_pose'], [(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)]),
                           color='#865139', alpha=.55, label='Static block'))
    wide.plot([r['obstacle'][0] for r in physical], [r['obstacle'][1] for r in physical], '--', color='#a73d2a', label='Moving actor path (truth)')
    wide.plot(5.6, 0, '*', color='#333333', markersize=12, label='Goal, not reached')
    wide.plot(*witness['physical_pose'][:2], 'o', color='#c53528', markersize=6)
    wide.set_title('Soft clearance trial: task and safety FAILED', fontsize=11)
    wide.set_xlim(-.3,6.1); wide.set_ylim(-1.4,1.8)
    near.add_patch(Polygon(rotation(witness['physical_pose'], padded), facecolor='none', edgecolor='#1d769d', linestyle='--', label='Padded footprint'))
    near.add_patch(Polygon(rotation(witness['physical_pose'], body), facecolor='#74a6c0', edgecolor='#286e9d', alpha=.65, label='Robot body (truth)'))
    near.add_patch(Polygon(rotation(witness['physical_actor_pose'], actor_shape), facecolor='#d4785e', edgecolor='#a73d2a', alpha=.65, label='Actor box (truth)'))
    near.plot(*witness['physical_actor_pose'][:2], 'x', color='#a73d2a')
    guard = witness['latest_observer_available_guard']; values = guard['statuses'][0]['values']
    source = witness['matched_guard_source_obstacle_receipt']
    rotation_yaw = float(values['world_transform_yaw']); c, s = math.cos(rotation_yaw), math.sin(rotation_yaw)
    for model in witness.get('source_matched_model_labels', []):
        track = next(t for t in source['tracks'] if t['id'] == model['id'])
        dt = witness['time'] - source['stamp']; px = track['xy'][0]+dt*track['vxy'][0]; py = track['xy'][1]+dt*track['vxy'][1]
        center = (float(values['world_transform_x'])+c*px-s*py, float(values['world_transform_y'])+s*px+c*py)
        near.add_patch(Circle(center, model['radius'], fill=False, color='#7f48ac', linewidth=1.5, label='Source-matched CV disk'))
        near.plot(*center, '+', color='#7f48ac', markersize=9)
    near.set_title(f"First sampled body gap < 0.05 m, t={witness['time']:.3f}s", fontsize=11)
    near.text(4.05, .98, f"Physical gap: {witness['physical_body_gap']:.4f} m\nPrior 0.2 s pose deviation: {witness['prior_window_max_pose_deviation_m']:.3g} m\nPrior 0.2 s max |command|: {witness['prior_window_max_abs_final_command']:.3g}", fontsize=9, va='top')
    near.annotate('Actor approaching', xy=(4.9,-.18), xytext=(5.13,-.83), arrowprops={'arrowstyle':'->','color':'#a73d2a'}, fontsize=8, color='#a73d2a')
    near.set_xlim(3.95,5.75); near.set_ylim(-1.0,1.05)
    for axis in (wide, near):
        axis.set_aspect('equal'); axis.set_xlabel('world / odom x [m]'); axis.set_ylabel('y [m]'); axis.grid(alpha=.2)
        axis.legend(loc='lower left', fontsize=7.5, framealpha=.9)
    fig.text(.5,.025,'Offline truth labels. CV recomputed at witness time from matched public source state; no exact optimizer / SG coverage claim.', ha='center', fontsize=8)
    fig.tight_layout(rect=(0,.055,1,1))
    fig.savefig(root/'contact_witness.png',dpi=160); fig.savefig(root/'contact_witness.svg',metadata={'Date':None}); plt.close(fig)


if __name__ == '__main__': main()
