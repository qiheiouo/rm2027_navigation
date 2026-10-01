#!/usr/bin/env python3
"""Independent mechanical event figure, with reported model epoch kept explicit."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt']='rm2027_mechanical_contract_v1'
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Polygon
import yaml
from analyze_trial import load_truth_rows,rotation
from audit_mechanical_footprint import components
from audit_scan_geometry import fixture
from fixture_robot_geometry import base_body_polygon


def main():
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path)
    parser.add_argument('--title',default='Mechanical footprint trial: safety / task FAILED')
    parser.add_argument('--fit-near-geometry',action='store_true')
    args=parser.parse_args();root=args.trial
    execution=json.loads((root/'execution.json').read_text());analysis=json.loads((root/'analysis.json').read_text())
    mechanical=json.loads((root/'mechanical_footprint_audit.json').read_text())
    contact=json.loads((root/'mechanical_contact_audit.json').read_text())
    witness=contact['witnesses']['first_mechanical_margin_violation'];event=witness['mechanical_projection_trigger']
    truth=load_truth_rows(root,execution);scene=fixture(root/'scene_inputs');ax,ay=scene['actor_dimensions']
    actor=[(-ax/2,-ay/2),(ax/2,-ay/2),(ax/2,ay/2),(-ax/2,ay/2)]
    cm=yaml.safe_load((root/'profile.yaml').read_text())['local_costmap']['local_costmap']['ros__parameters']
    padded=[tuple(v+(cm['footprint_padding'] if v>0 else -cm['footprint_padding']) for v in p)
            for p in yaml.safe_load(cm['footprint'])]
    fig,(wide,near)=plt.subplots(1,2,figsize=(12.2,5.3))
    wide.plot([r['robot'][0] for r in truth],[r['robot'][1] for r in truth],color='#286e9d',label='Robot path (physical truth)')
    wide.plot([r['obstacle'][0] for r in truth],[r['obstacle'][1] for r in truth],'--',color='#a73d2a',label='Actor path (truth)')
    sx,sy=scene['static_dimensions'];wide.add_patch(Polygon(rotation(scene['static_pose'],[(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)]),color='#865139',alpha=.55,label='Static block'))
    wide.plot(5.6,0,'*',color='#333333',markersize=12,label='Goal, not reached')
    wide.plot(*witness['physical_pose'][:2],'o',color='#c53528',markersize=6,label='First mechanical margin violation')
    wide.set_xlim(-.3,6.1);wide.set_ylim(-1.4,1.8)
    wide.set_title(args.title,fontsize=11)
    near.add_patch(Polygon(rotation(witness['physical_pose'],padded),fill=False,edgecolor='#1d769d',linestyle='--',label='Configured padded envelope'))
    near.add_patch(Polygon(rotation(witness['physical_pose'],base_body_polygon(root/'scene_inputs')),facecolor='#74a6c0',edgecolor='#286e9d',alpha=.65,label='Actual base box (truth)'))
    first=True
    for shape in components(root/'scene_inputs'):
        if shape['kind']!='circle':continue
        center=rotation(witness['physical_pose'],[shape['center']])[0]
        near.add_patch(Circle(center,shape['radius'],fill=False,color='#4b5358',label='Full wheel projections' if first else None));first=False
    near.add_patch(Polygon(rotation(witness['physical_actor_pose'],actor),facecolor='#d4785e',edgecolor='#a73d2a',alpha=.65,label='Actor box at event (truth)'))
    guard=witness['latest_observer_available_guard'];epoch=guard['stamp']
    for model in witness.get('source_matched_model_labels',[]):
        near.add_patch(Circle(model['CV_center_at_guard_epoch'],model['radius'],fill=False,color='#7f48ac',linewidth=1.5,label=f'Source-matched CV disk at {epoch:.3f} s'))
    xy=witness['physical_pose'][:2];near.set_xlim(xy[0]-.65,xy[0]+1.05);near.set_ylim(xy[1]-.85,xy[1]+.8)
    if args.fit_near_geometry:
        vertices=rotation(witness['physical_actor_pose'],actor)+rotation(witness['physical_pose'],padded)
        for model in witness.get('source_matched_model_labels',[]):
            x,y=model['CV_center_at_guard_epoch'];radius=model['radius']
            vertices.extend([(x-radius,y-radius),(x+radius,y+radius)])
        lo,hi=near.get_ylim();near.set_ylim(min(lo,min(p[1] for p in vertices)-.1),max(hi,max(p[1] for p in vertices)+.1))
    near.set_title(f"First full mechanical gap < 0.05 m, t={witness['time']:.3f} s",fontsize=11)
    note=(f"Wheel gap: {event['gap']:.4f} m; base: {witness['physical_body_gap']:.4f} m\n"
          f"Prior 0.2 s: pose deviation {witness['prior_window_max_pose_deviation_m']:.3g} m, "
          f"max |command| {witness['prior_window_max_abs_final_command']:.3g}\n"
          f"Latest guard: {guard['statuses'][0]['reason']}, TTC={guard['statuses'][0]['values']['TTC']}")
    near.text(.02,.98,note,transform=near.transAxes,fontsize=8.5,va='top')
    for axis in (wide,near):
        axis.set_aspect('equal',adjustable='box');axis.set_xlabel('world x (m)');axis.set_ylabel('world y (m)');axis.grid(alpha=.18)
        axis.legend(loc='lower left',fontsize=7.4,framealpha=.9)
    fig.text(.5,.025,'SDF planar projections and physical truth are offline labels. CV disk is from the reported guard epoch; no native SG / coverage claim.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.06,1,1))
    fig.savefig(root/'mechanical_contract_witness.png',dpi=160)
    fig.savefig(root/'mechanical_contract_witness.svg',metadata={'Date':None})
    plt.close(fig)


if __name__=='__main__':main()
