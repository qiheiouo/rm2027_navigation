#!/usr/bin/env python3
"""Fixed scientific reveal labels; unknown physical stopping remains explicit."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.hashsalt']='rm2027_surface_reveal_v1'
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser();parser.add_argument('report',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--title',required=True);args=parser.parse_args();report=json.loads(args.report.read_text())
    if report['actual_safe_reveal_verdict']!='NOT ESTABLISHED':raise ValueError('explicit unknown stopping status required')
    scans=report['scan_records'];inventory=report['material_inventory'];times=[r['source_stamp'] for r in scans]
    tokens=[f"{r['face']}:{r['bin']}" for r in inventory]
    matrix=np.array([[token in row['reliable_material_keys'] for row in scans] for token in tokens],dtype=float)
    fig,(top,bottom)=plt.subplots(2,1,figsize=(11.6,9.2),gridspec_kw={'height_ratios':[1.8,1]})
    top.pcolormesh(times,np.arange(len(tokens)),matrix,shading='nearest',cmap=matplotlib.colors.ListedColormap(['#dddddd','#207e61']),vmin=0,vmax=1)
    top.set_yticks(np.arange(len(tokens)));top.set_yticklabels(tokens,fontsize=6);top.set_ylabel('Material bins (5 cm, offline box labels)')
    top.set_title(args.title+'\nGreen: reliable observed proxy; grey: no reliable evidence at this source scan',fontsize=11)
    events=report['events']
    for kind,marker,color,label in [('first_reliable','o','#286e9d','First reliable proxy'),('reappearance','x','#91628d','Intermittent reappearance')]:
        rows=[r for r in events if r['event_kind']==kind]
        bottom.scatter([r['confirmation_source_stamp'] for r in rows],[r['full_mechanical_gap_m'] for r in rows],s=18,marker=marker,color=color,label=label)
    bottom.axhline(.05,color='#bf4230',linestyle='--',label='Original mechanical margin: 0.05 m')
    for name,w in report['original_contact_witnesses'].items():
        if name not in ('first_mechanical_margin_violation','first_mechanical_projected_contact'):continue
        color='#c13d30';label='First mechanical margin' if 'margin' in name else 'First projected contact'
        for axis in (top,bottom):axis.axvline(w['time'],color=color,linestyle=':' if 'margin' in name else '--',alpha=.8,label=label if axis==bottom else None)
    bottom.set_ylabel('Gap to newly observed material segment (m)');bottom.set_xlabel('Source scan clock (simulation seconds)')
    bottom.set_xlim(top.get_xlim());bottom.grid(alpha=.2);bottom.legend(loc='upper right',fontsize=8)
    fig.text(.5,.016,f"{report['reliable_bins']}/{report['material_bins']} bins ever reliable; {report['never_reliably_revealed_bins']} never reliable. "
        'No MID360, actual surface-consumption, measured brake bound or end-to-end latency certificate.\n'
        'First visibility and later reappearance are distinct. Existing A/B trajectories are not moving-surface C tests; original task FAILED.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.055,1,1));fig.savefig(args.output.with_suffix('.png'),dpi=160)
    fig.savefig(args.output.with_suffix('.svg'),metadata={'Date':None});plt.close(fig)


if __name__=='__main__':main()
