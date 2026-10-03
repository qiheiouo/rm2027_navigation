#!/usr/bin/env python3
"""Compare point/boundary costs and whole polygon contacts on fixed model paths."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(report_path,cpp_path,output):
    report=json.loads(report_path.read_text());cpp=[json.loads(l) for l in cpp_path.read_text().splitlines()][1:]
    plt.rcParams.update({'svg.hashsalt':'rm_native_cost_map_contract_v1','font.size':10})
    fig,axes=plt.subplots(1,3,figsize=(14,4.9),sharey=True)
    for ax,trace,title in zip(axes,[5,7,36],['Local CV: after SG','Future circle: after SG','Individual SG: raw row185']):
        row=report['rows'][trace];points=[r for r in cpp if r['trace']==trace]
        ax.plot([p['future_seconds'] for p in points],[p['native_footprint_cost'] for p in points],color='#168aad',label='SDK boundary raster')
        ax.plot([p['future_seconds'] for p in points],[p['center_raw'] for p in points],color='#7d8597',label='Center raw')
        ax.axhline(203,color='#bc1f2c',linestyle='--',linewidth=1,label='Original raw203 gate value')
        contact=row['first_whole_polygon_contact'];first=row['first_native_footprint_ge203']
        if contact:
            ax.axvline(contact['future_seconds'],color='#bc1f2c',linestyle=':',linewidth=1)
        text=f"Boundary max {row['maximum_native_footprint_cost']}\nWhole polygon +{contact['future_seconds']:.3f}s" if contact else f"Boundary max {row['maximum_native_footprint_cost']}"
        text+='\nSDK >=203: '+(f"+{first['future_seconds']:.3f}s" if first else 'none')
        ax.text(.04,.72,text,transform=ax.transAxes,fontsize=9,bbox={'facecolor':'white','edgecolor':'#dddddd','alpha':.9})
        ax.set(title=title,xlabel='Future time from actual score (s)',xlim=(0,3.03),ylim=(-12,270));ax.grid(alpha=.18)
    axes[0].set_ylabel('Frozen grid cost byte');axes[0].legend(loc='lower left',fontsize=8)
    fig.suptitle('Source298: separate center, boundary raster and whole polygon definitions',fontsize=12)
    fig.text(.5,.025,'Interpolated three-second model paths. Original raw203 interval gates remain FAILED. Actual CostCritic score/state unknown.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.07,1,.94]);output.mkdir()
    fig.savefig(output/'native_cost_map_contract.png',dpi=160);fig.savefig(output/'native_cost_map_contract.svg',metadata={'Date':None});plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['report','cpp','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();plot(args.report,args.cpp,args.output)
