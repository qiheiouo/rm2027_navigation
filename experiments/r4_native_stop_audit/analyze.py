#!/usr/bin/env python3
"""Original ROS stop receipts and plant pose audit; no runtime or state replacement."""
import argparse
import collections
import csv
import hashlib
import json
import math
import pathlib
import shutil
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def rows(path):
    with path.open() as f:return list(csv.DictReader(f))


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def command(r):return tuple(float(r[k]) for k in ['vx','vy','wz'])
def zero(r):return max(abs(x) for x in command(r))<=1e-9


def stats(values):
    v=list(values)
    return dict(n=len(v),median=float(np.median(v)) if v else None,p95=float(np.percentile(v,95)) if v else None,
                min=min(v) if v else None,max=max(v) if v else None)


def normalized(element):
    return [element.tag,sorted(element.attrib.items()),(element.text or '').strip(),[normalized(e) for e in element]]


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=pathlib.Path)
    p.add_argument('a13',type=pathlib.Path);p.add_argument('decoded_odom',type=pathlib.Path)
    args=p.parse_args();root=pathlib.Path(__file__).resolve().parents[2];summary={};derived=[]
    original=root/'src/rm_simulation/worlds/phase1_omni.sdf'
    original_robot=ET.parse(original).getroot().find("world/model[@name='rm_sentry_2027']")
    for scene in ['S0','S1','S2']:
        old=root/'build/r4_runtime_shadow_20261005/assets'/f'{scene}.sdf'
        new=args.output/'corrected_assets'/f'{scene}.sdf'
        assert normalized(ET.parse(old).getroot().find("world/model[@name='rm_sentry_2027']"))==normalized(original_robot)
        assert normalized(ET.parse(new).getroot().find("world/model[@name='rm_sentry_2027']"))==normalized(original_robot)
        assert 'ns0:expressed_in' in old.read_text() and 'ignition:expressed_in' in new.read_text()
    before=[line.split('\t') for line in (args.output/'sdf_before.tsv').read_text().splitlines()]
    after=[line.split('\t') for line in (args.output/'sdf_after.tsv').read_text().splitlines()]
    assert all(r[2:4]==['1','0'] for r in before[:4])
    assert len(before[4:])==12 and all(r[2:4]==['0','1'] for r in before[4:])
    assert len(after)==12 and all(r[2:4]==['1','0'] for r in after)
    # Known wheel roll and revolute axis; static tangent projection, not contact simulation.
    intended=[];fallback=[]
    for link in original_robot.findall('link'):
        f=link.find('collision/surface/friction/ode/fdir1')
        if f is None:continue
        v=np.array([float(x) for x in f.text.split()]);roll=float(link.find('pose').text.split()[3])
        rx=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
        intended.append((v[:2]/np.linalg.norm(v[:2])).tolist())
        for angle in [0.,.37,1.,1.7]:
            rz=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
            tangent=(rx@rz@v)[:2]
            fallback.append((tangent/np.linalg.norm(tangent)).tolist())
    source_summary=dict(robot_expanded_XML_and_numeric_tree_unchanged=True,original_literal_frame_attributes=4,
        A13_literal_frame_attributes=0,corrected_literal_frame_attributes=12,
        intended_planar_direction_rank=int(np.linalg.matrix_rank(np.array(intended),tol=1e-9)),
        fallback_wheel_local_planar_direction_rank=int(np.linalg.matrix_rank(np.array(fallback),tol=1e-9)),
        meaning='Static known-plane wheel-axis projection only; no wheel/contact/Gazebo delivery trace or physical cause certificate.')
    fig,axes=plt.subplots(2,3,figsize=(13,7))
    for scene in ['S0','S1','S2']:
        cmds=rows(args.output/(scene+'_commands.csv'))
        refs=rows(args.a13/(scene+'_references.csv'));odom=rows(args.decoded_odom/(scene+'_odometry.csv'))
        events={e['kind']:e for e in json.loads((args.a13/(scene+'_events.json')).read_text())['events']}
        start=events['goal_accepted']['ROS_ns'];end=events['observation_end']['ROS_ns']
        goal=events.get('native_goal_result',{}).get('ROS_ns')
        per_topic={t:[r for r in cmds if r['topic']==t] for t in ['/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel']}
        for topic,kind in [('/cmd_vel_nav','mppi_upstream'),('/cmd_vel','actual_output')]:
            a=[command(r) for r in per_topic[topic]];b=[command(r) for r in refs if r['kind']==kind]
            assert a==b,(scene,topic,'bag/reference values')
        upstream=per_topic['/cmd_vel'];relay=per_topic['/simulation/chassis/cmd_vel']
        assert len(relay)>=len(upstream)
        assert [command(r) for r in relay[:len(upstream)]]==[command(r) for r in upstream]
        assert all(zero(r) for r in relay[len(upstream):])
        topic_summary={}
        for topic,rs in per_topic.items():
            nz=[j for j,r in enumerate(rs) if not zero(r)]
            j=nz[-1]+1 if nz else 0;stop=rs[j] if j<len(rs) else None
            topic_summary[topic]=dict(rows=len(rs),nonzero_rows=len(nz),
                last_nonzero_recorder_clock_before_ns=int(rs[nz[-1]]['recorder_clock_before_ns']) if nz else None,
                permanent_zero_recorder_clock_bracket_ns=[int(stop['recorder_clock_before_ns']),int(stop['recorder_clock_after_ns'])] if stop else None,
                last_recorder_clock_bracket_ns=[int(rs[-1]['recorder_clock_before_ns']),int(rs[-1]['recorder_clock_after_ns'])])
        result=dict(native_goal_source_observer_ns=goal,topics=topic_summary,relay_value_mismatch=0,
                    extra_relay_zero_rows=len(relay)-len(upstream))
        stop=topic_summary['/simulation/chassis/cmd_vel']['permanent_zero_recorder_clock_bracket_ns']
        post=[]
        if goal and stop:
            # Fixed analysis window leaves one second after the last transition to zero.
            cutoff=max(goal,stop[1])+10**9
            post=[r for r in odom if cutoff<=int(r['source_ns'])<=end]
            result['post_zero_plus_1s']=dict(source_window_ns=[int(post[0]['source_ns']),int(post[-1]['source_ns'])],
                measured_vx=stats(float(r['vx']) for r in post),measured_vy=stats(float(r['vy']) for r in post),
                measured_wz=stats(float(r['wz']) for r in post),
                displacement_m=math.hypot(float(post[-1]['x'])-float(post[0]['x']),float(post[-1]['y'])-float(post[0]['y'])))
        body_diffs=[]
        for a,b in zip(odom,odom[1:]):
            ns=int(b['source_ns']);dt=(ns-int(a['source_ns']))/1e9
            if not(start<=ns<=end and 0<dt<=.1):continue
            yaw=float(a['yaw'])+.5*math.remainder(float(b['yaw'])-float(a['yaw']),2*math.pi)
            dx=(float(b['x'])-float(a['x']))/dt;dy=(float(b['y'])-float(a['y']))/dt
            vx=math.cos(yaw)*dx+math.sin(yaw)*dy;vy=-math.sin(yaw)*dx+math.cos(yaw)*dy
            wz=math.remainder(float(b['yaw'])-float(a['yaw']),2*math.pi)/dt
            row=dict(scene=scene,source_ns=ns,pose_derived_body_vx=vx,pose_derived_body_vy=vy,pose_derived_wz=wz,
                reported_midpoint_vx=.5*(float(a['vx'])+float(b['vx'])),reported_midpoint_vy=.5*(float(a['vy'])+float(b['vy'])),
                reported_midpoint_wz=.5*(float(a['wz'])+float(b['wz'])))
            body_diffs.append(row);derived.append(row)
        result['pose_derivative_midpoint_linear_error_mps']=stats(math.hypot(r['pose_derived_body_vx']-r['reported_midpoint_vx'],r['pose_derived_body_vy']-r['reported_midpoint_vy']) for r in body_diffs)
        if post:
            a=int(post[0]['source_ns']);b=int(post[-1]['source_ns'])
            result['post_zero_pose_derivative_linear_error_mps']=stats(math.hypot(r['pose_derived_body_vx']-r['reported_midpoint_vx'],r['pose_derived_body_vy']-r['reported_midpoint_vy']) for r in body_diffs if a<=r['source_ns']<=b)
        summary[scene]=result
        if scene=='S1':
            for j,k in enumerate(['vx','vy','wz']):
                for topic,rs in per_topic.items():
                    axes[0,j].plot([(int(r['recorder_clock_before_ns'])-goal)/1e9 for r in rs],[float(r[k]) for r in rs],
                        label=topic,linewidth=1,alpha=.7)
                selected=[r for r in odom if goal-10**9<=int(r['source_ns'])<=end]
                axes[1,j].plot([(int(r['source_ns'])-goal)/1e9 for r in selected],[float(r[k]) for r in selected],label='recorded Odometry',linewidth=1)
                axes[1,j].plot([(r['source_ns']-goal)/1e9 for r in body_diffs],[r['pose_derived_body_'+k] if k!='wz' else r['pose_derived_wz'] for r in body_diffs],label='pose derivative in body frame',linewidth=.8,alpha=.7)
                for i in [0,1]:
                    ax=axes[i,j];ax.set_xlim(-.5,(end-goal)/1e9);ax.grid(alpha=.2);ax.legend(fontsize=7)
                    ax.axvline(0,color='gray',linestyle='--');ax.axvline((stop[1]-goal)/1e9,color='black',linestyle=':',alpha=.5)
                    ax.set_ylabel(('ROS command ' if i==0 else 'Measured ')+k+(' (rad/s)' if k=='wz' else ' (m/s)'))
                axes[1,j].set_xlabel('Original ROS seconds since native goal result')
    fig.suptitle('A15 original S1: ROS zero output and continued plant lateral motion; no new runtime')
    fig.tight_layout();fig.savefig(args.output/'native_stop.png',dpi=140);plt.close(fig)
    with (args.output/'pose_velocity_diagnostics.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(derived[0]),lineterminator='\n');w.writeheader();w.writerows(derived)
    record=dict(base_commit='ce87459b531e6bbff4009d62aa0eb9c6756b9c4d',static_inspection=source_summary,scenes=summary,
                interpretation='Native ROS zero forwarding observed; physical response after ROS/Gazebo boundary lacks delivery/wheel/contact trace. A13 generated physics was altered by namespace prefix serialization.')
    (args.output/'summary.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
    evidence=pathlib.Path(__file__).parent/'evidence';evidence.mkdir(exist_ok=True)
    for name in ['summary.json','native_stop.png','pose_velocity_diagnostics.csv','decode_manifest.json','sdf_before.tsv','sdf_after.tsv','installed_packages.tsv','physics_libraries.sha256','installed_literal.txt']+[s+'_commands.csv' for s in ['S0','S1','S2']]:
        if name=='installed_packages.tsv':
            # dpkg wildcards include aliases without a reported Version; preserve raw build TSV.
            (evidence/name).write_text((args.output/name).read_text().replace('\t\n','\tNA\n'))
        else:
            shutil.copyfile(args.output/name,evidence/name)
    shutil.copyfile(args.output/'upstream/sources.json',evidence/'upstream_sources.json')
    files=list(pathlib.Path(__file__).parent.glob('*.py'))+list(pathlib.Path(__file__).parent.glob('*.cpp'))+[root/'experiments/r4_runtime_shadow/prepare.py']
    provenance=dict(image='sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3',
        sources={str(p.relative_to(root)):sha(p) for p in files},
        generated_assets={str(p.resolve().relative_to(root)):sha(p) for p in (args.output/'corrected_assets').iterdir() if p.is_file()},
        inspector_binary_sha256=sha(args.output/'inspect_sdf'),linked_libraries_record_sha256=sha(args.output/'inspect_sdf_ldd.txt'),
        upstream_record_sha256=sha(evidence/'upstream_sources.json'),evidence={p.name:sha(p) for p in evidence.iterdir() if p.is_file() and p.name!='provenance.json'},
        raw_installed_packages_sha256=sha(args.output/'installed_packages.tsv'),
        installed_packages_normalization='Unreported wildcard alias Version is NA; original raw TSV remains in build output.',
        logs={p.name:sha(p) for p in args.output.glob('*.log') if p.name!='analysis.log'},
        execution=dict(new_ROS_nodes=0,new_Gazebo_runs=0,Follow_calls=0,algorithm_changes=False),
        static_regression='PASS',old_bag_command_to_reference_and_relay_regression='PASS')
    (evidence/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
