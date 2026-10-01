#!/usr/bin/env python3
"""Native input vs available canonical motion; no claim of exact odom consumer."""
import argparse
import bisect
import json
from pathlib import Path
import yaml
from native_snapshot_io import read_snapshot
from trial_io import rows


def analyze(root):
    config=yaml.safe_load((root/'profile.yaml').read_text())
    controller=config['controller_server']['ros__parameters']
    odom=sorted((r for r in rows(root,'observations.jsonl') if r['kind']=='canonical_odom'),key=lambda r:r['stamp'])
    times=[r['stamp'] for r in odom];records=[];missing=0
    thresholds=[controller[k] for k in ['min_x_velocity_threshold','min_y_velocity_threshold','min_theta_velocity_threshold']]
    for path in sorted((root/'native_cycles').glob('cycle_*.json'),key=lambda p:int(p.stem.split('_')[1])):
        meta,_=read_snapshot(path);stamp=meta['pose_stamp_sec']+meta['pose_stamp_nanosec']*1e-9
        index=bisect.bisect_right(times,stamp)-1
        if index<0 or not 0<=stamp-odom[index]['stamp']<=.04:missing+=1;continue
        canonical=odom[index]
        moving=any(abs(v)>limit for v,limit in zip(canonical['velocity'],thresholds))
        records.append({'ordinal':meta['ordinal'],'source_pose_stamp':stamp,'native_speed':meta['speed'],
                        'canonical_odom_stamp':canonical['stamp'],'canonical_velocity':canonical['velocity'],
                        'source_age':stamp-canonical['stamp'],'canonical_motion_above_threshold':moving,
                        'native_input_all_zero':all(v==0 for v in meta['speed'])})
    zero=bool(records) and all(r['native_input_all_zero'] for r in records)
    moving=[r for r in records if r['canonical_motion_above_threshold']]
    failed=zero and bool(moving)
    return {'verdict':'VELOCITY CONTRACT FAILED' if failed else 'REQUIRES CONSUMER VALIDATION',
            'scope':'recorded native state.speed vs source-pose-time available canonical odometry; not exact native odom subscriber/averaging identity',
            'captures_checked':len(records),'missing_source_odom':missing,'all_native_inputs_zero':zero,
            'canonical_motion_above_threshold_cycles':len(moving),
            'max_abs_canonical_component':max(abs(v) for r in records for v in r['canonical_velocity']),
            'max_abs_canonical_by_component':{name:max(abs(r['canonical_velocity'][index]) for r in records)
                                             for index,name in enumerate(['vx_m_s','vy_m_s','wz_rad_s'])},
            'controller_odom_topic_explicit':controller.get('odom_topic'),
            'bt_navigator_odom_topic':config['bt_navigator']['ros__parameters'].get('odom_topic'),
            'velocity_smoother_odom_topic':config['velocity_smoother']['ros__parameters'].get('odom_topic'),
            'first_moving_canonical_with_zero_native':next((r for r in moving if r['native_input_all_zero']),None),
            'records':records,
            'limits':'Exact preservation of zero native input does not establish effective physical measurement. Instantaneous canonical twist need not equal native averaged/thresholded speed. All-zero input across substantial motion invalidates a physical measured-speed witness; actual subscriber/parameter readback must locate cause. No old physical verdict or saved native proof is overwritten.'}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('archive',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    result=analyze(args.archive)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='records'}))


if __name__=='__main__':main()
