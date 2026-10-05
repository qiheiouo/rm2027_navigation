"""A20 finite conditional command-transition audit; no runtime or solver calls."""
import argparse
import bisect
import csv
import importlib.util
import json
import math
import sys
sys.dont_write_bytecode = True
from pathlib import Path


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def advance(pose, vx, vy, wz, duration):
    # Independent offline evaluation of the existing A09 held-twist expression.
    # Each interval is <=50ms. This is a command model, not a physical forecast.
    assert 0 <= duration <= .05+1e-12
    angle = wz*duration
    sinc = math.sin(angle)/angle if angle else 1.
    cosc = 2*math.sin(angle/2)**2/angle if angle else 0.
    dx, dy = duration*(sinc*vx-cosc*vy), duration*(cosc*vx+sinc*vy)
    x, y, yaw = pose
    return x+math.cos(yaw)*dx-math.sin(yaw)*dy, y+math.sin(yaw)*dx+math.cos(yaw)*dy, yaw+angle


def transition(pose, vx, vy, previous_wz, phase, dt, decrement, horizon):
    # Assume current OPEN_LOOP/no-scaling owner requests zero continuously.
    # First tick can be immediate or one nominal timer period later.
    time, wz, ticks = 0., previous_wz, 0
    current = pose
    if phase:
        current = advance(current,vx,vy,wz,phase)
        time = phase
    while wz != 0:
        wz = wz+max(-decrement,min(decrement,-wz))
        ticks += 1
        if wz == 0:
            zero_time = time
            break
        current = advance(current,vx,vy,wz,dt)
        time += dt
    else:
        zero_time = time
    while time < horizon-1e-12:
        length = min(dt,horizon-time)
        current = advance(current,vx,vy,0.,length)
        time += length
    return current, zero_time, ticks


def yaw_error(yaw):
    # Existing A16 goal is (4,0,0); no terminal-yaw action is being evaluated.
    return abs(math.atan2(math.sin(yaw),math.cos(yaw)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    spec = importlib.util.spec_from_file_location('a17_support',args.root/'experiments/r4_rotation_scope_audit/analyze.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # Reuse support extrema; main()/hash code is not called.
    stats, support = module.stats, module.support
    text = (args.root/'src/rm_nav_config/config/nav2_phase1_5_mppi.yaml').read_text()
    import re
    smoother = text.split('velocity_smoother:',1)[1].split('local_costmap:',1)[0]
    dt = 1/float(re.search(r'smoothing_frequency:\s*([\d.]+)',smoother)[1])
    decel = -json.loads(re.search(r'max_decel:\s*(\[[^\n]+\])',smoother)[1])[2]
    assert 'feedback: OPEN_LOOP' in smoother and 'scale_velocities: false' in smoother
    assert json.loads(re.search(r'deadband_velocity:\s*(\[[^\n]+\])',smoother)[1])[2] == 0
    assert dt == .05 and decel == 2.
    footprint = json.loads(re.search(r'footprint:\s*"(\[[^\n]+\])"',text)[1])
    padding = float(re.search(r'footprint_padding:\s*([\d.]+)',text)[1])
    assert max(abs(a-b) for a,b in zip(advance((0,0,0),.4,.1,0,.05),(.02,.005,0))) < 1e-12
    check, zt, ticks = transition((0,0,0),0,0,.25,.05,.05,.1,1.5)
    assert ticks == 3 and abs(zt-.15)<1e-12 and abs(check[2]-.0225)<1e-12
    evidence = args.root/'experiments/r4_corrected_runtime_shadow/evidence'
    a19 = rows(args.root/'experiments/r4_aligned_follow/evidence/replay.csv')
    results, scenes = [], {}
    for scene in ('S1','S2'):
        selected = {r['cycle']:r for r in a19 if r['scene']==scene}
        commands = sorted([r for r in rows(evidence/f'{scene}_references.csv') if r['kind']=='actual_output'],
                          key=lambda r:int(r['receipt_ros_ns']))
        stamps = [int(r['receipt_ros_ns']) for r in commands]
        for source in rows(evidence/f'{scene}_cycles.csv'):
            if source['cycle'] not in selected:
                continue
            epoch = int(source['acquire_ros_ns'])
            index = bisect.bisect_right(stamps,epoch)-1
            assert index >= 0
            command = commands[index]
            previous_wz = float(command['wz'])
            age_ms = (epoch-stamps[index])*1e-6
            assert age_ms <= 100  # Already recorded A19 proxy; not an owner grant.
            vx, vy, measured_wz = (float(source[k]) for k in ('measured_vx','measured_vy','measured_wz'))
            pose = float(source['x']),float(source['y']),float(source['yaw'])
            age = (epoch-int(source['pose_source_ns']))*1e-9
            while age > 1e-12:
                length = min(dt,age)
                pose = advance(pose,vx,vy,measured_wz,length)
                age -= length
            fixed = pose
            for _ in range(30):
                fixed = advance(fixed,vx,vy,0.,dt)
            b = support(footprint,padding,pose[2],pose[2])
            value = {'scene':scene,'cycle':int(source['cycle']),'epoch_ns':epoch,'proxy_receipt_age_ms':age_ms,
                     'proxy_previous_wz':previous_wz,'model_epoch_yaw':pose[2],
                     'epoch_yaw_outside_goal_tolerance':yaw_error(pose[2])>.2}
            for phase,label in ((0.,'immediate'),(dt,'one_period')):
                final, zero_time, ticks = transition(pose,vx,vy,previous_wz,phase,dt,decel*dt,1.5)
                s = support(footprint,padding,min(pose[2],final[2]),max(pose[2],final[2]))
                value.update({f'{label}_zero_ms':zero_time*1000,f'{label}_ticks':ticks,
                    f'{label}_yaw_shift_rad':abs(final[2]-pose[2]),
                    f'{label}_constant_body_1500ms_endpoint_delta_m':math.hypot(final[0]-fixed[0],final[1]-fixed[1]),
                    f'{label}_swept_support_expansion_m':max(0,b[0]-s[0],s[1]-b[1],b[2]-s[2],s[3]-b[3]),
                    f'{label}_terminal_yaw_outside_tolerance':yaw_error(final[2])>.2})
            results.append(value)
        chosen = [r for r in results if r['scene']==scene]
        scenes[scene] = {'samples':len(chosen), 'proxy_previous_wz_nonzero':sum(r['proxy_previous_wz']!=0 for r in chosen),
            'more_than_one_tick':sum(r['immediate_ticks']>1 for r in chosen),
            'even_ideal_immediate_zero_exceeds_75ms':sum(r['immediate_zero_ms']>75 for r in chosen),
            'one_period_phase_zero_exceeds_75ms':sum(r['one_period_zero_ms']>75 for r in chosen),
            'epoch_yaw_outside_goal_tolerance':sum(r['epoch_yaw_outside_goal_tolerance'] for r in chosen),
            'one_period_terminal_yaw_outside_tolerance':sum(r['one_period_terminal_yaw_outside_tolerance'] for r in chosen),
            'metrics':{key:stats(r[key] for r in chosen) for key in chosen[0] if any(key.endswith(suffix) for suffix in
                ('zero_ms','ticks','yaw_shift_rad','endpoint_delta_m','expansion_m'))}}
        # Native finish observation from the same completed run, not R4 completion.
        events = json.loads((evidence/f'{scene}_events.json').read_text())['events']
        goal = next(e['ROS_ns'] for e in events if e['kind']=='native_goal_result')
        odom = min(rows(evidence/f'{scene}_odometry.csv'),key=lambda r:abs(int(r['source_ns'])-goal))
        scenes[scene]['native_goal_pose_observation'] = {k:float(odom[k]) for k in ('x','y','yaw','wz')}
    with (args.output/'transitions.csv').open('w') as stream:
        writer = csv.DictWriter(stream,list(results[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(results)
    summary = {'stage':'Research A20','baseline':'c744d4e7','scope':'source review and conditional geometry; zero runtime calls',
        'assumptions':{'previous_wz':'nearest earlier actual_output receipt proxy; not send/last-applied evidence',
            'target':'persistent zero angular target with no scaling/deadband, nominal timer, no replacement/cancel',
            'phase_seconds':[0.,dt],'clock':'nominal 1:1 ROS/model and wall progression; actual wall-timer phase is not measured',
            'deceleration_rad_s2':decel,'dt_seconds':dt,
            'translation':'hold source measured body velocity for 1.5s; not solved controls or actual motion',
            'lease_comparison':'ideal target arrival at epoch; excludes compute/transport/jitter; not lease PASS'},
        'scenes':scenes,'decision':'Modify: zero-target ownership is reusable but A19 constant future yaw cannot express the nonzero-history transition. No production wiring.'}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
