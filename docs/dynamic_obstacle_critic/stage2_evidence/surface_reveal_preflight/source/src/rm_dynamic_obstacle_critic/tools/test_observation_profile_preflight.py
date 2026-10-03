#!/usr/bin/env python3
"""Actual three-node readback and initialization rejection, zero goals.

Controllers remain inactive; native final snapshots alone are disabled in this
configuration fixture. Guard output is isolated and may publish timeout zeros.
No fixture commands, scan, odometry or TF are published.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import time
import yaml
from cost_accumulator_preflight import configure_profile
from observation_profile_preflight import (ObservationProfilePreflight, registered_profiles,
    transformed_controller)
from test_controller_odom_preflight import stop_group

CONFIG = Path(__file__).resolve().parents[1]/'config'
def load(name): return yaml.safe_load((CONFIG/name).read_text())
def save(path, data): path.write_text(yaml.safe_dump(data, sort_keys=False));return path


def main():
    import rclpy
    from lifecycle_msgs.srv import ChangeState
    parser=argparse.ArgumentParser();parser.add_argument('new_output',type=Path);args=parser.parse_args()
    args.new_output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('observation_profile_fixture');cases=[]
    baselines=dict(controller=load('nav2_cv_cost_accumulator_evidence.yaml'),tracker=load('tracker_cv.yaml'),
                   guard=load('guard_mechanical_footprint.yaml'))

    def start(package, executable, params, log, name=None):
        command=['ros2','run',package,executable,'--ros-args','--params-file',str(params)]
        if name:command+=['-r','__node:='+name]
        return subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

    def configure_controller():
        client=node.create_client(ChangeState,'/controller_server/change_state')
        try:
            if not client.wait_for_service(timeout_sec=8.):raise RuntimeError('controller configure service unavailable')
            request=ChangeState.Request();request.transition.id=1;future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=15.)
            if not future.done() or future.result() is None:raise RuntimeError('controller configure timeout')
            return future.result().success
        finally:node.destroy_client(client)

    def readback(controller,tracker,guard):
        check=ObservationProfilePreflight(node,controller,tracker,guard,use_sim_time=False,
                                           guard_output='/observation_profile_fixture/output')
        try:
            result=None
            while result is None:
                rclpy.spin_once(node,timeout_sec=.02);result=check.poll()
            return result
        finally:
            for client in check.clients.values():node.destroy_client(client)

    try:
        for label,mode in [('visible','visible_extent'),('nominal','nominal_diameter')]:
            case=args.new_output/label;case.mkdir()
            controller=load('nav2_cv_observation_'+label+'.yaml');tracker=load('tracker_observation_cv.yaml')
            guard=load('guard_observation_'+label+'.yaml')
            selection=registered_profiles(controller,tracker,guard,baselines,mode)
            configure_profile(controller,transformed_controller(load('nav2_cv_dynamic_consumption.yaml'),mode),case)
            cp=controller['controller_server']['ros__parameters'];tp=tracker['dynamic_obstacle_tracker_shadow']['ros__parameters']
            gp=guard['dynamic_safety_guard']['ros__parameters']
            cp['FollowPath']['NativeCycleSnapshotCritic']['enabled']=False
            cp['FollowPath']['DynamicObstacleCritic']['consumption_evidence_directory']=str((case/'dynamic_scores').resolve())
            gp['output_topic']='/observation_profile_fixture/output'
            processes=[];logs=[]
            try:
                for role,package,executable,data,name in [
                    ('controller','nav2_controller','controller_server',controller,None),
                    ('tracker','rm_dynamic_obstacle_tracking','dynamic_obstacle_tracker_node',tracker,'dynamic_obstacle_tracker_shadow'),
                    ('guard','rm_dynamic_obstacle_critic','dynamic_safety_guard',guard,None)]:
                    log=(case/(role+'.log')).open('x');logs.append(log)
                    processes.append(start(package,executable,save(case/(role+'.yaml'),data),log,name))
                if not configure_controller():raise RuntimeError('registered '+label+' controller rejected')
                actual=readback(cp,tp,gp)
                probes=[]
                for role,key,value in [('controller','FollowPath.DynamicObstacleCritic.nominal_obstacle_diameter',.4),
                                       ('tracker','prediction.max_speed',3.0),('guard','obstacle_radius_mode','wrong')]:
                    expected=[copy.deepcopy(v) for v in [cp,tp,gp]]
                    target=expected[['controller','tracker','guard'].index(role)]
                    if role=='controller':target['FollowPath']['DynamicObstacleCritic']['nominal_obstacle_diameter']=value
                    elif role=='tracker':target['prediction']['max_speed']=value
                    else:target[key]=value
                    error=None
                    try:readback(*expected)
                    except RuntimeError as failure:error=str(failure)
                    if error is None or 'configured parameter mismatch' not in error:raise RuntimeError('actual mismatch probe did not reject '+role)
                    probes.append(dict(node=role,parameter=key,expected=value,error=error,verdict='REJECTED'))
                result=dict(name=label,verdict='PASS',selection=selection,readback=actual,mismatch_probes=probes,
                            controller_lifecycle='inactive, never activated',goals_sent=0,fixture_commands_published=0,
                            fixture_odometry_published=0,fixture_TF_published=0,guard_output=gp['output_topic'])
                save(case/'result.yaml',result)
                (case/'result.json').write_text(json.dumps(result,indent=2)+'\n');cases.append(result)
                print(label+' actual three-node readback and three mismatch probes PASS',flush=True)
            finally:
                for process in reversed(processes):stop_group(process)
                for log in logs:log.close()
                until=time.monotonic()+.5
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)

        invalids=[('v1_nominal',{'input_schema':'rm_dynamic_obstacle_predictions/v1'}),
                  ('unknown_schema',{'input_schema':'unknown'}),('unknown_mode',{'obstacle_radius_mode':'unknown'}),
                  ('missing_diameter',{'nominal_obstacle_diameter':None}),('missing_prior',{'geometry_prior_source':None}),
                  ('blank_prior',{'geometry_prior_source':' \t\r\n\f\v'}),('negative_diameter',{'nominal_obstacle_diameter':-.1}),
                  ('over_budget_diameter',{'nominal_obstacle_diameter':3.1}),('visible_with_nominal',{'obstacle_radius_mode':'visible_extent'})]
        for label,changes in invalids:
            for role in ['controller','guard']:
                case=args.new_output/(label+'_'+role);case.mkdir()
                data=load('nav2_cv_observation_nominal.yaml' if role=='controller' else 'guard_observation_nominal.yaml')
                parameters=data['controller_server']['ros__parameters'] if role=='controller' else data['dynamic_safety_guard']['ros__parameters']
                selected=parameters['FollowPath']['DynamicObstacleCritic'] if role=='controller' else parameters
                if role=='controller':parameters['FollowPath']['NativeCycleSnapshotCritic']['enabled']=False
                else:parameters['output_topic']='/observation_profile_fixture/output'
                for key,value in changes.items():
                    if value is None:selected.pop(key)
                    else:selected[key]=value
                with (case/'process.log').open('x') as log:
                    process=start('nav2_controller' if role=='controller' else 'rm_dynamic_obstacle_critic',
                                  'controller_server' if role=='controller' else 'dynamic_safety_guard',
                                  save(case/'parameters.yaml',data),log)
                    try:
                        if role=='controller':
                            if configure_controller():raise RuntimeError('invalid controller configuration accepted: '+label)
                        else:
                            try:code=process.wait(timeout=6.)
                            except subprocess.TimeoutExpired:raise RuntimeError('invalid guard configuration remained running: '+label)
                            if code==0:raise RuntimeError('invalid guard exited successfully: '+label)
                    finally:stop_group(process)
                reason='invalid CV critic parameters' if role=='controller' else 'invalid guard route or physical parameters'
                if reason not in (case/'process.log').read_text():raise RuntimeError('rejection was not the registered model validation: '+label+' '+role)
                result=dict(name=label,node=role,verdict='REJECTED AS REGISTERED',reason=reason,goals_sent=0)
                (case/'result.json').write_text(json.dumps(result,indent=2)+'\n');cases.append(result)
                until=time.monotonic()+.3
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        report=dict(verdict='PASS',cases=cases,scope='Actual installed configuration/readback only; no geometry, future support or physical/task certificate.')
        (args.new_output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Two paired readbacks, six actual mismatches and eighteen native initialization rejections PASS',flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
