#!/usr/bin/env python3
"""Actual installed inactive controller configuration/readback; zero goals.

Native final snapshot is disabled only in these inactive startup fixtures.
The registered Gazebo profile retains it and all original runtime parameters.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time
import yaml
from cost_accumulator_preflight import CostAccumulatorPreflight, configure_profile, ROLES
from test_controller_odom_preflight import stop_group


def main():
    import rclpy
    from lifecycle_msgs.srv import ChangeState
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    config=Path(__file__).resolve().parents[1]/'config'
    baseline=yaml.safe_load((config/'nav2_cv_dynamic_consumption.yaml').read_text())
    rclpy.init();node=rclpy.create_node('cost_accumulator_config_fixture');cases=[]
    try:
        for name in ('default_off','active_empty','existing_directory','wrong_order','zero_budget','unequal_budgets','same_directories','peer_disabled'):
            case=args.output/name;case.mkdir();profile=yaml.safe_load((config/'nav2_cv_cost_accumulator_evidence.yaml').read_text())
            follow=profile['controller_server']['ros__parameters']['FollowPath']
            if name!='default_off':configure_profile(profile,baseline,case)
            follow['NativeCycleSnapshotCritic']['enabled']=False
            if name=='existing_directory':
                directory=Path(follow[ROLES[0]]['output_directory']);directory.mkdir();(directory/'preserve').write_text('preserve')
            if name=='wrong_order':follow['critics'][1],follow['critics'][2]=follow['critics'][2],follow['critics'][1]
            if name=='zero_budget':
                for role in ROLES:follow[role]['max_records']=0
            if name=='unequal_budgets':follow[ROLES[1]]['max_records']=999
            if name=='same_directories':follow[ROLES[1]]['output_directory']=follow[ROLES[0]]['output_directory']
            if name=='peer_disabled':follow[ROLES[1]]['capture_enabled']=False
            params=case/'profile.yaml';params.write_text(yaml.safe_dump(profile,sort_keys=False))
            client=check=None
            with (case/'controller.log').open('x') as log:
                proc=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args','--params-file',str(params),'-p','use_sim_time:=false'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    client=node.create_client(ChangeState,'/controller_server/change_state')
                    if not client.wait_for_service(timeout_sec=8):raise RuntimeError('configure service unavailable')
                    request=ChangeState.Request();request.transition.id=1;future=client.call_async(request)
                    rclpy.spin_until_future_complete(node,future,timeout_sec=15)
                    if not future.done():raise RuntimeError('configure timeout')
                    configured=future.result().success
                    if configured!=(name in ('default_off','active_empty')):raise RuntimeError('unexpected configure result '+name)
                    readback=None
                    if configured:
                        check=CostAccumulatorPreflight(node,profile['controller_server']['ros__parameters'])
                        while readback is None:rclpy.spin_once(node,timeout_sec=.02);readback=check.poll()
                        if name=='active_empty':
                            for role in ROLES:
                                if sorted(p.name for p in Path(follow[role]['output_directory']).iterdir())!=['writer.lock']:
                                    raise RuntimeError('inactive observer unexpectedly captured data')
                    if name=='existing_directory' and (directory/'preserve').read_text()!='preserve':raise RuntimeError('existing evidence overwritten')
                    result={'name':name,'configured':configured,'readback':readback,
                        'verdict':'PASS' if configured else 'REJECTED AS REGISTERED','lifecycle':'inactive, never activated',
                        'goals_sent':0,'commands_published':0,'odometry_messages_published':0,'TF_published':0}
                    (case/'result.json').write_text(json.dumps(result,indent=2)+'\n');cases.append(result)
                finally:
                    stop_group(proc)
                    if client is not None:node.destroy_client(client)
                    if check is not None:node.destroy_client(check.client)
                    until=time.monotonic()+.5
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        (args.output/'result.json').write_text(json.dumps({'verdict':'PASS','cases':cases,
            'scope':'Actual installed configuration only; final snapshot disabled in inactive fixtures, no actual CostCritic score delta.'},indent=2)+'\n')
        print('Two actual inactive readbacks and six initialization rejection cases PASS',flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
