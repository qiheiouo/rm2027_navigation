#!/usr/bin/env python3
"""Actual installed controller configure/readback, never activated or commanded."""
import argparse
import json
from pathlib import Path
import subprocess
import time
from dynamic_consumption_preflight import DynamicConsumptionPreflight
from test_controller_odom_preflight import stop_group


def main():
    import rclpy
    from lifecycle_msgs.srv import ChangeState
    parser = argparse.ArgumentParser(); parser.add_argument('output', type=Path); args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = Path(__file__).resolve().parents[1]/'config/nav2_cv_dynamic_consumption.yaml'
    rclpy.init(); node = rclpy.create_node('dynamic_consumption_config_fixture'); cases = []
    try:
        for name in ('empty_directory', 'existing_directory', 'invalid_budget'):
            directory = (args.output/name/'scores').resolve(); directory.mkdir(parents=True)
            if name == 'existing_directory': (directory/'preserve').write_text('preserve')
            maximum = 0 if name == 'invalid_budget' else 400
            client = check = None
            with (args.output/(name+'.log')).open('x') as log:
                proc = subprocess.Popen(['ros2', 'run', 'nav2_controller', 'controller_server', '--ros-args',
                    '--params-file', str(profile), '-p', 'use_sim_time:=false',
                    '-p', 'FollowPath.NativeCycleSnapshotCritic.enabled:=false',
                    '-p', 'FollowPath.DynamicObstacleCritic.consumption_evidence_directory:='+str(directory),
                    '-p', 'FollowPath.DynamicObstacleCritic.consumption_evidence_max_records:='+str(maximum)],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    client = node.create_client(ChangeState, '/controller_server/change_state')
                    if not client.wait_for_service(timeout_sec=8): raise RuntimeError('configure service unavailable')
                    request = ChangeState.Request(); request.transition.id = 1; future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                    if not future.done(): raise RuntimeError('configure timeout')
                    configured = future.result().success
                    if configured != (name == 'empty_directory'): raise RuntimeError('unexpected configure result: '+name)
                    readback = None
                    if configured:
                        check = DynamicConsumptionPreflight(node, str(directory), maximum)
                        while readback is None:
                            rclpy.spin_once(node, timeout_sec=.02); readback = check.poll()
                        if sorted(p.name for p in directory.iterdir()) != ['writer.lock']:
                            raise RuntimeError('inactive evidence directory contents changed')
                    if name == 'existing_directory' and (directory/'preserve').read_text() != 'preserve':
                        raise RuntimeError('existing evidence overwritten')
                    cases.append({'name': name, 'configured': configured, 'readback': readback,
                        'verdict': 'PASS' if configured else 'REJECTED AS REGISTERED',
                        'lifecycle': 'inactive, never activated', 'goals_sent': 0, 'commands_published': 0,
                        'odometry_messages_published': 0, 'TF_published': 0})
                    (args.output/(name+'_result.json')).write_text(json.dumps(cases[-1], indent=2)+'\n')
                finally:
                    stop_group(proc)
                    if client is not None: node.destroy_client(client)
                    if check is not None: node.destroy_client(check.client)
                    until = time.monotonic()+1
                    while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.05)
        (args.output/'result.json').write_text(json.dumps({'verdict': 'PASS', 'cases': cases}, indent=2)+'\n')
        print('Actual configured evidence/readback and two initialization rejection cases PASS', flush=True)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
