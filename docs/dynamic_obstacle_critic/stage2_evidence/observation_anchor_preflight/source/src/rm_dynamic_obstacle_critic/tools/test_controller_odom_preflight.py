#!/usr/bin/env python3
"""Exercise the route gate against the installed, inactive Humble controller.

Run in an isolated ROS domain. The fixture advertises one canonical odometry
publisher, but never publishes a message, TF, command, or navigation goal.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from controller_odom_preflight import ControllerOdomPreflight


def stop_group(process):
    # ros2 run owns a child executable. Terminating only its wrapper leaves a
    # second controller with the same name in the next case's DDS graph.
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def main():
    import rclpy
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import Odometry

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    configs = Path(__file__).resolve().parents[1] / 'config'
    profiles = [('legacy', configs / 'nav2_cv_native_cycle_snapshot.yaml'),
                ('canonical', configs / 'nav2_cv_controller_odom.yaml')]
    rclpy.init()
    node = rclpy.create_node('controller_odom_fixture')
    publisher = node.create_publisher(Odometry, '/odometry/lio', 10)
    cases = []
    try:
        for name, profile in profiles:
            client = None
            check = None
            with (args.output / (name + '.log')).open('x') as log:
                process = subprocess.Popen([
                    'ros2', 'run', 'nav2_controller', 'controller_server',
                    '--ros-args', '--params-file', str(profile),
                    '-p', 'use_sim_time:=false',
                    '-p', 'FollowPath.NativeCycleSnapshotCritic.enabled:=false'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    client = node.create_client(ChangeState, '/controller_server/change_state')
                    if not client.wait_for_service(timeout_sec=8):
                        raise RuntimeError('configure service unavailable')
                    request = ChangeState.Request()
                    request.transition.id = 1
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                    if not future.done() or not future.result().success:
                        raise RuntimeError('actual configure failed')
                    check = ControllerOdomPreflight(node, '/odometry/lio')
                    result = error = None
                    while result is None and error is None:
                        rclpy.spin_once(node, timeout_sec=.02)
                        try:
                            result = check.poll()
                        except (RuntimeError, ValueError) as failure:
                            error = str(failure)
                    case = {
                        'profile': name, 'configured': True,
                        'verdict': 'PASS' if result else 'REJECTED',
                        'readback': result or check.evidence, 'error': error,
                        'lifecycle': 'inactive, never activated',
                        'goals_sent': 0, 'commands_published': 0,
                        'odometry_messages_published': 0}
                    cases.append(case)
                    (args.output / (name + '_result.json')).write_text(
                        json.dumps(case, indent=2) + '\n')
                    if (case['verdict'] == 'PASS') != (name == 'canonical'):
                        raise RuntimeError('actual route gate case failed: ' + name)
                finally:
                    stop_group(process)
                    if client is not None:
                        node.destroy_client(client)
                    if check is not None:
                        node.destroy_client(check.client)
                    until = time.monotonic() + 1
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.05)
        result = {
            'verdict': 'PASS', 'cases': cases,
            'scope': 'actual pinned controller lifecycle configure, parameter '
                     'and subscriber checks; effective speed requires physical capture'}
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result), flush=True)
    finally:
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
