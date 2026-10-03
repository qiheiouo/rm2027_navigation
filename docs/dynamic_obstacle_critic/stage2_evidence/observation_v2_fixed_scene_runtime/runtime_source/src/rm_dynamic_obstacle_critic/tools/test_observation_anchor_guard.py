#!/usr/bin/env python3
"""Actual guard DDS schema and source-epoch tests on isolated test topics."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray as Array
from rm_competition_interfaces.msg import DynamicObstaclePrediction as Track


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('new_output', type=Path)
    parser.add_argument('--nominal-diameter', type=float, default=0.)
    parser.add_argument('--prior-source', default='')
    args = parser.parse_args(); args.new_output.mkdir(exist_ok=False)
    log = (args.new_output/'guard.log').open('w')
    namespace = '/observation_anchor_test'
    command = ['ros2', 'run', 'rm_dynamic_obstacle_critic', 'dynamic_safety_guard', '--ros-args',
        '-p', 'world_frame:=map', '-p', 'input_schema:='+Array.SCHEMA_OBSERVATION_ANCHOR]
    selected_mode = 'nominal_diameter' if args.nominal_diameter else 'visible_extent'
    if selected_mode == 'nominal_diameter':
        command += ['-p', 'obstacle_radius_mode:=nominal_diameter', '-p',
                    'nominal_obstacle_diameter:='+str(args.nominal_diameter), '-p',
                    'geometry_prior_source:='+args.prior_source]
    for name, suffix in [('input_topic', 'input'), ('output_topic', 'output'),
                         ('obstacles_topic', 'obstacles'), ('odom_topic', 'odom'),
                         ('costmap_topic', 'map')]:
        command += ['-p', name+':='+namespace+'/'+suffix]
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(); node = rclpy.create_node('observation_anchor_guard_fixture')
    outputs, diagnostics, subscriptions = [], [], []
    subscriptions += [node.create_subscription(Twist, namespace+'/output',
        lambda m: outputs.append([m.linear.x, m.linear.y, m.angular.z]), 10)]
    subscriptions += [node.create_subscription(DiagnosticArray, '/dynamic_guard/diagnostics',
        lambda m: diagnostics.append(m.status[0].message), 10)]
    command_pub = node.create_publisher(Twist, namespace+'/input', 1)
    obstacle_pub = node.create_publisher(Array, namespace+'/obstacles', 1)
    odom_pub = node.create_publisher(Odometry, namespace+'/odom', 1)
    map_pub = node.create_publisher(Costmap, namespace+'/map',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    rows = []

    def exercise(name, schema, state, observation_delta_ns, expected, expected_reason, position_x=3.):
        outputs.clear(); diagnostics.clear(); deadline = time.monotonic()+.4
        while time.monotonic() < deadline:
            now = node.get_clock().now()
            proposal = Twist(); proposal.linear.x = .2
            odom = Odometry(); odom.header.stamp = now.to_msg(); odom.header.frame_id = 'map'
            odom.child_frame_id = 'base_link'; odom.pose.pose.orientation.w = 1.
            array = Array(); array.header.stamp = now.to_msg(); array.header.frame_id = 'map'
            array.schema = schema; array.authority = Array.AUTHORITY_SHADOW_ONLY
            array.complete = True; array.prediction_dt = .1; array.prediction_steps = 30
            track = Track(); track.track_id = 1; track.state = state
            track.position.x = position_x; track.size.x = track.size.y = .2
            track.last_observation_stamp = Time(nanoseconds=now.nanoseconds+observation_delta_ns).to_msg()
            array.tracks = [track]; array.total_track_count = 1
            grid = Costmap(); grid.header.stamp = now.to_msg(); grid.header.frame_id = 'map'
            grid.metadata.size_x = grid.metadata.size_y = 80; grid.metadata.resolution = .1
            grid.metadata.origin.position.x = grid.metadata.origin.position.y = -4.
            grid.metadata.origin.orientation.w = 1.; grid.data = [0]*6400
            command_pub.publish(proposal); odom_pub.publish(odom)
            map_pub.publish(grid); obstacle_pub.publish(array)
            rclpy.spin_once(node, timeout_sec=.01)
        tail = outputs[-5:]; reason_tail = diagnostics[-5:]
        passed = len(tail) == 5 and all(v == [expected, 0., 0.] for v in tail)
        passed &= bool(reason_tail) and all(expected_reason in reason for reason in reason_tail)
        rows.append(dict(case=name, passed=passed, output_tail=tail, reason_tail=reason_tail))

    try:
        client = node.create_client(GetParameters, '/dynamic_safety_guard/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('guard parameter service missing')
        request = GetParameters.Request(names=['input_schema', 'obstacle_radius_mode',
                                                'nominal_obstacle_diameter', 'geometry_prior_source'])
        future = client.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        if not future.done() or future.result() is None:
            raise RuntimeError('schema readback failed')
        values = future.result().values
        actual = dict(input_schema=values[0].string_value, obstacle_radius_mode=values[1].string_value,
                      nominal_obstacle_diameter=values[2].double_value, geometry_prior_source=values[3].string_value)
        expected_parameters = dict(input_schema=Array.SCHEMA_OBSERVATION_ANCHOR, obstacle_radius_mode=selected_mode,
                                   nominal_obstacle_diameter=args.nominal_diameter, geometry_prior_source=args.prior_source)
        if [v.type for v in values] != [4,4,3,4] or actual != expected_parameters:
            raise RuntimeError('selected guard schema was not loaded')
        deadline = time.monotonic()+3.
        while obstacle_pub.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        if obstacle_pub.get_subscription_count() != 1:
            raise RuntimeError('guard subscription missing')
        exercise('fresh_observation_passes', Array.SCHEMA_OBSERVATION_ANCHOR,
                 Track.STATE_CONFIRMED, 0, .2, 'clear')
        exercise('v1_replacement_brakes', Array.SCHEMA, Track.STATE_CONFIRMED, 0, 0., 'schema_or_frame')
        exercise('v2_recovers_after_transition', Array.SCHEMA_OBSERVATION_ANCHOR,
                 Track.STATE_CONFIRMED, 0, .2, 'clear')
        exercise('old_confirmed_epoch_brakes', Array.SCHEMA_OBSERVATION_ANCHOR,
                 Track.STATE_CONFIRMED, -100000000, 0., 'observation_anchor_epoch')
        exercise('fresh_coasting_passes', Array.SCHEMA_OBSERVATION_ANCHOR,
                 Track.STATE_COASTING, -100000000, .2, 'clear')
        exercise('coasting_at_source_brakes', Array.SCHEMA_OBSERVATION_ANCHOR,
                 Track.STATE_COASTING, 0, 0., 'observation_anchor_epoch')
        exercise('expired_observation_brakes', Array.SCHEMA_OBSERVATION_ANCHOR,
                 Track.STATE_COASTING, -1000000000, 0., 'stale_observation')
        if selected_mode == 'nominal_diameter':
            exercise('nominal_full_diameter_near_anchor_brakes', Array.SCHEMA_OBSERVATION_ANCHOR,
                     Track.STATE_CONFIRMED, 0, 0., 'dynamic_collision', position_x=.9)
        else:
            exercise('visible_near_anchor_passes', Array.SCHEMA_OBSERVATION_ANCHOR,
                     Track.STATE_CONFIRMED, 0, .2, 'clear', position_x=.9)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
        node.destroy_node(); rclpy.shutdown(); log.close()
    report = dict(schema=1, status='PASS' if all(row['passed'] for row in rows) else 'FAILED',
                  parameter_readback=actual, cases=rows,
                  scope='Public-message schema and pass/brake regression on isolated DDS topics; no physical safety certificate.')
    (args.new_output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, sort_keys=True))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
