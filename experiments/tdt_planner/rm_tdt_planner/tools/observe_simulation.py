#!/usr/bin/env python3
"""Observe one goal in the isolated Phase 1.5 static simulation; preserve raw evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import traceback

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rcl_interfaces.msg import Log
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from simulation_geometry import clearance, path_distance
from simulation_evidence import analyze


def seconds(msg):
    return msg.sec + msg.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Observer(Node):
    def __init__(self, output, launch_log):
        super().__init__('tdt_simulation_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.streams = {name: (output / f'{name}.jsonl').open('x', buffering=1)
                        for name in ('trajectory', 'plans', 'commands', 'events', 'costmap')}
        self.launch_log = launch_log
        self.fixture_ready = False
        self.now_sim = 0.0
        self.scan_count = self.odom_count = self.map_count = 0
        self.lethal_cells = 0
        self.path, self.path_id = [], 0
        self.current = None
        self.last_sample = self.last_scan = self.last_odom = None
        self.scan_gap = self.odom_gap = 0.0
        self.clock_reversed = False
        self.active = False
        self.samples, self.commands, self.events = [], [], []
        self.recoveries = 0
        self.refs = [
            self.create_subscription(Clock, '/clock', self.clock_cb, qos_profile_sensor_data),
            self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/simulation/ground_truth/odom', self.odom_cb, qos_profile_sensor_data),
            self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_cb,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(NavPath, '/plan', self.plan_cb, 10),
            self.create_subscription(Twist, '/cmd_vel', self.command_cb, 20),
            self.create_subscription(Log, '/rosout', self.log_cb, 100),
        ]
        self.navigation = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planning = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def write(self, stream, data):
        self.streams[stream].write(json.dumps(data, allow_nan=False) + '\n')

    def clock_cb(self, msg):
        t = seconds(msg.clock)
        self.clock_reversed |= t < self.now_sim
        self.now_sim = t

    def scan_cb(self, msg):
        t = seconds(msg.header.stamp)
        if self.last_scan is not None:
            self.scan_gap = max(self.scan_gap, t - self.last_scan)
        self.last_scan = t
        self.scan_count += 1

    def map_cb(self, msg):
        self.map_count += 1
        self.lethal_cells = sum(c >= 99 for c in msg.data)
        self.write('costmap', {'t': seconds(msg.header.stamp), 'frame': msg.header.frame_id,
                              'width': msg.info.width, 'height': msg.info.height,
                              'resolution': msg.info.resolution,
                              'origin': [msg.info.origin.position.x, msg.info.origin.position.y],
                              'data': list(msg.data)})

    def odom_cb(self, msg):
        t = seconds(msg.header.stamp)
        self.odom_count += 1
        if self.last_odom is not None:
            self.odom_gap = max(self.odom_gap, t - self.last_odom)
        self.last_odom = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.current = (p.x, p.y, yaw(q))
        if not self.active or (self.last_sample is not None and t - self.last_sample < .039):
            return
        self.last_sample = t
        x, y, heading = self.current
        row = {'t': t, 'x': x, 'y': y, 'yaw': heading, 'plan_id': self.path_id,
               'body_clearance_m': clearance(x, y, heading),
               'padded_clearance_m': clearance(x, y, heading, .03),
               'cross_track_m': path_distance(x, y, self.path),
               'vx': msg.twist.twist.linear.x, 'vy': msg.twist.twist.linear.y,
               'wz': msg.twist.twist.angular.z}
        self.samples.append(row)
        self.write('trajectory', row)

    def plan_cb(self, msg):
        if msg.header.frame_id != 'map':
            return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_id += 1
        self.write('plans', {'id': self.path_id, 't': seconds(msg.header.stamp), 'xy': self.path})

    def command_cb(self, msg):
        row = {'t': self.now_sim, 'vx': msg.linear.x, 'vy': msg.linear.y, 'wz': msg.angular.z}
        self.commands.append(row)
        self.write('commands', row)

    def log_cb(self, msg):
        # Humble Log.level is uint8/int, but Log.WARN is a byte constant.
        if msg.level >= LoggingSeverity.WARN:
            row = {'t': self.now_sim, 'node': msg.name, 'level': msg.level, 'message': msg.msg}
            self.events.append(row)
            self.write('events', row)

    def feedback_cb(self, msg):
        self.recoveries = max(self.recoveries, msg.feedback.number_of_recoveries)

    def until(self, predicate, wall_seconds):
        end = time.monotonic() + wall_seconds
        while rclpy.ok() and not predicate():
            if time.monotonic() >= end:
                return False
            rclpy.spin_once(self, timeout_sec=.05)
        return bool(predicate())

    def ready(self):
        if not (self.now_sim >= 6 and self.scan_count >= 30 and self.odom_count >= 100 and
                self.map_count >= 2 and self.lethal_cells >= 3 and self.navigation.server_is_ready() and
                self.planning.server_is_ready()):
            return False
        if not self.fixture_ready:
            text = self.launch_log.read_text(errors='replace')
            self.fixture_ready = all(f'[TDT_P2B] spawn_course_wall_{side} exit=0' in text
                                     for side in ('north', 'south'))
        return self.fixture_ready

    def pose(self, x, y, heading=0.0):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        return p

    def run(self, goal_x, goal_y):
        if not self.until(self.ready, 120):
            raise RuntimeError(f'simulation readiness timeout: clock={self.now_sim}, scans={self.scan_count}, '
                               f'odom={self.odom_count}, maps={self.map_count}, fixture={self.fixture_ready}')
        preflight = ComputePathToPose.Goal()
        preflight.start, preflight.goal = self.pose(*self.current), self.pose(goal_x, goal_y)
        preflight.use_start, preflight.planner_id = True, 'GridBased'
        started = time.monotonic()
        future = self.planning.send_goal_async(preflight)
        if not self.until(future.done, 15) or not future.result().accepted:
            raise RuntimeError('compute-path action not accepted')
        handle = future.result()
        planned = handle.get_result_async()
        if not self.until(planned.done, 30):
            handle.cancel_goal_async()
            raise RuntimeError('compute-path action timeout')
        before = {'status': planned.result().status, 'wall_ms': (time.monotonic()-started)*1000,
                  'samples': len(planned.result().result.path.poses)}
        self.write('events', {'t': self.now_sim, 'event': 'preflight', 'result': before})
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(goal_x, goal_y)
        self.active = True
        sim_start, wall_start = self.now_sim, time.monotonic()
        future = self.navigation.send_goal_async(goal, feedback_callback=self.feedback_cb)
        if not self.until(future.done, 15) or not future.result().accepted:
            raise RuntimeError('navigation action not accepted')
        handle = future.result()
        result = handle.get_result_async()
        self.until(lambda: result.done() or self.now_sim-sim_start >= 90, 300)
        timed_out = not result.done()
        if timed_out:
            cancel = handle.cancel_goal_async()
            self.until(cancel.done, 10)
            self.until(result.done, 10)
        status = result.result().status if result.done() else None
        settle = self.now_sim + .5
        self.until(lambda: self.now_sim >= settle, 10)
        self.active = False
        self.write('events', {'t': self.now_sim, 'event': 'navigation_result', 'status': status,
                              'timed_out': timed_out, 'recoveries': self.recoveries,
                              'goal': [goal_x, goal_y, 0.0]})
        report = analyze(self.samples, self.commands, self.events, goal_x, goal_y,
                         status, timed_out, self.recoveries)
        report.update({'preflight': before, 'sim_seconds': self.now_sim-sim_start,
                       'wall_seconds': time.monotonic()-wall_start, 'plan_messages': self.path_id,
                       'max_scan_gap_sim_s': self.scan_gap, 'max_odom_gap_sim_s': self.odom_gap,
                       'clock_reversed': self.clock_reversed, 'fixture_spawn_verified': self.fixture_ready})
        report['evidence_valid'] = bool(report['evidence_valid'] and not self.clock_reversed and self.fixture_ready)
        report['static_geometry_and_goal_pass'] &= report['evidence_valid']
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--launch-log', required=True, type=Path)
    parser.add_argument('--goal-x', type=float, default=4.3)
    parser.add_argument('--goal-y', type=float, default=0.0)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '174' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('use the documented isolated localhost simulation domain 174')
    if not all(math.isfinite(v) for v in (args.goal_x, args.goal_y)):
        parser.error('non-finite goal')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Observer(args.output, args.launch_log)
    report = {}
    try:
        report = node.run(args.goal_x, args.goal_y)
    except Exception as exc:
        traceback.print_exc()  # stderr is retained in observer.log by the trial runner.
        report = {'error': str(exc), 'error_type': type(exc).__name__,
                  'traceback': traceback.format_exc(), 'evidence_valid': False,
                  'static_geometry_and_goal_pass': False, 'simulation_clock': node.now_sim}
    finally:
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        for stream in node.streams.values():
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, allow_nan=False), flush=True)
    if not report.get('evidence_valid'):
        return 2
    return 0 if report.get('static_geometry_and_goal_pass') else 1


if __name__ == '__main__':
    raise SystemExit(main())
