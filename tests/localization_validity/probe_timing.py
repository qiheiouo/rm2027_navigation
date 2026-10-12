"""Simulated ROS clock anomalies, actual nodes; no simulated motion authorization."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rm_competition_interfaces.msg import LocalizationStatus, MissionState, SystemReadiness
from rosgraph_msgs.msg import Clock

OUT = Path(os.environ['PROBE_OUTPUT'])
OUT.mkdir(parents=True, exist_ok=True)
INSTALL = Path(os.environ['PROBE_INSTALL'])
rclpy.init()
n = Node('aud001_time_observer')
clock = n.create_publisher(Clock, '/clock', 10)
raw = n.create_publisher(PoseWithCovarianceStamped, '/localization/amcl_pose_raw', 10)
odom = n.create_publisher(Odometry, '/odometry/lio', 100)
states, corrections, mission, readiness, procs, logs = [], [], [], [], [], []
n.create_subscription(LocalizationStatus, '/localization/status', lambda m: states.append((time.monotonic(), m)), 100)
n.create_subscription(TransformStamped, '/localization/map_to_odom', lambda m: corrections.append((time.monotonic(), m)), 100)
n.create_subscription(MissionState, '/mission/state', lambda m: mission.append(m.detail), 1)
n.create_subscription(SystemReadiness, '/system/readiness', lambda m: readiness.append(m.ready_for_navigation), 1)
ros_ns = 0

def start(pkg, exe, args):
    f = (OUT / (exe + '.log')).open('w')
    logs.append(f)
    p = subprocess.Popen([str(INSTALL / pkg / 'lib' / pkg / exe), '--ros-args',
                          '-p', 'use_sim_time:=true', *args], stdout=f, stderr=subprocess.STDOUT)
    procs.append(p)
    return p

def spin(sec, advance=True, input=True):
    global ros_ns
    end = time.monotonic() + sec
    last = 0.0
    while time.monotonic() < end:
        if time.monotonic() - last > .04:
            if advance:
                ros_ns += 40_000_000
            c = Clock()
            c.clock.sec, c.clock.nanosec = divmod(ros_ns, 1_000_000_000)
            clock.publish(c)
            if input and ros_ns > 0:
                o = Odometry()
                o.header.stamp = c.clock
                o.header.frame_id = 'odom'
                o.child_frame_id = 'base_link'
                o.pose.pose.orientation.w = 1.0
                odom.publish(o)
                p = PoseWithCovarianceStamped()
                p.header.stamp = c.clock
                p.header.frame_id = 'map'
                p.pose.pose.orientation.w = 1.0
                raw.publish(p)
            last = time.monotonic()
        rclpy.spin_once(n, timeout_sec=.005)

def snap():
    return {'status': states[-1][1].valid if states else None,
            'tf_copies_last_half_sec': sum(t > time.monotonic()-.5 for t, _ in corrections),
            'mission': mission[-1] if mission else None,
            'readiness': readiness[-1] if readiness else None}

result = {}
try:
    b = start('rm_relocalization_bridge', 'map_odom_from_global_pose', ['-p', 'upstream_valid_topic:=/localization/amcl_backend_valid'])
    g = start('rm_relocalization_bridge', 'global_pose_gate_node', [])
    start('rm_competition_mission', 'competition_mission_node', [
        '-p', 'tree_xml:=' + str(Path.cwd() / 'src/rm_competition_mission/trees/competition_default.xml'),
        '-p', 'startup_enabled:=true', '-p', 'require_referee_state:=false',
        '-p', 'require_game_running:=false', '-p', 'require_chassis_mode:=false'])
    start('rm_system_monitor', 'readiness_monitor', ['-p', 'require_lio:=false',
          '-p', 'require_obstacle_input:=false', '-p', 'require_nav2:=false'])
    spin(.8, False, False)
    result['zero_clock_initialization'] = snap()
    assert states and not states[-1][1].valid and not corrections
    ros_ns = 10_000_000_000
    spin(2.0)
    result['positive_clock_healthy'] = snap()
    assert states[-1][1].valid and mission[-1] == 'safety_ready' and readiness[-1]
    # No advancing ROS time or source input; leases still close on wall/steady time.
    spin(1.7, False, False)
    result['paused_clock_input_loss'] = snap()
    assert not states[-1][1].valid and mission[-1] == 'safety_gate_closed' and not readiness[-1]
    assert result['paused_clock_input_loss']['tf_copies_last_half_sec'] == 0
    ros_ns -= 5_000_000_000
    spin(.8)
    result['reversed_clock'] = snap()
    assert not states[-1][1].valid and not readiness[-1]
    ros_ns += 10_000_000_000
    spin(2.0)
    result['clock_catches_up_fresh_measurement'] = snap()
    assert states[-1][1].valid and mission[-1] == 'safety_ready' and readiness[-1]
    g.kill()
    g.wait(timeout=3)
    spin(1.5, False, False)
    result['paused_clock_gate_exit'] = snap()
    assert not states[-1][1].valid and mission[-1] == 'safety_gate_closed' and not readiness[-1]
    result['passed'] = True
finally:
    for p in reversed(procs):
        if p.poll() is None:
            p.terminate()
            p.wait(timeout=3)
    for f in logs:
        f.close()
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    n.destroy_node()
    rclpy.shutdown()
print(json.dumps(result, indent=2))
