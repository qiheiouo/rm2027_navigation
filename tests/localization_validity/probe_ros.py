"""Actual gate/bridge/consumers, synthetic boundary messages, no robot or serial."""
import copy
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rm_competition_interfaces.msg import LocalizationStatus, MissionState, SystemReadiness
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

OUT = Path(os.environ['PROBE_OUTPUT'])
OUT.mkdir(parents=True, exist_ok=True)
INSTALL = Path(os.environ['PROBE_INSTALL'])
ROOT = Path.cwd()
Q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL)
rclpy.init()
n = Node('aud001_observer')
processes, logs, result = [], [], {}
valid, status, upstream, transforms, tf, mission, readiness = [], [], [], [], [], [], []
n.create_subscription(Bool, '/localization/global_localization_valid',
                      lambda m: valid.append((time.monotonic(), m.data)), Q)
n.create_subscription(LocalizationStatus, '/localization/status',
                      lambda m: status.append((time.monotonic(), m)), 100)
n.create_subscription(TransformStamped, '/localization/map_to_odom',
                      lambda m: transforms.append((time.monotonic(), m)), 100)
n.create_subscription(TFMessage, '/tf', lambda m: tf.extend(
    (time.monotonic(), t) for t in m.transforms
    if t.header.frame_id == 'map' and t.child_frame_id == 'odom'), 100)
n.create_subscription(MissionState, '/mission/state',
                      lambda m: mission.append((time.monotonic(), m.detail)), Q)
n.create_subscription(SystemReadiness, '/system/readiness',
                      lambda m: readiness.append((time.monotonic(), m.ready_for_navigation)), Q)
op = n.create_publisher(Odometry, '/odometry/lio', 100)
raw_topic = '/localization/' + os.environ.get('PROBE_BACKEND', 'amcl') + '_pose_raw'
backend = os.environ.get('PROBE_BACKEND', 'amcl')
valid_topic = '/localization/' + backend + '_backend_valid'
rp = n.create_publisher(PoseWithCovarianceStamped, raw_topic, 10)
ep = n.create_publisher(LocalizationStatus, valid_topic + '/status', 10)
sp = n.create_publisher(LocalizationStatus, '/localization/status', 10)
n.create_subscription(LocalizationStatus, valid_topic + '/status',
                      lambda m: upstream.append((time.monotonic(), m)), 100)
last_raw = None

def launch(package, executable, args=()):
    log = (OUT / (executable + f'_{len(processes)}.log')).open('w')
    logs.append(log)
    p = subprocess.Popen([str(INSTALL / package / 'lib' / package / executable),
                          '--ros-args', *args], stdout=log, stderr=subprocess.STDOUT)
    processes.append(p)
    return p

def bridge():
    return launch('rm_relocalization_bridge', 'map_odom_from_global_pose',
                  ['-p', 'upstream_valid_topic:=' + valid_topic])

def gate():
    config = ('src/rm_relocalization_bridge/config/amcl_2d.yaml' if backend == 'amcl'
              else 'src/rm_gicp_relocalization/config/gicp_3d.yaml')
    return launch('rm_relocalization_bridge', 'global_pose_gate_node',
                  ['--params-file', str(ROOT / config), '-r', '__node:=' + backend + '_pose_gate'])

def consumers():
    m = launch('rm_competition_mission', 'competition_mission_node', [
        '-p', 'tree_xml:=' + str(ROOT / 'src/rm_competition_mission/trees/competition_default.xml'),
        '-p', 'startup_enabled:=true', '-p', 'require_referee_state:=false',
        '-p', 'require_game_running:=false', '-p', 'require_chassis_mode:=false'])
    r = launch('rm_system_monitor', 'readiness_monitor', [
        '-p', 'require_lio:=false', '-p', 'require_obstacle_input:=false',
        '-p', 'require_nav2:=false'])
    return m, r

def stop(p, sig=signal.SIGKILL):
    if p.poll() is None:
        p.send_signal(sig)
        p.wait(timeout=3)

def pose(stamp):
    m = PoseWithCovarianceStamped()
    m.header.stamp = stamp
    m.header.frame_id = 'map'
    m.pose.pose.orientation.w = 1.0
    m.pose.pose.position.x = 2.125
    return m

def spin(sec, raw=False, od=True, inject=None):
    global last_raw
    end = time.monotonic() + sec
    last = 0.0
    while time.monotonic() < end:
        if time.monotonic() - last >= 0.04:
            stamp = n.get_clock().now().to_msg()
            if od:
                o = Odometry()
                o.header.stamp = stamp
                o.header.frame_id = 'odom'
                o.child_frame_id = 'base_link'
                o.pose.pose.orientation.w = 1.0
                o.pose.pose.position.x = 0.125
                op.publish(o)
            if raw:
                last_raw = pose(stamp)
                rp.publish(last_raw)
            if inject:
                inject()
            last = time.monotonic()
        rclpy.spin_once(n, timeout_sec=0.005)

def snapshot():
    cutoff = time.monotonic() - 0.5
    return {'bridge_bool': valid[-1][1] if valid else None,
            'bridge_status': status[-1][1].valid if status else None,
            'tf_last_half_sec': sum(t > cutoff for t, _ in tf),
            'transform_last_half_sec': sum(t > cutoff for t, _ in transforms),
            'mission': mission[-1][1] if mission else None,
            'readiness': readiness[-1][1] if readiness else None}

def settle():
    # DDS discovery/reconnect time is not part of the fault-expiry budget.
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        spin(.1, True)
        s = snapshot()
        if (s['bridge_status'] and s['bridge_bool'] and s['tf_last_half_sec'] >= 8
                and s['mission'] == 'safety_ready' and s['readiness']):
            return
    raise AssertionError(('healthy stream did not settle', snapshot()))

def healthy(label):
    s = snapshot()
    result[label] = s
    assert s['bridge_bool'] and s['bridge_status'] and s['tf_last_half_sec'] >= 8, s
    assert s['mission'] == 'safety_ready' and s['readiness'], s
    assert abs(transforms[-1][1].transform.translation.x - 2.0) < 1e-12

def closed(label, since=None, bound=None):
    s = snapshot()
    if since is not None:
        transitions = [t - since for t, m in status if t >= since and not m.valid]
        s['first_false_after_event_sec'] = min(transitions) if transitions else None
        s['last_tf_after_event_sec'] = max((t-since for t, _ in tf if t >= since), default=0.0)
        if bound is not None:
            assert transitions and min(transitions) <= bound, s
    result[label] = s
    assert s['bridge_bool'] is False and s['bridge_status'] is False, s
    assert s['tf_last_half_sec'] == 0 and s['transform_last_half_sec'] == 0, s
    assert s['mission'] == 'safety_gate_closed' and s['readiness'] is False, s

try:
    b, g = bridge(), gate()
    m, r = consumers()
    spin(1.5, True)
    healthy('healthy')
    begin = time.monotonic()
    spin(12.0, True)
    healthy('stable_12_seconds')
    samples = [v for t, v in status if t >= begin]
    result['stable_12_seconds']['status_count'] = len(samples)
    result['stable_12_seconds']['false_samples'] = sum(not v.valid for v in samples)
    result['stable_12_seconds']['max_correction_error'] = max(
        abs(t.transform.translation.x - 2.0) for at, t in transforms if at >= begin)
    assert all(v.valid for v in samples)
    at = time.monotonic()
    spin(0.15)  # drain the last in-flight accepted pose before comparing held stamps
    source = copy.deepcopy(status[-1][1].pose.header.stamp)
    spin(0.45)
    healthy('no_new_correction_within_source_budget')
    assert status[-1][1].pose.header.stamp == source
    spin(1.5)
    closed('raw_input_stopped', at, 1.15 if backend == 'amcl' else 1.65)
    spin(0.9, True)
    healthy('raw_restored')
    # Kill actual gate, continue odometry, then replay a captured state.
    cached = copy.deepcopy(upstream[-1][1])
    at = time.monotonic()
    stop(g)
    spin(1.7)
    closed('gate_sigkill_odom_continues', at, 1.1)
    spin(0.7, inject=lambda: ep.publish(cached))
    closed('old_status_replayed_after_expiry')
    g = gate()
    spin(0.4)
    closed('gate_restart_without_pose')
    spin(0.7, True)
    healthy('gate_restart_fresh_pose')
    # Stop bridge while its upstream remains healthy: downstream leases expire.
    at = time.monotonic()
    stop(b)
    spin(1.35, True)
    result['bridge_sigkill'] = snapshot()
    result['bridge_sigkill']['mission_first_closed_sec'] = min(
        t-at for t, v in mission if t >= at and v == 'safety_gate_closed')
    result['bridge_sigkill']['readiness_first_closed_sec'] = min(
        t-at for t, v in readiness if t >= at and not v)
    assert mission[-1][1] == 'safety_gate_closed' and not readiness[-1][1]
    assert not any(t > time.monotonic() - .5 for t, _ in tf)
    b = bridge()
    settle()
    healthy('bridge_restart_fresh_pose')
    # A still-fresh pre-start message cannot authorize a new bridge.
    cached = copy.deepcopy(upstream[-1][1])
    g.send_signal(signal.SIGSTOP)
    stop(b)
    b = bridge()
    spin(0.7, inject=lambda: ep.publish(cached))
    result['fresh_old_status_new_bridge'] = snapshot()
    assert not valid[-1][1] and not status[-1][1].valid
    g.send_signal(signal.SIGCONT)
    spin(0.25)
    assert not valid[-1][1]  # new heartbeat, held pre-start pose: no restoration
    settle()
    healthy('new_bridge_requires_new_pose')
    # Gate rejects source replay, future/stale/zero timestamps and invalid quality.
    for label, alter in [
        ('repeated_source_stamp', lambda p: None),
        ('future_source_stamp', lambda p: setattr(p.header.stamp, 'sec', p.header.stamp.sec + 10)),
        ('stale_source_stamp', lambda p: setattr(p.header.stamp, 'sec', p.header.stamp.sec - 10)),
        ('negative_source_stamp', lambda p: setattr(p.header.stamp, 'sec', -1)),
        ('zero_source_stamp', lambda p: (setattr(p.header.stamp, 'sec', 0), setattr(p.header.stamp, 'nanosec', 0))),
        ('invalid_covariance', lambda p: p.pose.covariance.__setitem__(0, float('nan'))),
    ]:
        bad = copy.deepcopy(last_raw)
        alter(bad)
        at = time.monotonic()
        spin(.8, inject=lambda: rp.publish(bad))
        closed(label, at, .2)
        settle()
        healthy(label + '_fresh_recovery')
    # A new consumer receives an old (age < 1s) status after startup.
    cached = copy.deepcopy(status[-1][1])
    stop(b)
    stop(m)
    stop(r)
    m, r = consumers()
    spin(.7, True, inject=lambda: sp.publish(cached))
    result['fresh_old_status_new_consumers'] = snapshot()
    assert mission[-1][1] == 'safety_gate_closed' and not readiness[-1][1]
    b = bridge()
    settle()
    healthy('consumers_fresh_restoration')
    # Consumer rejects abnormal status timestamps and sequences independently.
    g.send_signal(signal.SIGSTOP)
    cached = copy.deepcopy(upstream[-1][1])
    cached.header.stamp.sec += 10
    spin(.8, inject=lambda: ep.publish(cached))
    closed('future_status_rejected')
    g.send_signal(signal.SIGCONT)
    spin(.8, True)
    healthy('future_status_fresh_recovery')
    result['scope'] = 'Actual ROS processes; boundary pose/odom fixture; no sensors/vehicle/serial/Nav2 acceptance.'
    result['backend'] = backend
    result['passed'] = True
finally:
    for p in reversed(processes):
        if p.poll() is None:
            p.send_signal(signal.SIGCONT)
            stop(p, signal.SIGTERM)
    for f in logs:
        f.close()
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    n.destroy_node()
    rclpy.shutdown()
print(json.dumps(result, indent=2))
