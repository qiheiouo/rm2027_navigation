"""Real AMCL/PCL GICP plus gate/bridge, existing synthetic ROS fixtures only."""
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
from rm_competition_interfaces.msg import LocalizationStatus
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool
from types import SimpleNamespace
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

ROOT = Path.cwd()
OUT = Path(os.environ['PROBE_OUTPUT'])
OUT.mkdir(parents=True, exist_ok=True)
INSTALL = Path(os.environ['PROBE_INSTALL'])
BACKEND = os.environ['PROBE_BACKEND']
processes, logs, statuses, raw_poses, corrections = [], [], [], [], []
rclpy.init()
n = Node('aud001_native_backend_observer')
tf = TransformBroadcaster(n)
op = n.create_publisher(Odometry, '/odometry/lio', 100)
cloud = n.create_publisher(PointCloud2, '/audit/gicp_cloud', 10)
seed = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
if os.environ.get('PROBE_BASELINE'):
    n.create_subscription(Bool, '/localization/global_localization_valid',
        lambda m: statuses.append((time.monotonic(), SimpleNamespace(valid=m.data))),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
else:
    n.create_subscription(LocalizationStatus, '/localization/status',
                          lambda m: statuses.append((time.monotonic(), m)), 100)
n.create_subscription(PoseWithCovarianceStamped, '/localization/' + BACKEND + '_pose_raw',
                      lambda m: raw_poses.append((time.monotonic(), m)), 100)
n.create_subscription(TransformStamped, '/localization/map_to_odom',
                      lambda m: corrections.append((time.monotonic(), m)), 100)
nomotion = n.create_client(Empty, "/request_nomotion_update")
nomotion_requests = 0
points = []
for i in range(13):
    for j in range(7):
        points += [(-2. + i*.35, -1.7, j*.35), (-2., -1.7 + i*.35, j*.35)]
for i in range(10):
    for j in range(9):
        points.append((-.4 + i*.35, -.7 + j*.35, .91))
pcd = OUT / 'synthetic.pcd'
pcd.write_text('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
               'COUNT 1 1 1\nWIDTH '+str(len(points))+'\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
               'POINTS '+str(len(points))+'\nDATA ascii\n' + ''.join(
                   ' '.join(map(str, p)) + '\n' for p in points))

def start(command, label):
    f = (OUT / (label + '.log')).open('w')
    logs.append(f)
    p = subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT)
    processes.append(p)
    return p

def executable(package, name, args=()):
    return [str(INSTALL / package / 'lib' / package / name), '--ros-args', *args]

def spin(sec, predicate=None, clouds=True):
    global nomotion_requests
    end = time.monotonic() + sec
    last = 0.0
    last_request = 0.0
    while time.monotonic() < end:
        if time.monotonic() - last > .04:
            stamp = n.get_clock().now().to_msg()
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = 'odom'
            o.child_frame_id = 'base_link'
            o.pose.pose.orientation.w = 1.0
            op.publish(o)
            t = TransformStamped()
            t.header = o.header
            t.child_frame_id = 'base_link'
            t.transform.rotation.w = 1.0
            tf.sendTransform(t)
            if BACKEND == 'gicp' and clouds:
                cloud.publish(point_cloud2.create_cloud_xyz32(
                    Header(stamp=stamp, frame_id='odom'),
                    [(x-.4, y, z) for x, y, z in points]))
            last = time.monotonic()
        if BACKEND == 'amcl' and not os.environ.get('PROBE_NO_NOMOTION') and clouds and time.monotonic() - last_request >= .2 and nomotion.service_is_ready():
            nomotion.call_async(Empty.Request())
            nomotion_requests += 1
            last_request = time.monotonic()
        rclpy.spin_once(n, timeout_sec=.005)
        if predicate and predicate():
            return True
    return predicate is None or predicate()

result = {'backend': BACKEND, 'scope': 'Synthetic map/scans/PCD; actual backend/gate/bridge; no vehicle.'}
try:
    b = start(executable('rm_relocalization_bridge', 'map_odom_from_global_pose',
                         ['-p', 'upstream_valid_topic:=/localization/' + BACKEND + '_backend_valid']), 'bridge')
    if BACKEND == 'amcl':
        producer = start(executable('rm_relocalization_bridge', 'fake_2d_localization_world'), 'world')
        backend = start(['ros2', 'launch', 'rm_relocalization_bridge', 'amcl_2d_backend.launch.py',
                         'enable_backend:=true', 'set_initial_pose:=true'], 'backend')
    else:
        backend = start(executable('rm_gicp_relocalization', 'gicp_relocalization_node', [
            '--params-file', str(ROOT / 'src/rm_gicp_relocalization/config/gicp_3d.yaml'),
            '-p', 'prior_pcd_file:=' + str(pcd), '-p', 'map_id:=audit_asymmetric',
            '-p', 'input_cloud_topic:=/audit/gicp_cloud']), 'backend')
        gate = start(executable('rm_relocalization_bridge', 'global_pose_gate_node', [
            '--params-file', str(ROOT / 'src/rm_gicp_relocalization/config/gicp_3d.yaml'),
            '-r', '__node:=gicp_pose_gate']), 'gate')
        spin(1.0)
        s = PoseWithCovarianceStamped()
        s.header.stamp = n.get_clock().now().to_msg()
        s.header.frame_id = 'map'
        s.pose.pose.orientation.w = 1.0
        s.pose.pose.position.x = .4
        seed.publish(s)
    assert spin(10.0, lambda: statuses and statuses[-1][1].valid), 'no accepted native pose'
    at = time.monotonic()
    spin(6.0)
    observations = [m for t, m in statuses if t > at + .5]
    result['stable'] = {'status_count': len(observations),
                        'false_samples': sum(not m.valid for m in observations),
                        'raw_pose_count': sum(t > at for t, _ in raw_poses),
                        'correction_count': sum(t > at for t, _ in corrections),
                        'last_position_x': raw_poses[-1][1].pose.pose.position.x}
    end = time.monotonic()
    start = at + .5
    previous = next((m.valid for t, m in reversed(statuses) if t <= start), True)
    cursor = start
    closed_duration = 0.0
    for t, m in statuses:
        if t < start:
            continue
        if not previous:
            closed_duration += t-cursor
        cursor, previous = t, m.valid
    if not previous:
        closed_duration += end-cursor
    result['stable']['closed_duration_sec'] = closed_duration
    result['stable']['raw_stamps'] = [m.header.stamp.sec + m.header.stamp.nanosec*1e-9 for t, m in raw_poses if t > at]
    if not os.environ.get('PROBE_OBSERVE_ONLY'):
        assert observations and all(m.valid for m in observations), result
    assert result['stable']['raw_pose_count'] >= 5
    if BACKEND == 'gicp':
        assert abs(result['stable']['last_position_x'] - .4) < .05
    # Stop the actual input source/backend while the common gate remains alive.
    at = time.monotonic()
    if BACKEND == 'amcl':
        producer.kill()
        producer.wait(timeout=3)
    else:
        backend.kill()
        backend.wait(timeout=3)
    spin(2.2, clouds=False)
    false_times = [t-at for t, m in statuses if t >= at and not m.valid]
    result['input_loss'] = {'first_false_sec': min(false_times) if false_times else None,
                            'status_valid': statuses[-1][1].valid,
                            'tf_copies_last_half_sec': sum(t > time.monotonic()-.5 for t, _ in corrections)}
    assert false_times and min(false_times) < (1.15 if BACKEND == 'amcl' else 1.65), result
    assert not statuses[-1][1].valid and result['input_loss']['tf_copies_last_half_sec'] == 0
    result['nomotion_requests_for_stationary_fixture'] = nomotion_requests
    result['passed'] = not bool(os.environ.get('PROBE_OBSERVE_ONLY'))
    result['observe_only'] = bool(os.environ.get('PROBE_OBSERVE_ONLY'))
finally:
    for p in reversed(processes):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
    for f in logs:
        f.close()
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    n.destroy_node()
    rclpy.shutdown()
print(json.dumps(result, indent=2))
