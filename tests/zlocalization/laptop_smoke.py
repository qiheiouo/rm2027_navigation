#!/usr/bin/env python3
"""Short ROS integration smoke; deterministic fixture, never sensor evidence."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rm_competition_interfaces.msg import LocalizationStatus
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/rm_zlocalization_candidate/scripts'))
from geometry import matrix, quaternion


def rotate(axis, angle, position=(0., 0., 0.)):
    return matrix(position, [*(np.asarray(axis) * np.sin(angle/2)), np.cos(angle/2)])


def fill_pose(pose, transform):
    pose.position.x, pose.position.y, pose.position.z = map(float, transform[:3, 3])
    q = quaternion(transform[:3, :3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


def tf_matrix(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prior', action='store_true')
    parser.add_argument('--event', choices=('invalidate', 'restart'), default='invalidate')
    parser.add_argument('--save-map-only', action='store_true',
        help='Reuse the fixture only to exercise the upstream runtime bundle save service')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.save_map_only and args.prior:
        parser.error('--save-map-only requires mapping mode, without --prior')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('zlocalization_laptop_observer')
    raw_pub = node.create_publisher(Odometry, '/odometry/fast_lio_raw', 20)
    cloud_pub = node.create_publisher(PointCloud2, '/lio/cloud_registered', 20)
    broadcaster = TransformBroadcaster(node)
    received = {'status': [], 'tf': [], 'state': [], 'odom': [], 'private_odom': [], 'private_cloud': []}
    subs = [
        node.create_subscription(LocalizationStatus, '/localization/status', received['status'].append, 100),
        node.create_subscription(TFMessage, '/tf', received['tf'].append, 100),
        node.create_subscription(String, '/zlocalization/state', received['state'].append, 20),
        node.create_subscription(Odometry, '/odometry/lio', received['odom'].append, 100),
        node.create_subscription(Odometry, '/zlocalization/fastlio_odom', received['private_odom'].append, 100),
        node.create_subscription(PointCloud2, '/zlocalization/cloud_registered', received['private_cloud'].append, 20),
    ]
    k0 = rotate((0, 1, 0), .35, (.3, -.1, .4))
    # Arbitrary stable frontend origin, including a non-identity first state.
    raw_from_world = rotate((0, 0, 1), -.25, (.6, .2, -.1)) @ np.linalg.inv(k0)
    points = []
    for x in np.arange(-4., 6.01, .18):
        for y in np.arange(-3., 4.01, .18):
            for z in (-1., 2.5):
                points.append((x, y, z))
    for z in np.arange(-1., 2.51, .18):
        for x in np.arange(-4., 6.01, .18):
            for y in (-3., 4.):
                points.append((x, y, z))
        for y in np.arange(-3., 4.01, .18):
            for x in (-4., 6.):
                points.append((x, y, z))
    points = (np.asarray(points) @ raw_from_world[:3, :3].T + raw_from_world[:3, 3]).astype(np.float32)
    launch_args = ['mode:=mapping']
    if args.save_map_only:
        config_path = out / 'mapping_config.yaml'
        config_path.write_text(yaml.safe_dump({
            'extends': str(ROOT/'src/rm_zlocalization_candidate/config/z_localization_candidate.yaml'),
            'trajectory': {'save_directory': str(out/'raw'), 'save_global_pcd': True,
                           'save_on_shutdown': False}}))
        launch_args.append('config_file:=' + str(config_path))
    if args.prior:
        pcd = out / 'fixture.pcd'
        with pcd.open('w') as f:
            f.write(f'# synthetic interface fixture\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
            np.savetxt(f, points, fmt='%.7f')
        subprocess.run(['ros2', 'run', 'backend', 'build_pcd_map_bundle', '--input', str(pcd),
            '--out-bundle', str(out/'prior'), '--x-min', '0', '--x-max', '0', '--y-min', '0',
            '--y-max', '0', '--sensor-z', '0', '--map-voxel', '.15', '--scan-voxel', '.15',
            '--max-keyframes', '1', '--sc-augment-count', '1', '--occlusion', 'false',
            '--fov-down', '-90', '--fov-up', '90'], check=True, stdout=(out/'map_tool.log').open('w'), stderr=subprocess.STDOUT)
        config = yaml.safe_load((ROOT/'src/rm_zlocalization_candidate/config/z_localization_candidate.yaml').read_text())
        # Match this fixture bundle's saved descriptor metadata exactly.
        config['common']['scan_context'] = {'augment_count': 1, 'lidar_height': 0.0}
        config['startup_hint'] = {'enabled': True, 'force_until_matched': True,
            'x': .6, 'y': .2, 'z': -.1, 'yaw_deg': float(np.rad2deg(-.25))}
        config_path = out/'fixture_config.yaml'
        config_path.write_text(yaml.safe_dump(config))
        launch_args = ['prior_map:='+str(out/'prior'), 'config_file:='+str(config_path)]
    env = dict(os.environ)
    log = (out/'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'rm_zlocalization_candidate',
        'z_localization_candidate.launch.py', *launch_args], env=env, stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    result = {}

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('candidate exited; see launch.log')

    def drive(index):
        stamp = node.get_clock().now().to_msg()
        b = rotate((0, 0, 1), .04*np.sin(index*.04), (.02*np.sin(index*.04), 0., 0.))
        k = rotate((0, 0, 1), .3*np.sin(index*.04)) @ k0
        raw = raw_from_world @ b @ k
        tf = TransformStamped()
        tf.header = Header(stamp=stamp, frame_id='base_link')
        tf.child_frame_id = 'lio_imu_link'
        t, q = tf.transform.translation, tf.transform.rotation
        t.x, t.y, t.z = map(float, k[:3, 3])
        q.x, q.y, q.z, q.w = map(float, quaternion(k[:3, :3]))
        broadcaster.sendTransform(tf)
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='odom')
        odom.child_frame_id = 'body'
        fill_pose(odom.pose.pose, raw)
        raw_pub.publish(odom)
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='odom'), points))
        spin(.10)

    try:
        spin(2.)
        assert received['status'] and not any(m.valid for m in received['status']), 'startup trust must be false'
        result['startup_untrusted'] = True
        index = 0
        for index in range(100):
            drive(index)
            if any(m.valid for m in received['status'][-10:]) and index >= 20:
                break
        assert any(m.valid for m in received['status'][-10:]), 'no trusted output'
        corrections = [t for msg in received['tf'] for t in msg.transforms if t.header.frame_id == 'map' and t.child_frame_id == 'odom']
        assert corrections, 'no canonical correction'
        error = float(np.max(np.abs(tf_matrix(corrections[-1]) - raw_from_world)))
        assert error < .03, f'dynamic geometry correction error {error}'
        result['max_correction_matrix_error'] = error
        assert received['private_odom'] and received['private_cloud'] and received['odom']
        assert received['private_odom'][-1].header.frame_id == 'fast_lio_odom'
        assert received['private_odom'][-1].child_frame_id == 'lio_imu_link'
        assert received['private_cloud'][-1].header.frame_id == 'fast_lio_odom'
        assert received['private_odom'][-1].header.stamp == received['private_cloud'][-1].header.stamp
        assert not any(received['private_odom'][-1].pose.covariance)
        result['input_frames_times_fallback'] = True
        # Identify the only public map/odom-capable publisher in this composition.
        owners = [p.node_name for p in node.get_publishers_info_by_topic('/tf')]
        assert owners.count('map_odom_from_global_pose') == 1 and 'zlocalization_backend' not in owners
        assert owners.count('lio_adapter') == 1
        result['public_tf_publishers'] = owners
        if args.save_map_only:
            # Stop fixture input, allow pending frames/map work to drain, then save.
            spin(.7)
            client = node.create_client(Trigger, '/zlocalization/save_map')
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 5.
            while not future.done() and time.monotonic() < end:
                spin(.05)
            assert future.done() and future.result().success, 'runtime map save failed'
            result['runtime_bundle_saved'] = future.result().message
            result['scope'] = 'reused deterministic fixture; file/tool connectivity only'
            (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2))
            return
        spin(.9)
        assert not received['status'][-1].valid, 'stale trusted pose survived lease'
        tf_count = len(received['tf'])
        spin(.2)
        new_corrections = [t for msg in received['tf'][tf_count:] for t in msg.transforms if t.header.frame_id == 'map' and t.child_frame_id == 'odom']
        assert not new_corrections, 'bridge renewed stale correction'
        result['input_loss_invalid_and_tf_stopped'] = True
        for j in range(12):
            drive(index + 1 + j)
        assert any(m.valid for m in received['status'][-10:]), 'same-session input did not resume'
        if args.event == 'invalidate':
            client = node.create_client(Trigger, '/zlocalization/invalidate')
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 3.
            while not future.done() and time.monotonic() < end:
                spin(.05)
            assert future.done() and future.result().success
        else:
            node.destroy_publisher(raw_pub)
            raw_pub = node.create_publisher(Odometry, '/odometry/fast_lio_raw', 20)
            spin(.4)
        for j in range(6):
            drive(index + 20 + j)
        assert not received['status'][-1].valid
        result[args.event + '_latched'] = True
        result['state_messages'] = len(received['state'])
        result['canonical_odometry_messages'] = len(received['odom'])
        result['scope'] = 'deterministic ROS fixture; no real sensor or localization performance evidence'
        (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        # launch forwards SIGINT to children; avoid delivering it twice.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
