#!/usr/bin/env python3
"""Real Humble message bindings at the recorder boundary; no node or Gazebo startup."""
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from nav_msgs.msg import OccupancyGrid, Path as NavPath
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message, serialize_message

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
import observe_simulation as observer


def roundtrip(msg):
    # Exercise generated Python/C bindings and signed arrays, without a DDS participant.
    return deserialize_message(serialize_message(msg), type(msg))


class Sink:
    """Use the production JSON writer and callbacks without constructing a ROS node."""
    write = observer.Observer.write

    def __init__(self):
        self.now_sim = 0.0
        self.events = []
        self.map_count = self.lethal_cells = 0
        self.streams = {name: io.StringIO() for name in ('events', 'costmap')}


class RecorderMessagesTest(unittest.TestCase):
    def test_low_severity_rosout_before_clock_or_costmap_is_ignored(self):
        sink = Sink()
        for level in (0, 10, 20, 29):
            with self.subTest(level=level):
                msg = roundtrip(Log(level=level, name='startup', msg='before costmap'))
                observer.Observer.log_cb(sink, msg)
        self.assertEqual(sink.events, [])
        self.assertEqual(sink.streams['events'].getvalue(), '')
        self.assertEqual(sink.map_count, 0)

    def test_warning_and_higher_survive_ros_serialization_and_json(self):
        sink = Sink()
        for level in (30, 40, 50, 255):
            msg = roundtrip(Log(level=level, name='planner', msg='保留原始告警'))
            observer.Observer.log_cb(sink, msg)
        rows = [json.loads(line) for line in sink.streams['events'].getvalue().splitlines()]
        self.assertEqual(rows, sink.events)
        self.assertEqual([row['level'] for row in rows], [30, 40, 50, 255])
        for row in rows:
            self.assertIs(type(row['level']), int)
            self.assertEqual(row['node'], 'planner')
            self.assertEqual(row['message'], '保留原始告警')
            self.assertEqual(row['t'], 0.0)

    def test_costmap_signed_unknown_and_occupancy_values_remain_exact(self):
        sink = Sink()
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = 7
        msg.header.stamp.nanosec = 250000000
        msg.info.width, msg.info.height = 3, 2
        msg.info.resolution = 0.05
        msg.info.origin.position.x, msg.info.origin.position.y = -1.5, -2.0
        msg.data = [-1, 0, 98, 99, 100, 0]
        observer.Observer.map_cb(sink, roundtrip(msg))
        row = json.loads(sink.streams['costmap'].getvalue())
        self.assertEqual(row['data'], [-1, 0, 98, 99, 100, 0])
        self.assertEqual(sink.lethal_cells, 2)
        self.assertEqual(sink.map_count, 1)
        self.assertEqual((row['frame'], row['width'], row['height']), ('map', 3, 2))
        self.assertEqual(row['origin'], [-1.5, -2.0])
        self.assertAlmostEqual(row['resolution'], 0.05, places=7)
        self.assertEqual(row['t'], 7.25)

    def test_plan_yaw_survives_ros_roundtrip_and_keeps_xy(self):
        msg = NavPath()
        msg.header.frame_id = 'map'
        pose = PoseStamped()
        pose.pose.position.x = 2.0
        pose.pose.orientation.z = 1.0
        pose.pose.orientation.w = 0.0
        msg.poses = [pose]
        sink = Sink()
        sink.path_id = 0
        sink.streams['plans'] = io.StringIO()
        observer.Observer.plan_cb(sink, roundtrip(msg))
        row = json.loads(sink.streams['plans'].getvalue())
        self.assertEqual(row['xy'], [[2.0, 0.0]])
        self.assertAlmostEqual(abs(row['yaw'][0]), 3.141592653589793)

    def test_callback_failure_keeps_traceback_and_returns_invalid_evidence(self):
        # Main's failure handler is exercised, but ROS init/node creation are mocked.
        log_cb = observer.Observer.log_cb
        msg = roundtrip(Log(level=30, name='recorder', msg='write failure'))
        sink = SimpleNamespace(now_sim=0.0, events=[], streams={'events': io.StringIO()},
                               destroy_node=Mock())
        sink.write = Mock(side_effect=OSError('recorder write failed'))
        sink.run = lambda x, y: log_cb(sink, msg)
        stderr, stdout = io.StringIO(), io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'observation'
            argv = ['observe_simulation.py', str(output), '--launch-log', str(Path(directory)/'launch.log')]
            with patch.object(sys, 'argv', argv), \
                    patch.dict('os.environ', {'ROS_DOMAIN_ID': '174', 'ROS_LOCALHOST_ONLY': '1'}), \
                    patch.object(observer.rclpy, 'init') as init, \
                    patch.object(observer.rclpy, 'shutdown') as shutdown, \
                    patch.object(observer, 'Observer', return_value=sink), \
                    redirect_stderr(stderr), redirect_stdout(stdout):
                code = observer.main()
            report = json.loads((output / 'summary.json').read_text())
        self.assertEqual(code, 2)
        self.assertFalse(report['evidence_valid'])
        self.assertFalse(report['static_geometry_and_goal_pass'])
        self.assertEqual(report['error_type'], 'OSError')
        self.assertEqual(report['error'], 'recorder write failed')
        self.assertIn('in log_cb', report['traceback'])
        self.assertIn('OSError: recorder write failed', report['traceback'])
        self.assertIn(report['traceback'], stderr.getvalue())
        self.assertEqual(json.loads(stdout.getvalue()), report)
        self.assertTrue(sink.streams['events'].closed)
        sink.destroy_node.assert_called_once_with()
        init.assert_called_once_with()
        shutdown.assert_called_once_with()


if __name__ == '__main__':
    unittest.main(verbosity=2)
