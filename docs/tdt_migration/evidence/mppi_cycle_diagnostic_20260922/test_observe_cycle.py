"""Guard the Humble one-argument subscription callback and metadata semantics."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parent))
from observe_cycle import CycleObserver, Twist, info_row


class ObserverCallbackTest(unittest.TestCase):
    def test_humble_message_only(self):
        records = []
        fake = SimpleNamespace(now_sim=1.25, write=lambda stream, record: records.append((stream, record)))
        msg = Twist()
        msg.linear.x = 0.42
        CycleObserver.make_cb(fake, '/cmd_vel_nav', Twist)(msg)
        self.assertEqual(len(records), 1)
        stream, record = records[0]
        self.assertEqual(stream, 'command_chain')
        self.assertEqual(record['vx'], 0.42)
        self.assertIsNone(record['publisher_gid'])
        self.assertIsNone(record['received_system_ns'])
        self.assertIsInstance(record['callback_steady_ns'], int)

    def test_metadata_when_available(self):
        info = SimpleNamespace(publisher_gid=bytes([1, 2]), source_timestamp=3, received_timestamp=4)
        record = info_row(info)
        self.assertEqual(record['publisher_gid'], '0102')
        self.assertEqual(record['source_system_ns'], 3)
        self.assertEqual(record['received_system_ns'], 4)


if __name__ == '__main__':
    unittest.main()
