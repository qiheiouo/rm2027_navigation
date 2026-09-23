#!/usr/bin/env python3
"""Reuse the frozen dynamic observer and record the filtered chassis input."""
import sys
import time

sys.path.insert(0, "/ws/docs/tdt_migration/evidence/mppi_cycle_diagnostic_20260922")
from observe_cycle import CycleObserver, original
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


class GuardObserver(CycleObserver):
    def __init__(self, output, launch_log):
        super().__init__(output, launch_log)
        self.refs.append(self.create_subscription(
            Twist, "/cmd_vel_guarded", self.make_cb("/cmd_vel_guarded", Twist),
            QoSProfile(depth=100)))


original.Observer = GuardObserver
if __name__ == "__main__":
    sys.exit(original.main())
