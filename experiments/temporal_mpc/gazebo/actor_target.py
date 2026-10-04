#!/usr/bin/env python3
"""Registered head-on PD target; no state or truth published by this node."""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

class Target(Node):
    def __init__(self):
        super().__init__('temporal_actor_target',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.output=self.create_publisher(Float64,'/simulation/moving_obstacle/target',1)
        self.create_timer(.05,self.tick)
    def tick(self):
        time=self.get_clock().now().nanoseconds/1e9
        # x-world target: 4.9 at fixed goal phase 10s -> 1.4 by 17s, then hold.
        self.output.publish(Float64(data=-min(3.5,max(0.,(time-10.)*.5))))
if __name__=='__main__':
    rclpy.init();node=Target()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
