#!/usr/bin/env python3
"""Publish the frozen fixture static map in the local costmap's odom frame."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'v1_tracker_probe_20260924'))
from probe import static_map
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
class MapNode(Node):
    def __init__(self):
        super().__init__('prediction_v1_local_frame_static_map')
        occupancy,_=static_map()
        msg=OccupancyGrid()
        msg.header.frame_id='odom'
        msg.info.width=occupancy.width;msg.info.height=occupancy.height
        msg.info.resolution=occupancy.resolution
        msg.info.origin.position.x=occupancy.origin_x
        msg.info.origin.position.y=occupancy.origin_y
        msg.info.origin.orientation.w=1.
        msg.data=occupancy.data
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.create_publisher(OccupancyGrid,'/prediction_v1/static_map',qos)
        self.message=msg
        self.timer=self.create_timer(2.,self.publish)
        self.publish()
    def publish(self):
        self.message.header.stamp=self.get_clock().now().to_msg()
        self.publisher.publish(self.message)
if __name__=='__main__':
    rclpy.init()
    node=MapNode()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
