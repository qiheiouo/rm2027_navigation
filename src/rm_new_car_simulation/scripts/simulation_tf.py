#!/usr/bin/env python3
"""Explicit simulation oracle. Never claims to be LIO or wheel odometry."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SimulationTF(Node):
    def __init__(self):
        super().__init__('new_car_simulation_truth_tf')
        self.publish_tf=self.declare_parameter('publish_truth_tf',True).value
        self.tf=TransformBroadcaster(self)
        self.static=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.
        if self.publish_tf:self.static.sendTransform(t)
        self.pub=self.create_publisher(Odometry,'/odometry/simulation',20)
        self.sub=self.create_subscription(Odometry,'/simulation/ground_truth/odom',self.odom,20)
        self.get_logger().info(f'SIMULATION ORACLE: TF publishing={self.publish_tf}; full SE(3) physics truth. Not localization validation.')

    def odom(self,msg):
        msg.header.frame_id='odom';msg.child_frame_id='base_link';self.pub.publish(msg)
        t=TransformStamped();t.header=msg.header;t.child_frame_id='base_link'
        t.transform.translation.x=msg.pose.pose.position.x
        t.transform.translation.y=msg.pose.pose.position.y
        t.transform.translation.z=msg.pose.pose.position.z
        t.transform.rotation=msg.pose.pose.orientation
        if self.publish_tf:self.tf.sendTransform(t)


def main():
    rclpy.init();node=SimulationTF()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__': main()
