#!/usr/bin/env python3
"""Static T-DT action frontend and real scan endpoint cloud. No dynamic truth input."""
import math
import struct
import os
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer, TransformListener
from temporal_mpc.frontend import prepare_route
from static_grid import grid_for


class Scene(Node):
    def __init__(self):
        super().__init__("temporal_static_frontend", parameter_overrides=[Parameter("use_sim_time",value=True)])
        self.scene=Path(os.environ["TEMPORAL_MPC_SCENE"])
        self.registration=json.loads((self.scene/"scene.json").read_text())
        self.frontend=os.environ["TEMPORAL_MPC_FRONTEND"]
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub=self.create_publisher(OccupancyGrid,"map",qos)
        self.path_pub=self.create_publisher(PathMessage,"plan",qos)
        self.cloud_pub=self.create_publisher(PointCloud2,"/simulation/scan_points",qos_profile_sensor_data)
        self.create_subscription(LaserScan,"scan",self.scan,qos_profile_sensor_data)
        self.map=None
        self.create_timer(.1,self.publish_map)
        self.server=ActionServer(self,ComputePathToPose,"compute_path_to_pose",self.execute, callback_group=ReentrantCallbackGroup())

    def publish_map(self):
        if self.map is not None or self.get_clock().now().nanoseconds <= 0: return
        message=OccupancyGrid()
        message.header.frame_id="map"; message.header.stamp=self.get_clock().now().to_msg()
        message.info.resolution=.05; message.info.width=160; message.info.height=120
        message.info.origin.position.x=-1.; message.info.origin.position.y=-3.
        message.info.origin.orientation.w=1.
        grid=grid_for(self.registration["scenario"])
        message.data=grid.ravel().tolist()
        self.costs=np.where(grid>=65,254,0).astype(np.uint8)
        self.map=message; self.map_pub.publish(message)

    def scan(self, scan):
        cloud=PointCloud2(); cloud.header=scan.header
        cloud.height=1
        cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                      for i,name in enumerate(("x","y","z"))]
        cloud.is_bigendian=False; cloud.point_step=12; cloud.is_dense=True
        points=[]
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min<=r<scan.range_max:
                a=scan.angle_min+i*scan.angle_increment
                points.append(struct.pack("<fff",r*math.cos(a),r*math.sin(a),0.))
        cloud.width=len(points); cloud.row_step=12*len(points); cloud.data=b"".join(points)
        self.cloud_pub.publish(cloud)

    def execute(self, handle):
        result=ComputePathToPose.Result()
        try:
            if self.map is None: raise ValueError("missing static map")
            request=handle.request
            if request.goal.header.frame_id!="map": raise ValueError("goal frame must be map")
            if request.use_start:
                if request.start.header.frame_id!="map": raise ValueError("start frame must be map")
                start=request.start.pose.position
                xy=[start.x,start.y]
            else:
                tf=self.buffer.lookup_transform("map","base_link",Time())
                xy=[tf.transform.translation.x,tf.transform.translation.y]
            goal=request.goal.pose.position
            route=prepare_route(self.frontend,self.costs,.05,(-1.,-3.),xy,[goal.x,goal.y])
            path=PathMessage(); path.header.frame_id="map"; path.header.stamp=self.get_clock().now().to_msg()
            for position in route.anchors:
                p=PoseStamped(); p.header=path.header
                p.pose.position.x=float(position[0]); p.pose.position.y=float(position[1]); p.pose.orientation.w=1.
                path.poses.append(p)
            result.path=path; self.path_pub.publish(path)
            (self.scene/"planned_route.json").write_text(json.dumps(route.document,indent=2)+"\n")
            handle.succeed()
            self.get_logger().info(f"T-DT frontend supplied {len(path.poses)} anchors")
        except Exception as error:
            self.get_logger().error(str(error)); handle.abort()
        return result

if __name__=="__main__":
    rclpy.init(); node=Scene()
    executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.try_shutdown()
