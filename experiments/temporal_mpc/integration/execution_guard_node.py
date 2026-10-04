#!/usr/bin/env python3
"""Experimental final command publisher, common to both Nav2 controllers."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray
from temporal_mpc.contracts import PublicAdapter, stamp_ns
from temporal_mpc.execution_guard import ExecutionGuard, StaticCells, GuardResult


class GuardNode(Node):
    def __init__(self):
        super().__init__('temporal_execution_guard',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.guard=ExecutionGuard(); self.adapter=PublicAdapter(geometry_mode='nominal_diameter')
        self.snapshot=self.cells=self.odom=self.request=None
        self.odom_receipt=self.request_receipt=0.
        self.map_revision=None; self.ticks=0; self.fault=""
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'map',self.map, qos)
        self.create_subscription(DynamicObstaclePredictionArray,'dynamic_obstacle_predictions',self.predictions,1)
        self.create_subscription(Odometry,'/odometry/lio',self.measured,1)
        self.create_subscription(Twist,'temporal_mpc/smoothed_cmd_vel',self.command,1)
        self.create_subscription(String,'temporal_mpc/test_guard_fault',self.test_fault,1)
        self.output=self.create_publisher(Twist,'cmd_vel',1)
        self.diagnostic=self.create_publisher(String,'temporal_mpc/execution_health',10)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def map(self,m):
        self.cells=None
        i=m.info; q=i.origin.orientation
        try:
            if m.header.frame_id!='map' or (q.x,q.y,q.z,q.w)!=(0.,0.,0.,1.): return
            grid=np.asarray(m.data).reshape(i.height,i.width)
            if np.any((grid < -1)|(grid > 100)): return
            costs=np.where((grid<0)|(grid>=65),254,0).astype(np.uint8)
            self.cells=StaticCells(costs,i.resolution,(i.origin.position.x,i.origin.position.y))
            self.map_revision=stamp_ns(m.header.stamp)
        except (ValueError,TypeError): pass

    def predictions(self,m):
        self.snapshot=None
        try: self.snapshot=self.adapter.consume(m,self.get_clock().now().nanoseconds)
        except (ValueError,TypeError): pass

    def measured(self,m):
        self.odom=m; self.odom_receipt=time.monotonic()

    def command(self,m):
        self.request=m; self.request_receipt=time.monotonic()

    def test_fault(self,m):
        if m.data in ('','exception'): self.fault=m.data

    def tick(self):
        started=time.perf_counter(); self.ticks+=1
        previous=self.guard.previous.copy()
        epoch=self.get_clock().now().nanoseconds; wall=time.monotonic()
        initial=np.full(6,np.nan); age=float('inf'); requested=np.full(3,np.nan); odom_ns=None
        if self.request and wall-self.request_receipt<=.15:
            requested=np.array([self.request.linear.x,self.request.linear.y,self.request.angular.z])
        if self.odom and wall-self.odom_receipt<=.15:
            m=self.odom; q=m.pose.pose.orientation; v=m.twist.twist
            try:
                odom_ns=stamp_ns(m.header.stamp)
                if m.header.frame_id!='odom' or m.child_frame_id!='base_link': raise ValueError('odom frame')
                if abs(q.x)>1e-6 or abs(q.y)>1e-6 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:
                    raise ValueError('nonplanar odom')
                transform=self.buffer.lookup_transform('map','odom',Time.from_msg(m.header.stamp))
                t=transform.transform; r=t.rotation
                if abs(r.x)>1e-6 or abs(r.y)>1e-6 or abs(r.z*r.z+r.w*r.w-1)>1e-3:
                    raise ValueError('nonplanar map transform')
                angle=2*math.atan2(r.z,r.w); c,s=math.cos(angle),math.sin(angle)
                p=m.pose.pose.position
                initial=np.array([t.translation.x+c*p.x-s*p.y,t.translation.y+s*p.x+c*p.y,
                                  angle+2*math.atan2(q.z,q.w),v.linear.x,v.linear.y,v.angular.z])
                age=(epoch-stamp_ns(m.header.stamp))*1e-9
            except Exception: pass  # Core emits bounded uncertified braking for unavailable input.
        try:
            if self.fault=='exception': raise RuntimeError('experimental fault')
            result=self.guard.step(initial,requested,epoch,self.snapshot,self.cells,age)
        except Exception as error:
            # Unexpected solver/data boundary failures must preserve an output.
            command=np.sign(previous)*np.maximum(np.abs(previous)-[.05,.05,0.],0.)
            self.guard.previous=command.copy()
            result=GuardResult(command,'uncertified_brake',type(error).__name__+': guard boundary',False,None,
                               time.perf_counter()-started)
        msg=Twist(); msg.linear.x=float(result.command[0]); msg.linear.y=float(result.command[1])
        self.output.publish(msg)
        data=dict(epoch_ns=epoch,tick_count=self.ticks,map_revision=self.map_revision,
                  initial=[float(v) if math.isfinite(v) else None for v in initial],
                  requested=[float(v) if math.isfinite(v) else None for v in requested],previous=previous.tolist(),
                  odom_ns=odom_ns,source_ns=self.snapshot.source_ns if self.snapshot else None,
                  model='fixed_yaw_velocity_zoh/v1',status=result.status,reason=result.reason,
                  model_certified=result.model_certified,constraint_min=result.constraint_min,
                  elapsed_s=result.elapsed_s,tick_to_output_s=time.perf_counter()-started,command=result.command.tolist(),state_age_s=age if math.isfinite(age) else None)
        self.diagnostic.publish(String(data=json.dumps(data,allow_nan=False)))


def main():
    rclpy.init(); node=GuardNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__=='__main__': main()
