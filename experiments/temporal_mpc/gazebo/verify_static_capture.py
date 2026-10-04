#!/usr/bin/env python3
"""Two late-join DDS static publishers must both reach the raw recorder."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from prepare_scene import prepare
from record_run import Recorder


def main():
    p=argparse.ArgumentParser();p.add_argument('output');root=Path(p.parse_args().output)
    if root.exists():raise SystemExit('refuse fixture overwrite')
    prepare(root/'scene',profile='open_long')
    rclpy.init();nodes=[Node('capture_static_'+str(i)) for i in range(2)]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    publishers=[n.create_publisher(TFMessage,'/tf_static',qos) for n in nodes]
    messages=[]
    for parent,child in [('map','odom'),('base_link','sim_lidar_link')]:
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.
        messages.append(TFMessage(transforms=[t]))
    # Only one publication per writer, BEFORE recorder discovery.
    for publisher,message in zip(publishers,messages):publisher.publish(message)
    recorder=Recorder(root,'shadow');empty_gate=not recorder.static_chain_recorded()
    limit=time.monotonic()+5.
    while time.monotonic()<limit and not recorder.static_chain_recorded():rclpy.spin_once(recorder,timeout_sec=.01)
    result=dict(empty_recording_rejected=empty_gate,two_writer_raw_chain_captured=recorder.static_chain_recorded(),
                raw_static_message_count=recorder.counts['/tf_static'],edges=sorted(recorder.recorded_static_edges),
                scope='Actual DDS late join to two once-only transient-local writers; no synthetic TF added to physical evidence.')
    recorder.stream.close();recorder.destroy_node()
    for node in nodes:node.destroy_node()
    rclpy.try_shutdown()
    result['pass']=bool(empty_gate and result['two_writer_raw_chain_captured'] and result['raw_static_message_count']==2)
    (root/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if not result['pass']:raise SystemExit(1)


if __name__=='__main__':main()
