#!/usr/bin/env python3
"""Replay actual serialized ROS TF; never admit oracle pose messages into TF."""
import argparse
import base64
import json
from pathlib import Path
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray
from rosidl_runtime_py.convert import message_to_ordereddict
from record_run import clean

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args();root=Path(args.directory)
    events=[json.loads(line) for line in (root/'events.jsonl').open()]
    rclpy.init();buffer=Buffer(cache_time=Duration(seconds=120.))
    kinds={'/tf':TFMessage,'/tf_static':TFMessage,'/odometry/lio':Odometry,
           '/scan':LaserScan,'/dynamic_obstacle_predictions':DynamicObstaclePredictionArray}
    cdr_checked=0;differences=[];predictions=[]
    for i,e in enumerate(events):
        topic=e['topic']
        if topic not in kinds:continue
        m=deserialize_message(base64.b64decode(e['cdr_b64']),kinds[topic]);cdr_checked+=1
        if clean(message_to_ordereddict(m))!=e['data']:differences.append(i)
        if topic in ('/tf','/tf_static'):
            for tf in m.transforms:
                if topic=='/tf':buffer.set_transform(tf,'recorded_canonical_tf')
                else:buffer.set_transform_static(tf,'recorded_canonical_static_tf')
        if topic=='/dynamic_obstacle_predictions':predictions.append(m)
    proofs=[]
    for m in predictions:
        ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        try:
            tf=buffer.lookup_transform('map','sim_lidar_link',Time.from_msg(m.header.stamp))
            proof=dict(source_ns=ns,available=True,transform=message_to_ordereddict(tf))
        except Exception as error:proof=dict(source_ns=ns,available=False,reason=str(error))
        proofs.append(proof)
    (root/'source_tf_replay.json').write_text(json.dumps(proofs,indent=2)+'\n')
    result=dict(serialized_messages_checked=cdr_checked,serialized_json_mismatches=differences,
                prediction_count=len(predictions),source_tf_replay_available=sum(p['available'] for p in proofs),
                source_tf_replay_failures=[p for p in proofs if not p['available']],
                method='Recorded canonical /tf and /tf_static only, exact source epoch, bounded interpolation; oracle excluded')
    (root/'ros_capture_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    rclpy.try_shutdown()
