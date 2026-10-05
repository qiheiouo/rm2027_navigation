#!/usr/bin/env python3
"""Offline decoding only: no ROS node, transform, state estimator or predictor."""
import argparse
import csv
import hashlib
import json
import math
import pathlib

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def source_ns(stamp):
    return stamp.sec*1000000000+stamp.nanosec


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('input',type=pathlib.Path)
    parser.add_argument('output',type=pathlib.Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    metadata={}
    for scene in ['S0','S1','S2']:
        bag=args.input/scene/'rosbag';reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        types={t.name:t.type for t in reader.get_all_topics_and_types()};counts={}
        files={kind:(args.output/(scene+'_'+kind+'.csv')).open('w') for kind in ['odometry','path','tf']}
        fields={'odometry':['bag_record_ns','source_ns','frame','child','x','y','yaw','qx','qy','qz','qw','vx','vy','wz'],
                'path':['bag_record_ns','source_ns','frame','poses','first_x','first_y','last_x','last_y'],
                'tf':['bag_record_ns','source_ns','frame','child','x','y','yaw']}
        writers={k:csv.DictWriter(f,fieldnames=fields[k],lineterminator='\n') for k,f in files.items()}
        for w in writers.values():w.writeheader()
        try:
            while reader.has_next():
                topic,raw,record_ns=reader.read_next();counts[topic]=counts.get(topic,0)+1
                if topic not in ['/odometry/lio','/plan','/tf']:continue
                m=deserialize_message(raw,get_message(types[topic]))
                if topic=='/odometry/lio':
                    p=m.pose.pose;q=p.orientation;t=m.twist.twist
                    writers['odometry'].writerow(dict(bag_record_ns=record_ns,source_ns=source_ns(m.header.stamp),frame=m.header.frame_id,child=m.child_frame_id,
                        x=p.position.x,y=p.position.y,yaw=yaw(q),qx=q.x,qy=q.y,qz=q.z,qw=q.w,vx=t.linear.x,vy=t.linear.y,wz=t.angular.z))
                elif topic=='/plan':
                    writers['path'].writerow(dict(bag_record_ns=record_ns,source_ns=source_ns(m.header.stamp),frame=m.header.frame_id,poses=len(m.poses),
                        first_x=m.poses[0].pose.position.x if m.poses else '',first_y=m.poses[0].pose.position.y if m.poses else '',
                        last_x=m.poses[-1].pose.position.x if m.poses else '',last_y=m.poses[-1].pose.position.y if m.poses else ''))
                else:
                    for tf in m.transforms:
                        if (tf.header.frame_id,tf.child_frame_id) not in [('map','odom'),('odom','base_link')]:continue
                        writers['tf'].writerow(dict(bag_record_ns=record_ns,source_ns=source_ns(tf.header.stamp),frame=tf.header.frame_id,child=tf.child_frame_id,
                            x=tf.transform.translation.x,y=tf.transform.translation.y,yaw=yaw(tf.transform.rotation)))
        finally:
            for f in files.values():f.close()
        metadata[scene]=dict(topic_counts=counts,input_files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in bag.iterdir() if p.is_file()})
    (args.output/'decode_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')


if __name__=='__main__':main()
