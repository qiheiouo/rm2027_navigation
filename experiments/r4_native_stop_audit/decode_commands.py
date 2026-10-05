#!/usr/bin/env python3
"""Read original bag commands and recorder clock brackets; no node or publisher."""
import argparse
import bisect
import csv
import hashlib
import json
import pathlib
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    p=argparse.ArgumentParser();p.add_argument('input',type=pathlib.Path);p.add_argument('output',type=pathlib.Path)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True);manifest={}
    for scene in ['S0','S1','S2']:
        bag=args.input/scene/'rosbag';r=rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        types={x.name:x.type for x in r.get_all_topics_and_types()};commands=[];clocks=[];counts={}
        while r.has_next():
            topic,data,receipt=r.read_next();counts[topic]=counts.get(topic,0)+1
            if topic not in ['/clock','/cmd_vel_nav','/cmd_vel','/simulation/chassis/cmd_vel']:continue
            m=deserialize_message(data,get_message(types[topic]))
            if topic=='/clock':
                clocks.append((receipt,m.clock.sec*10**9+m.clock.nanosec))
            else:
                commands.append(dict(topic=topic,bag_record_ns=receipt,vx=m.linear.x,vy=m.linear.y,wz=m.angular.z))
        clocks.sort();clock_receipts=[x[0] for x in clocks]
        assert all(b[1]>a[1] for a,b in zip(clocks,clocks[1:])), 'original recorder clock not monotonic'
        for c in commands:
            k=bisect.bisect_right(clock_receipts,c['bag_record_ns'])
            c['recorder_clock_before_ns']=clocks[k-1][1] if k else ''
            c['recorder_clock_after_ns']=clocks[k][1] if k<len(clocks) else ''
        with (args.output/(scene+'_commands.csv')).open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(commands[0]),lineterminator='\n');w.writeheader();w.writerows(commands)
        manifest[scene]=dict(counts=counts,command_rows=len(commands),clock_rows=len(clocks),
            bag_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in bag.iterdir() if p.is_file()},
            meaning='Recorder receipt clock brackets only; Twist has no source/send stamp. No Gazebo delivery acknowledgement recorded.')
    (args.output/'decode_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':main()
