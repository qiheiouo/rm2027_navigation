#!/usr/bin/env python3
"""Decode recorded public/private messages only; no prediction computation."""
import json,pathlib,sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
out=pathlib.Path(sys.argv[1]);reader=rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
types={t.name:t.type for t in reader.get_all_topics_and_types()};counts={}
with (out/'prediction_messages.jsonl').open('w') as f:
    while reader.has_next():
        topic,raw,ns=reader.read_next();counts[topic]=counts.get(topic,0)+1
        if topic not in ['/perception/dynamic_obstacles_shadow/predictions','/perception/dynamic_obstacles_shadow/observed_predictions']:continue
        msg=deserialize_message(raw,get_message(types[topic]));data=message_to_ordereddict(msg)
        f.write(json.dumps(dict(topic=topic,bag_record_ns=ns,message=data),default=lambda x:x.tolist() if hasattr(x,'tolist') else list(x),separators=(',',':'))+'\n')
(out/'bag_message_counts.json').write_text(json.dumps(counts,indent=2)+'\n')
