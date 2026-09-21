#!/usr/bin/env python3
"""Capture a complete raw costmap after navigation, not a planner-time snapshot."""
import json
from pathlib import Path
import sys
import rclpy
from rclpy.parameter import parameter_value_to_python
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.srv import GetParameters
import yaml

rclpy.init()
node=rclpy.create_node('tdt_new_car_geometry_capture')
profile=yaml.safe_load(Path(sys.argv[1]).read_text())
report={'scope':'post-navigation raw service snapshot; not the snapshot of any earlier rejection', 'runtime':{}}
try:
    for name in ('local_costmap','global_costmap'):
        client=node.create_client(GetParameters,f'/{name}/{name}/get_parameters')
        if not client.wait_for_service(timeout_sec=10): raise RuntimeError('missing '+name)
        keys=['footprint','footprint_padding']
        future=client.call_async(GetParameters.Request(names=keys))
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.result() is None: raise RuntimeError('parameter timeout')
        actual=dict(zip(keys,[parameter_value_to_python(v) for v in future.result().values]))
        expected={k:profile[name][name]['ros__parameters'][k] for k in keys}
        if actual!=expected: raise RuntimeError('runtime footprint differs from profile')
        report['runtime'][name]={'expected':expected,'actual':actual}
    client=node.create_client(GetCostmap,'/global_costmap/get_costmap')
    if not client.wait_for_service(timeout_sec=10): raise RuntimeError('missing raw costmap')
    future=client.call_async(GetCostmap.Request())
    rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    if not future.done() or future.result() is None: raise RuntimeError('raw map timeout')
    msg=future.result().map; meta=msg.metadata
    data=[v[0] if isinstance(v,bytes) and len(v)==1 else int(v) for v in msg.data]
    q=meta.origin.orientation
    if msg.header.frame_id!='map' or abs(q.x)+abs(q.y)+abs(q.z)>1e-9 or abs(abs(q.w)-1)>1e-9:
        raise RuntimeError('unsupported costmap frame/orientation')
    if meta.size_x*meta.size_y!=len(data) or not data: raise RuntimeError('invalid raw map size')
    report['map']={'frame':msg.header.frame_id,'width':meta.size_x,'height':meta.size_y,
        'resolution':meta.resolution,'origin':[meta.origin.position.x,meta.origin.position.y],
        'stamp':[msg.header.stamp.sec,msg.header.stamp.nanosec],'data':data}
    with Path(sys.argv[2]).open('x') as f:json.dump(report,f,allow_nan=False);f.write('\n')
finally:
    node.destroy_node();rclpy.shutdown()
