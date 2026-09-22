"""Observe dynamic pilot; existing static summary stays explicitly static-only."""
from pathlib import Path
import json
import math
import os
import sys
sys.path.insert(0,'/ws/docs/tdt_migration/evidence/snapshot_revalidation_20260922')
from observe_reference import ReferenceObserver, original
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy

class DynamicObserver(ReferenceObserver):
    def __init__(self,output,launch_log):
        super().__init__(output,launch_log)
        self.phase_at=float(os.environ['TDT_PHASE_SECONDS']);self.start_after=None
        for name in ('moving_target','local_costmap','scans'):
            self.streams[name]=(output/f'{name}.jsonl').open('x',buffering=1)
        self.refs.extend([
            self.create_subscription(Float64,'/simulation/moving_obstacle/target',lambda m:self.write('moving_target',{'t':self.now_sim,'target':m.data}),10),
            self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.local_map_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))])
    def local_map_cb(self,msg):
        self.write('local_costmap',{'t':original.seconds(msg.header.stamp),'frame':msg.header.frame_id,
            'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,
            'origin':[msg.info.origin.position.x,msg.info.origin.position.y],'data':list(msg.data)})
    def scan_cb(self,msg):
        super().scan_cb(msg)
        self.write('scans',{'t':original.seconds(msg.header.stamp),'frame':msg.header.frame_id,
            'angle_min':msg.angle_min,'angle_increment':msg.angle_increment,'range_min':msg.range_min,'range_max':msg.range_max,
            'ranges':[float(x) if math.isfinite(x) else ('inf' if x>0 else 'invalid') for x in msg.ranges]})
    def ready(self):
        if not super().ready() or self.now_sim<16.:return False
        if '[TDT_DYNAMIC] spawn_moving_obstacle exit=0' not in self.launch_log.read_text():return False
        if self.start_after is None:
            self.start_after=8*math.ceil((self.now_sim-self.phase_at)/8)+self.phase_at
            self.write('events',{'event':'prespecified_phase','t':self.now_sim,'period_s':8.,'phase_s':self.phase_at,'release_sim_t':self.start_after})
        return self.now_sim>=self.start_after
original.Observer=DynamicObserver
if __name__=='__main__':sys.exit(original.main())
