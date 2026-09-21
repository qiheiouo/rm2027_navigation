#!/usr/bin/env python3
"""Reuse the existing observer; capture sparse planner diagnostics separately."""
from pathlib import Path
import sys
sys.path.insert(0,'/ws/experiments/tdt_planner/rm_tdt_planner/tools')
import observe_simulation as original

class ReferenceObserver(original.Observer):
    def __init__(self,output,launch_log):
        super().__init__(output,launch_log)
        self.streams['planner_diagnostics']=(output/'planner_diagnostics.jsonl').open('x',buffering=1)

    def log_cb(self,msg):
        if msg.name=='planner_server':
            self.write('planner_diagnostics',{'t':self.now_sim,'node':msg.name,'level':msg.level,'message':msg.msg})
        super().log_cb(msg)

original.Observer=ReferenceObserver
if __name__=='__main__':sys.exit(original.main())
