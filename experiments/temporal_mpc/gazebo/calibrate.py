#!/usr/bin/env python3
"""Bounded pulse identification only, with measured odometry recorded independently."""
import argparse
from pathlib import Path
import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from record_run import Recorder

# Fixed sim phases, zero wz; no actor or navigation goal in this separate fixture.
PULSES=((2.,5.,.4,0.),(8.,11.,0.,.3),(14.,17.,.4,.3),(20.,23.,-.3,0.))

def target(t):
    for begin,end,x,y in PULSES:
        if begin<=t<end: return x,y
    return 0.,0.

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    rclpy.init();node=Recorder(args.output,'calibration')
    command=node.create_publisher(Twist,'/cmd_vel',10)
    node.event('calibration_registration',dict(pulses=PULSES,accel=1.,period=.05,cmd_wz=0.))
    wall=time.monotonic();next_step=0.;last=0.;vx=vy=0.;reason='wall timeout'
    try:
        while time.monotonic()-wall<90:
            rclpy.spin_once(node,timeout_sec=.005)
            now=node.get_clock().now().nanoseconds/1e9
            if now>=next_step:
                dt=min(.05,max(0.,now-last));last=now;next_step=now+.05
                tx,ty=target(now)
                vx+=max(-dt,min(dt,tx-vx));vy+=max(-dt,min(dt,ty-vy))
                msg=Twist();msg.linear.x=vx;msg.linear.y=vy;command.publish(msg)
                node.event('calibration_command',dict(vx=vx,vy=vy,wz=0.,target=[tx,ty]))
            if now>=26.:reason='calibration complete';break
        command.publish(Twist())
        end=time.monotonic()+.3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        node.graph()
        summary=dict(mode='calibration',stop_reason=reason,goal_epoch_s=None,
                     final_sim_s=node.get_clock().now().nanoseconds/1e9,wall_s=time.monotonic()-wall,
                     result=None,mpc_requested=False,counts=dict(node.counts),physical_acceptance=False,
                     input_gate_pending=True)
        (Path(args.output)/'run_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2),flush=True)
    finally:node.stream.close();node.destroy_node();rclpy.try_shutdown()
