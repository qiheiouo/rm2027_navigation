#!/usr/bin/env python3
"""Actual DDS publisher identity, budget, invalid payload and no-overwrite checks."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Twist


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    record=args.output/'commands.jsonl';log=(args.output/'observer.log').open('w')
    command=['ros2','run','rm_dynamic_obstacle_critic','native_command_observer','--ros-args',
             '-p','input_topic:=/native_observer_test','-p','output_file:='+str(record.resolve()),'-p','max_records:=16']
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();controller=rclpy.create_node('fake_native_controller');behavior=rclpy.create_node('fake_native_behavior')
    pubs=[controller.create_publisher(Twist,'/native_observer_test',10),behavior.create_publisher(Twist,'/native_observer_test',10)]
    try:
        end=time.monotonic()+5
        while time.monotonic()<end and any(p.get_subscription_count()==0 for p in pubs):time.sleep(.02)
        if any(p.get_subscription_count()==0 for p in pubs):raise RuntimeError('observer discovery timeout')
        for _ in range(7):
            for pub,value in zip(pubs,[.2,-.3]):
                msg=Twist();msg.linear.x=value;pub.publish(msg);time.sleep(.02)
        for value in [float('nan'),.9]:
            msg=Twist();msg.linear.x=value;pubs[0].publish(msg);time.sleep(.02)
        end=time.monotonic()+3
        while time.monotonic()<end and (not record.exists() or len(record.read_text().splitlines())<16):time.sleep(.02)
        for _ in range(5):pubs[1].publish(Twist());time.sleep(.01)
        time.sleep(.1)
    finally:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        controller.destroy_node();behavior.destroy_node();rclpy.shutdown();log.close()
    data=record.read_bytes();rows=[json.loads(line) for line in data.splitlines()]
    controller_rows=[r for r in rows if r['publisher_node']=='fake_native_controller']
    behavior_rows=[r for r in rows if r['publisher_node']=='fake_native_behavior']
    gids={r['publisher_node']:{v['publisher_gid'] for v in rows if v['publisher_node']==r['publisher_node']} for r in rows}
    with (args.output/'duplicate.log').open('w') as duplicate_log:
        duplicate=subprocess.run(command,stdout=duplicate_log,stderr=subprocess.STDOUT,timeout=10)
    gates={'all_16_records':len(rows)==16,'ordinals_contiguous':[r['ordinal'] for r in rows]==list(range(16)),
        'known_publisher_identities':len(controller_rows)==9 and len(behavior_rows)==7,
        'distinct_exact_gids':len(gids)==2 and all(len(values)==1 for values in gids.values()) and gids.get('fake_native_controller',set())!=gids.get('fake_native_behavior',set()),
        'controller_values':sum(r['velocity'][0]==.2 for r in controller_rows)==7 and any(r['velocity'][0]==.9 for r in controller_rows),
        'behavior_values':all(r['velocity'][0]==-.3 for r in behavior_rows),
        'invalid_payload_explicit':sum(not r['finite'] and r['velocity'][0] is None for r in rows)==1,
        'budget_holds':len(rows)==16,'existing_evidence_preserved':duplicate.returncode!=0 and record.read_bytes()==data}
    result={'verdict':'PASS' if all(gates.values()) else 'FAILED','gates':gates,'rows':rows,
            'scope':'actual isolated DDS read-only observer; no optimizer-cycle or SG association'}
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'verdict':result['verdict'],'gates':gates}))
    return 0 if result['verdict']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
