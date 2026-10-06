"""DDS geometry/source identity, coasting/reset, rejected input and sink budget."""
import json
import math
import time

import pytest
pytest.importorskip('rclpy')
pytest.importorskip('rm_competition_interfaces.msg')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from diagnostic_msgs.msg import DiagnosticArray
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray as Array
from rm_dynamic_obstacle_tracking.dynamic_obstacle_tracker_node import DynamicObstacleTrackerNode


def ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


@pytest.mark.parametrize('mode',['filtered','last_observation_cv'])
@pytest.mark.parametrize('exhaust_budget',[False,True])
def test_actual_scan_members_and_public_geometry_survive_reset_and_refusal(mode,exhaust_budget,tmp_path):
    sink=tmp_path/'member_evidence'
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','tracker.min_hits_to_confirm:=1',
        '-p','prediction.max_tracks:=1','-p','prediction.steps:=30',
        '-p','prediction.anchor_mode:='+mode,'-p','prediction.max_speed:=0.0',
        '-p','prediction.velocity_decay_tau:=0.0','-p','surface_evidence_directory:='+str(sink)])
    tracker=DynamicObstacleTrackerNode();source=Node('surface_member_fixture')
    executor=SingleThreadedExecutor();executor.add_node(tracker);executor.add_node(source)
    received=[];diagnostics=[]
    subscriptions=[source.create_subscription(Array,tracker._predictions_topic,received.append,10),
                   source.create_subscription(DiagnosticArray,tracker._diagnostics_topic,diagnostics.append,10)]
    scan_pub=source.create_publisher(LaserScan,tracker._scan_topic,qos_profile_sensor_data)
    map_pub=source.create_publisher(OccupancyGrid,tracker._map_topic,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clock_pub=source.create_publisher(Clock,'/clock',10)
    broadcaster=StaticTransformBroadcaster(source)
    epoch=1_700_000_000_123_456_789

    def until(predicate):
        deadline=time.monotonic()+3
        while not predicate() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        assert predicate(),'DDS member fixture timed out'

    def clock_at(stamp):
        clock_pub.publish(Clock(clock=Time(nanoseconds=stamp).to_msg()))
        until(lambda:tracker.get_clock().now().nanoseconds==stamp)

    def message(stamp,ranges):
        scan=LaserScan();scan.header.frame_id='member_laser';scan.header.stamp=Time(nanoseconds=stamp).to_msg()
        scan.angle_min=-.03;scan.angle_increment=.01;scan.angle_max=-.03+(len(ranges)-1)*.01
        scan.range_min=.1;scan.range_max=10.;scan.ranges=ranges
        return scan

    def scan_at(stamp,ranges):
        clock_at(stamp)
        broadcaster.sendTransform(transform)
        until(lambda:tracker._tf_buffer.can_transform('map','member_laser',Time()))
        scan=message(stamp,ranges);before=len(received);scan_pub.publish(scan)
        until(lambda:len(received)==before+1)
        return deserialize_message(serialize_message(scan),LaserScan),received[-1]

    try:
        until(lambda:scan_pub.get_subscription_count()==1 and map_pub.get_subscription_count()==1 and clock_pub.get_subscription_count()>=1)
        clock_at(epoch)
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='member_laser'
        transform.transform.translation.x=1.;transform.transform.translation.y=-2.
        transform.transform.rotation.z=math.sin(.2);transform.transform.rotation.w=math.cos(.2)
        broadcaster.sendTransform(transform)
        grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=Time(nanoseconds=epoch).to_msg()
        grid.info.width=grid.info.height=100;grid.info.resolution=.1
        grid.info.origin.position.x=grid.info.origin.position.y=-5.;grid.info.origin.orientation.w=1.;grid.data=[0]*10000
        map_pub.publish(grid);until(lambda:tracker._map is not None)
        map_records=[json.loads(p.read_text()) for p in sorted(sink.glob('map_*.json'))]
        assert len(map_records)==1 and map_records[0]['data']==list(grid.data)
        scans=[];outputs=[]
        for stamp,ranges in [(epoch,[2.,2.,2.,math.nan,4.,4.,4.]),(epoch+100_000_000,[]),(epoch,[2.,2.,2.,math.nan,4.,4.,4.])]:
            scan,public=scan_at(stamp,ranges);scans.append(scan);outputs.append(public)
        records=[json.loads(p.read_text()) for p in sorted(sink.glob('scan_*.json'))]
        assert len(records)==3 and all(r['status']=='accepted' for r in records)
        assert records[0]['scan']['ranges'][3]=='NaN'
        assert records[0]['candidate_source_indices']==[0,1,2,4,5,6]
        assert sorted(sorted(d['source_indices']) for d in records[0]['detections'])==[[0,1,2],[4,5,6]]
        assert records[0]['tracker_update']['detection_track_ids']==[1,2]
        assert not records[1]['detections'] and not records[1]['tracker_update']['detection_track_ids']
        assert all(t['misses']==1 for t in records[1]['tracker_update']['tracks'])
        assert records[2]['tracker_update']['time_reset']
        assert records[2]['tracker_update']['detection_track_ids']==[3,4]
        for record,scan,public in zip(records,scans,outputs):
            assert record['source_stamp_ns']==ns(scan.header.stamp)==record['source_tf']['requested_stamp_ns']
            assert record['callback_stamp_ns']==ns(scan.header.stamp)
            assert record['map_identity']['stamp_ns']==epoch
            assert record['source_tf']['frame']=='map' and record['source_tf']['child_frame']=='member_laser'
            for index,x,y in record['projected_endpoints']:
                a=scan.angle_min+index*scan.angle_increment
                assert abs(x-(1+scan.ranges[index]*math.cos(.4+a)))<1e-12
                assert abs(y-(-2+scan.ranges[index]*math.sin(.4+a)))<1e-12
            trace=record['public_prediction']
            assert trace['schema']==public.schema and trace['source_stamp_ns']==ns(public.header.stamp)
            assert trace['processing_stamp_ns']==ns(public.processing_stamp)
            assert trace['complete']==public.complete and trace['total_track_count']==public.total_track_count
            assert trace['frame']==public.header.frame_id and trace['authority']==public.authority
            assert trace['prediction_dt']==public.prediction_dt and trace['prediction_steps']==public.prediction_steps
            assert len(trace['tracks'])==len(public.tracks)==1
            for t,p in zip(trace['tracks'],public.tracks):
                assert t['id']==p.track_id and t['state']==p.state
                assert t['xy']==[p.position.x,p.position.y] and t['vxy']==[p.velocity.x,p.velocity.y]
                assert t['size_xy']==[p.size.x,p.size.y]
                assert [t['position_z'],t['velocity_z'],t['size_z']]==[p.position.z,p.velocity.z,p.size.z]
                assert t['last_observation_stamp_ns']==ns(p.last_observation_stamp)
                assert t['observation_count']==p.observation_count and t['miss_count']==p.miss_count
                assert t['prediction_xyz']==[[v.x,v.y,v.z] for v in p.prediction]
            assert not public.complete and public.total_track_count==2
        before=len(received);scan_pub.publish(message(epoch-1_000_000_000,[2.,2.,2.]))
        until(lambda:len(list(sink.glob('scan_*.json')))==4)
        rejected=json.loads(sorted(sink.glob('scan_*.json'))[-1].read_text())
        assert rejected['status']=='rejected' and rejected['reason']=='stale_scan' and len(received)==before
        if exhaust_budget:
            _,public=scan_at(epoch+100_000_000,[math.nan]*4097)
            assert not tracker._surface_member_evidence.complete
            assert json.loads((sink/'incomplete.json').read_text())['reason']=='complete scan beam budget exceeded'
            assert len(list(sink.glob('scan_*.json')))==4
        (tmp_path/'fixture_summary.json').write_text(json.dumps({'mode':mode,'source_epochs_exact':True,'full_public_fields_exact':True,'map_tf_members_verified':True,'coasting_reset_and_rejection_verified':True,'incomplete_budget_refused':exhaust_budget},indent=2)+'\n')
    finally:
        executor.remove_node(source);executor.remove_node(tracker)
        source.destroy_node();tracker.destroy_node();executor.shutdown();rclpy.shutdown()
    summary=json.loads((sink/'summary.json').read_text())
    assert summary['complete'] is not exhaust_budget and summary['records_written']==5
    assert [r['ordinal'] for r in summary['write_timings']]==list(range(5))
