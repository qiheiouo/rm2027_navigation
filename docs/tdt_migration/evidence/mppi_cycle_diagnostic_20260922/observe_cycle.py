"""Reuse frozen observer; add passive command/footprint/receipt capture."""
from pathlib import Path
import json,sys,time
sys.path.insert(0,'/ws/docs/tdt_migration/evidence/dynamic_reference_20260922')
from observe_dynamic import DynamicObserver, original
from geometry_msgs.msg import Twist, PolygonStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
import yaml

TOPICS={'/cmd_vel_nav':Twist,'/cmd_vel':Twist,'/simulation/chassis/cmd_vel':Twist,
        '/simulation/ground_truth/odom':Odometry,
        '/local_costmap/published_footprint':PolygonStamped,'/global_costmap/published_footprint':PolygonStamped,
        '/local_costmap/costmap_raw':Costmap}

def info_row(info):
    # rclpy Humble's executor passes only msg to Python subscription callbacks.
    # Preserve missing metadata explicitly; the graph snapshot is not a per-message GID.
    return {'publisher_gid':None if info is None else bytes(info.publisher_gid).hex(),
            'source_system_ns':None if info is None else int(info.source_timestamp),
            'received_system_ns':None if info is None else int(info.received_timestamp),
            'callback_steady_ns':time.monotonic_ns()}

class CycleObserver(DynamicObserver):
    def __init__(self,output,launch_log):
        super().__init__(output,launch_log)
        self.output=output;self.geometry_requests=None;self.geometry_verified=False
        for name in ('command_chain','footprints','raw_local_maps','publisher_graph','receipt_times'):
            self.streams[name]=(output/f'{name}.jsonl').open('x',buffering=1)
        for topic,kind in TOPICS.items():
            qos=QoSProfile(depth=100) if kind==Twist else qos_profile_sensor_data
            if kind==Costmap:qos=QoSProfile(depth=4,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.refs.append(self.create_subscription(kind,topic,self.make_cb(topic,kind),qos))
        self.refs.append(self.create_timer(1.,self.graph_cb))
    def make_cb(self,topic,kind):
        def callback(msg,info=None):
            record={'topic':topic,'t':self.now_sim,**info_row(info)}
            if hasattr(msg,'header'):record.update(stamp=original.seconds(msg.header.stamp),frame=msg.header.frame_id)
            if kind in (Twist,Odometry):
                twist=msg if kind==Twist else msg.twist.twist
                record.update(vx=twist.linear.x,vy=twist.linear.y,wz=twist.angular.z)
                self.write('command_chain',record)
            elif kind==PolygonStamped:
                record['points']=[[p.x,p.y] for p in msg.polygon.points];self.write('footprints',record)
            else:
                m=msg.metadata
                record.update(width=m.size_x,height=m.size_y,resolution=m.resolution,origin=[m.origin.position.x,m.origin.position.y],
                              data=[v[0] if isinstance(v,bytes) else int(v) for v in msg.data])
                self.write('raw_local_maps',record)
        return callback
    def graph_cb(self):
        records=[]
        for topic in TOPICS:
            for ep in self.get_publishers_info_by_topic(topic):
                records.append({'topic':topic,'node':ep.node_name,'namespace':ep.node_namespace,'gid':bytes(ep.endpoint_gid).hex()})
        self.write('publisher_graph',{'t':self.now_sim,'callback_steady_ns':time.monotonic_ns(),'publishers':records})
    def scan_cb(self,msg):
        super().scan_cb(msg)
        self.write('receipt_times',{'topic':'/scan','stamp':original.seconds(msg.header.stamp),'t':self.now_sim,'callback_steady_ns':time.monotonic_ns()})
    def ready(self):
        # Start independent service reads before choosing the fixed 8-second phase.
        if not self.geometry_verified:
            if self.now_sim<6:return False
            if self.geometry_requests is None:
                if not self.profile_to_verify:raise RuntimeError('profile required')
                self.expected_profile=yaml.safe_load(self.profile_to_verify.read_text());self.geometry_requests=[]
                for name in ('local_costmap','global_costmap'):
                    client=self.create_client(GetParameters,f'/{name}/{name}/get_parameters')
                    self.geometry_requests.append((name,client,None))
                self.geometry_deadline=time.monotonic()+20
            updated=[]
            for name,client,future in self.geometry_requests:
                if future is None and client.service_is_ready():future=client.call_async(GetParameters.Request(names=['footprint','footprint_padding']))
                updated.append((name,client,future))
            self.geometry_requests=updated
            if not all(f is not None and f.done() for _,_,f in updated):
                if time.monotonic()>self.geometry_deadline:raise RuntimeError('preflight footprint service timeout')
                return False
            report={'scope':'before preflight; fixed runtime footprint verified against unchanged profile','t':self.now_sim,'runtime':{}}
            for name,client,future in updated:
                actual=dict(zip(['footprint','footprint_padding'],[parameter_value_to_python(v) for v in future.result().values]))
                expected={k:self.expected_profile[name][name]['ros__parameters'][k] for k in actual}
                if actual!=expected:raise RuntimeError('runtime footprint mismatch '+name)
                report['runtime'][name]={'actual':actual,'expected':expected};self.destroy_client(client)
            with (self.output.parent/'runtime_geometry_preflight.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
            self.geometry_verified=True
        return super().ready()

original.Observer=CycleObserver
if __name__=='__main__':sys.exit(original.main())
