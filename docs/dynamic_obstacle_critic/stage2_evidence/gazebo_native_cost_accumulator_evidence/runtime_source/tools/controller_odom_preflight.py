"""Read-only public controller parameter/endpoint check after lifecycle configure."""
import time


def validate_route(topic,subscriptions,publishers,required):
    if topic!=required:raise ValueError('configured controller odom_topic mismatch: '+str(topic))
    if not any(name==required and 'nav_msgs/msg/Odometry' in types for name,types in subscriptions):
        raise ValueError('controller canonical Odometry subscription missing')
    if len(publishers)!=1 or publishers[0]['topic_type']!='nav_msgs/msg/Odometry':
        raise ValueError('canonical Odometry publisher count/type mismatch')
    return {'scope':'public readback after lifecycle configure; endpoint route only, not an effective-speed/freshness certificate',
            'required_topic':required,'configured_odom_topic':topic,'controller_subscriptions':subscriptions,
            'canonical_publishers':publishers,'verdict':'ROUTE PASS'}


class ControllerOdomPreflight:
    def __init__(self,node,required,timeout=10.):
        from rcl_interfaces.srv import GetParameters
        self.node=node;self.required=required;self.deadline=time.monotonic()+timeout
        self.service_type=GetParameters;self.client=node.create_client(GetParameters,'/controller_server/get_parameters')
        self.future=None;self.topic=None;self.finished=None
        self.evidence={'required_topic':required,'configured_odom_topic':None,'controller_subscriptions':[],
                       'canonical_publishers':[],'stage':'waiting for configured parameter'}

    def poll(self):
        if self.finished:return self.finished
        if time.monotonic()>self.deadline:raise RuntimeError('controller odom route preflight timeout')
        if self.future is None:
            if not self.client.service_is_ready():return None
            request=self.service_type.Request();request.names=['odom_topic'];self.future=self.client.call_async(request)
        if not self.future.done():return None
        values=self.future.result().values
        if len(values)!=1 or values[0].type!=4:
            raise RuntimeError('controller odom_topic not declared after explicit startup')
        self.topic=values[0].string_value
        subscriptions=self.node.get_subscriber_names_and_types_by_node('controller_server','/')
        publishers=[{'node_name':info.node_name,'namespace':info.node_namespace,'topic_type':info.topic_type}
                    for info in self.node.get_publishers_info_by_topic(self.required)]
        self.evidence.update(configured_odom_topic=self.topic,controller_subscriptions=subscriptions,
                             canonical_publishers=publishers,stage='configured parameter read')
        if self.topic!=self.required:raise RuntimeError('configured controller odom_topic mismatch: '+self.topic)
        # DDS graph discovery may lag a completed configure response. Wait for
        # endpoints; a wrong parameter already fails without sending a goal.
        if not publishers or not any(name==self.required for name,_ in subscriptions):return None
        self.finished=validate_route(self.topic,subscriptions,publishers,self.required);return self.finished
