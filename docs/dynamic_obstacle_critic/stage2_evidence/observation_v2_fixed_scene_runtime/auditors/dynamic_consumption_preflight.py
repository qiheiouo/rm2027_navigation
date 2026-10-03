"""Read only configured evidence parameters before the isolated trial's goal."""
import time


class DynamicConsumptionPreflight:
    def __init__(self, node, directory, maximum, timeout=10.):
        from rcl_interfaces.srv import GetParameters
        self.service_type = GetParameters
        self.client = node.create_client(GetParameters, '/controller_server/get_parameters')
        self.directory = directory; self.maximum = maximum; self.deadline = time.monotonic()+timeout
        self.future = None
        self.names = ['FollowPath.DynamicObstacleCritic.'+n for n in
            ('consumption_evidence_directory', 'consumption_evidence_max_records', 'enabled')]
        self.evidence = {'requested_directory': directory, 'requested_maximum': maximum, 'names': self.names}

    def poll(self):
        if time.monotonic() > self.deadline: raise RuntimeError('dynamic evidence readback timeout')
        if self.future is None:
            if not self.client.service_is_ready(): return None
            request = self.service_type.Request(); request.names = self.names
            self.future = self.client.call_async(request)
        if not self.future.done(): return None
        values = self.future.result().values
        if len(values) != 3 or [v.type for v in values] != [4, 2, 1]:
            raise RuntimeError('dynamic evidence parameters not configured with expected types')
        configured = [values[0].string_value, values[1].integer_value, values[2].bool_value]
        self.evidence['configured'] = configured
        if configured != [self.directory, self.maximum, True]:
            raise RuntimeError('configured dynamic evidence parameter mismatch')
        return {'verdict': 'CONFIGURED PARAMETER PASS', **self.evidence,
            'scope': 'Readback only. Exact cost replay, native join and physical gates remain independent.'}
