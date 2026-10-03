"""Registered v2 profile selection and read-only three-node parameter gate.

This is experiment tooling. It neither publishes geometry nor grants a future
support certificate to the fixed nominal diameter.
"""
import copy
import json
import math
import time
from cost_accumulator_preflight import expected_parameters

SCHEMA = 'rm_dynamic_obstacle_predictions/v2_observation_anchor'
NOMINAL_DIAMETER = 1.6970562748477143  # upward from sqrt(1.2**2 + 1.2**2)
PRIOR_SOURCE = ('RoboMaster2026/V2.0.0/20260626;sha256='
    '77010458d06616b2bafad4400aad384f72e1b7007269c58061384de8fd0fb41b;'
    'S94,S96,S98,S113;listed_ground_classes;upright_projection;'
    'payload,arbitrary_tilt,2027_applicability_unverified;nominal_CV_only')


def radius_parameters(mode):
    if mode not in ('visible_extent', 'nominal_diameter'):
        raise ValueError('registered observation radius mode required')
    return dict(input_schema=SCHEMA, obstacle_radius_mode=mode,
                nominal_obstacle_diameter=NOMINAL_DIAMETER if mode == 'nominal_diameter' else 0.,
                geometry_prior_source=PRIOR_SOURCE if mode == 'nominal_diameter' else '')


def transformed_controller(baseline, mode):
    result = copy.deepcopy(baseline)
    result['controller_server']['ros__parameters']['FollowPath']['DynamicObstacleCritic'].update(radius_parameters(mode))
    return result


def registered_profiles(controller, tracker, guard, baselines, mode):
    """Require the complete fixed scene profiles, with only preregistered edits."""
    expected_controller = transformed_controller(baselines['controller'], mode)
    expected_tracker = copy.deepcopy(baselines['tracker'])
    expected_tracker['dynamic_obstacle_tracker_shadow']['ros__parameters']['prediction'].update(
        anchor_mode='last_observation_cv', max_speed=0.0)
    expected_guard = copy.deepcopy(baselines['guard'])
    expected_guard['dynamic_safety_guard']['ros__parameters'].update(radius_parameters(mode))
    for name, actual, expected in (
            ('controller', controller, expected_controller), ('tracker', tracker, expected_tracker),
            ('guard', guard, expected_guard)):
        # JSON comparison retains bool/int/float distinctions, unlike dict ==.
        if json.dumps(actual, sort_keys=True, allow_nan=False) != json.dumps(expected, sort_keys=True, allow_nan=False):
            raise ValueError('observation '+name+' differs from complete registered profile')
    return dict(schema=1, input_schema=SCHEMA, radius_parameters=radius_parameters(mode),
                scope='Fixed nominal CV comparison; physical e/B, association and future support remain unverified.')


def flatten_parameters(value, prefix=''):
    result = {}
    for name, child in value.items():
        key = prefix+'.'+name if prefix else name
        if isinstance(child, dict): result.update(flatten_parameters(child, key))
        else: result[key] = child
    return result


def typed_value(value):
    if type(value) is bool: return 1, value
    if type(value) is int: return 2, value
    if type(value) is float and math.isfinite(value): return 3, value
    if type(value) is str: return 4, value
    if isinstance(value, list) and value and all(type(v) is float and math.isfinite(v) for v in value): return 8, value
    if isinstance(value, list) and all(type(v) is str for v in value): return 9, value
    raise ValueError('unsupported observation parameter expectation')


def decoded_parameter(value):
    fields = {1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value',
              8:'double_array_value', 9:'string_array_value'}
    if value.type not in fields: raise RuntimeError('unexpected observation parameter type')
    result = getattr(value, fields[value.type])
    return list(result) if value.type in (8, 9) else result


class ObservationProfilePreflight:
    def __init__(self, node, controller, tracker, guard, use_sim_time=True,
                 guard_output='/cmd_vel', timeout=10.):
        from rcl_interfaces.srv import GetParameters
        self.service_type = GetParameters
        expected = {
            'controller_server': {**expected_parameters(controller), 'use_sim_time':use_sim_time},
            'dynamic_obstacle_tracker_shadow': {**flatten_parameters(tracker), 'use_sim_time':use_sim_time},
            'dynamic_safety_guard': {**flatten_parameters(guard), 'use_sim_time':use_sim_time, 'output_topic':guard_output}}
        self.deadline = time.monotonic()+timeout
        self.evidence = {'requested':expected, 'configured':{}, 'configured_types':{}}
        self.clients = {name:node.create_client(GetParameters, '/'+name+'/get_parameters') for name in expected}
        self.futures = {}; self.names = {name:sorted(values) for name, values in expected.items()}

    def poll(self):
        if time.monotonic() > self.deadline: raise RuntimeError('observation three-node readback timeout')
        for name, client in self.clients.items():
            if name in self.evidence['configured']: continue
            if name not in self.futures:
                if not client.service_is_ready(): continue
                self.futures[name] = client.call_async(self.service_type.Request(names=self.names[name]))
            future = self.futures[name]
            if not future.done(): continue
            response = future.result()
            if response is None or len(response.values) != len(self.names[name]):
                raise RuntimeError('observation readback count mismatch: '+name)
            actual = {key:decoded_parameter(value) for key, value in zip(self.names[name], response.values)}
            types = {key:value.type for key, value in zip(self.names[name], response.values)}
            self.evidence['configured'][name] = actual; self.evidence['configured_types'][name] = types
            for key, expected in self.evidence['requested'][name].items():
                required_type, required_value = typed_value(expected)
                if types[key] != required_type or actual[key] != required_value:
                    raise RuntimeError('observation configured parameter mismatch: '+name+'.'+key)
        if len(self.evidence['configured']) != len(self.clients): return None
        return {'verdict':'CONFIGURED PARAMETER PASS', **self.evidence,
                'scope':'All selected tracker/guard fields and controller FollowPath/frequency/odom. Receipt, consumed data and physical/task gates remain independent.'}


def receipt_evidence(message, now_ns):
    """Fresh selected public v2 wire receipt required at the goal decision."""
    source_ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
    if (message.schema != SCHEMA or message.authority != message.AUTHORITY_SHADOW_ONLY or
            message.header.frame_id != 'map' or not message.complete or message.prediction_steps != 30 or
            message.prediction_dt != .1 or message.total_track_count != len(message.tracks) or
            now_ns-source_ns > 400_000_000 or source_ns-now_ns > 50_000_000):
        raise RuntimeError('observation public receipt schema/grid/clock mismatch')
    return dict(schema=message.schema, authority=message.authority, frame=message.header.frame_id,
                source_stamp_ns=source_ns, receive_clock_ns=now_ns, tracks=len(message.tracks),
                complete=message.complete, prediction_steps=message.prediction_steps,
                prediction_dt=message.prediction_dt)
