"""Explicit profile validation and read-only controller parameter preflight."""
import copy
import json
from pathlib import Path
import time

ROLES = ('BeforeCostAccumulatorCritic', 'AfterCostAccumulatorCritic')
ORIGINAL_ORDER = ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
    'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic', 'DynamicObstacleCritic',
    'StaticStoppingCritic', 'NativeCycleSnapshotCritic']
ORDER = ORIGINAL_ORDER[:1]+[ROLES[0], 'CostCritic', ROLES[1]]+ORIGINAL_ORDER[2:]


def configure_profile(profile, baseline, output):
    clean = copy.deepcopy(profile); follow = clean['controller_server']['ros__parameters']['FollowPath']
    if follow['critics'] != ORDER: raise ValueError('registered accumulator critic order required')
    for role in ROLES:
        expected = {'enabled': True, 'capture_enabled': False, 'output_directory': '', 'max_records': 1000}
        if json.dumps(follow.pop(role), sort_keys=True) != json.dumps(expected, sort_keys=True):
            raise ValueError('registered default-off accumulator parameters required')
    follow['critics'] = ORIGINAL_ORDER
    if json.dumps(clean, sort_keys=True) != json.dumps(baseline, sort_keys=True):
        raise ValueError('accumulator profile differs from complete registered baseline')
    actual = profile['controller_server']['ros__parameters']['FollowPath']
    for role, leaf in zip(ROLES, ('cost_before', 'cost_after')):
        directory = (Path(output)/leaf).resolve()
        if directory.exists(): raise ValueError('new exclusive cost evidence directory required')
        actual[role]['capture_enabled'] = True
        actual[role]['output_directory'] = str(directory)
    return actual


def expected_parameters(controller):
    """Read back all explicit FollowPath scalars and critic settings, not only IO."""
    result = {'controller_frequency': controller['controller_frequency'], 'odom_topic': controller['odom_topic']}
    def flatten(prefix, value):
        if isinstance(value, dict):
            for name, child in value.items(): flatten(prefix+'.'+name, child)
        else: result[prefix] = value
    flatten('FollowPath', controller['FollowPath'])
    return result


def configured_value(value):
    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value', 9: 'string_array_value'}
    if value.type not in fields: raise RuntimeError('unexpected configured accumulator parameter type')
    result = getattr(value, fields[value.type])
    return list(result) if value.type == 9 else result


def expected_type(value):
    if type(value) is bool: return 1
    if type(value) is int: return 2
    if type(value) is float: return 3
    if type(value) is str: return 4
    if isinstance(value, list) and all(type(v) is str for v in value): return 9
    raise ValueError('unsupported accumulator preflight expectation')


class CostAccumulatorPreflight:
    def __init__(self, node, controller, timeout=10.):
        from rcl_interfaces.srv import GetParameters
        self.service_type = GetParameters
        self.client = node.create_client(GetParameters, '/controller_server/get_parameters')
        self.expected = expected_parameters(controller)
        self.names = sorted(self.expected); self.future = None; self.deadline = time.monotonic()+timeout
        self.evidence = {'requested': self.expected, 'names': self.names}

    def poll(self):
        if time.monotonic() > self.deadline: raise RuntimeError('cost accumulator readback timeout')
        if self.future is None:
            if not self.client.service_is_ready(): return None
            request = self.service_type.Request(); request.names = self.names
            self.future = self.client.call_async(request)
        if not self.future.done(): return None
        values = self.future.result().values
        if len(values) != len(self.names): raise RuntimeError('cost accumulator readback count mismatch')
        configured = {n: configured_value(v) for n, v in zip(self.names, values)}
        self.evidence['configured'] = configured
        if any(v.type != expected_type(self.expected[n]) or configured[n] != self.expected[n]
                for n, v in zip(self.names, values)):
            raise RuntimeError('cost accumulator configured parameter mismatch')
        return {'verdict': 'CONFIGURED PARAMETER PASS', **self.evidence,
            'scope': 'All explicit FollowPath settings and controller frequency/odom readback. Actual cost pairs, timing and physical/task gates remain independent.'}
