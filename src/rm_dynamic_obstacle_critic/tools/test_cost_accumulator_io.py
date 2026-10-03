#!/usr/bin/env python3
"""Probe actual serializer/plugin files; retain every negative fixture.

Synthetic cost changes below test association only, not actual SDK scoring.
"""
import argparse
import copy
import json
from pathlib import Path
import struct
from types import SimpleNamespace
import sys
import yaml
from cost_accumulator_io import read_observation, pair_unknown_reason, input_bytes, COST
from cost_accumulator_preflight import configure_profile, expected_parameters, CostAccumulatorPreflight, expected_type


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('plugin_fixtures', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    source = args.plugin_fixtures/'ActiveCallbacksShareCommittedTokenAndLeaveCostsUnchanged_before/cost_0.json'
    before = read_observation(source)
    after = read_observation(args.plugin_fixtures/'ActiveCallbacksShareCommittedTokenAndLeaveCostsUnchanged_after/cost_0.json')
    results = []
    def passed(name): results.append({'name': name, 'verdict': 'PASS'})
    assert pair_unknown_reason(before, after) is None
    assert list(before[1][COST]['values']) == [.25, 8.] and before[1][COST]['bytes'] == after[1][COST]['bytes']
    passed('actual read-only plugin pair and copied costs')
    for leaf in ('AfterWithoutBeforeIsExplicitlyUnpaired', 'BudgetExhaustionCannotReuseOrphanedBeforeToken', 'DifferentCriticDataObjectCannotReuseToken'):
        observed = read_observation(args.plugin_fixtures/(leaf+'_after')/'cost_0.json')
        assert observed[0]['paired_before_ordinal'] is None and pair_unknown_reason(before, observed) is not None
        passed('actual unpaired '+leaf)
    meta = copy.deepcopy(after[0]); blob = (source.with_suffix('.bin')).read_bytes()
    def fixture(name, change=None, binary=None):
        directory = args.output/name; directory.mkdir(); value = copy.deepcopy(meta)
        if change: change(value)
        path = directory/'cost_0.json'; path.write_text(json.dumps(value, allow_nan=False)+'\n')
        path.with_suffix('.bin').write_bytes(blob if binary is None else binary)
        return path
    invalid = {
        'wrong_schema': lambda m: m.update(schema=1),
        'bool_ordinal': lambda m: m.update(ordinal=True),
        'bool_token': lambda m: m.update(paired_before_ordinal=True),
        'bad_stage': lambda m: m.update(stage='CostCritic_private_branch'),
        'frame_mismatch': lambda m: m.update(pose_frame='map'),
        'bad_stamp': lambda m: m.update(pose_stamp_nanosec=10**9),
        'bool_fail_flag': lambda m: m.update(fail_flag=0),
        'bool_lock': lambda m: m.update(map_snapshot_mutex_locked=1),
        'zero_dt': lambda m: m.update(model_dt=0),
        'huge_integer_dt': lambda m: m.update(model_dt=10**400),
        'wrong_pose_count': lambda m: m['pose'].append(0.),
        'oversized_map': lambda m: m['map'].update(size=[65536,2]),
        'zero_resolution': lambda m: m['map'].update(resolution=0),
        'extra_metadata': lambda m: m.update(private_cost_increment=0),
        'unknown_scope': lambda m: m.update(unavailable=[]),
        'duplicate_block': lambda m: m['blocks'][1].update(name='x'),
        'unknown_block': lambda m: m['blocks'][1].update(name='private_selection'),
        'overlap': lambda m: m['blocks'][1].update(offset=0),
        'bool_offset': lambda m: m['blocks'][0].update(offset=False),
        'bad_dtype': lambda m: m['blocks'][0].update(dtype='>f4'),
        'bad_shape': lambda m: m['blocks'][0].update(shape=[1,60]),
        'negative_dimension': lambda m: m['blocks'][0].update(shape=[2,-30]),
        'path_budget': lambda m: m['blocks'][10].update(shape=[2001]),
        'payload_budget': lambda m: m.update(payload_bytes=2_000_001),
    }
    for name, change in invalid.items():
        path = fixture(name, change)
        try: read_observation(path)
        except ValueError: passed('reader rejects '+name)
        else: raise AssertionError(name)
    for name, data in [('truncated', blob[:-1]), ('trailing', blob+b'\x00'), ('nan_tensor', struct.pack('<f',float('nan'))+blob[4:])]:
        path = fixture(name, binary=data)
        try: read_observation(path)
        except ValueError: passed('reader rejects '+name)
        else: raise AssertionError(name)
    path = fixture('duplicate_json_key'); path.write_text(path.read_text().replace('"ordinal": 0', '"ordinal": 0, "ordinal": 0'))
    try: read_observation(path)
    except ValueError: passed('reader rejects duplicate_json_key')
    else: raise AssertionError('duplicate_json_key')
    mismatches = {
        'missing_token': lambda m: m.update(paired_before_ordinal=None),
        'wrong_token': lambda m: m.update(paired_before_ordinal=1),
        'failed_batch': lambda m: m.update(fail_flag=True),
        'unlocked_map': lambda m: m.update(map_snapshot_mutex_locked=False),
        'earlier_capture': lambda m: m.update(capture_stamp=before[0]['capture_stamp']-1),
        'signed_zero_pose': lambda m: m['pose'].__setitem__(0,-0.),
        'different_source_stamp': lambda m: m.update(pose_stamp_nanosec=1),
        'different_map_origin': lambda m: m['map']['origin'].__setitem__(0,m['map']['origin'][0]+.05),
    }
    for name, change in mismatches.items():
        assert pair_unknown_reason(before,read_observation(fixture(name,change))) is not None
        passed('pair remains unknown '+name)
    for name, block in [('control_ulp','cvx'),('path_ulp','path_x'),('map_byte','raw_costmap')]:
        mutated = bytearray(blob); offset = next(b['offset'] for b in meta['blocks'] if b['name']==block); mutated[offset] ^= 1
        assert pair_unknown_reason(before,read_observation(fixture(name,binary=mutated))) is not None
        passed('pair remains unknown '+name)
    synthetic = bytearray(blob); offset = next(b['offset'] for b in meta['blocks'] if b['name']==COST)
    synthetic[offset:offset+4] = struct.pack('<f',1.25)
    observed = read_observation(fixture('synthetic_cost_change',binary=synthetic))
    assert pair_unknown_reason(before,observed) is None and input_bytes(*before)==input_bytes(*observed)
    assert observed[1][COST]['values'][0]-before[1][COST]['values'][0]==1.
    passed('synthetic delta excludes costs from input identity; no scoring claim')
    config = Path(__file__).resolve().parents[1]/'config'
    profile = yaml.safe_load((config/'nav2_cv_cost_accumulator_evidence.yaml').read_text())
    baseline = yaml.safe_load((config/'nav2_cv_dynamic_consumption.yaml').read_text())
    configure_profile(profile,baseline,args.output/'profile_test'); passed('complete typed baseline profile identity')
    for name, change in [('cost_weight',lambda p:p['controller_server']['ros__parameters']['FollowPath']['CostCritic'].update(cost_weight=1.)),
                         ('outside_controller',lambda p:p['bt_navigator']['ros__parameters'].update(odom_topic='/wrong')),
                         ('numeric_type',lambda p:p['controller_server']['ros__parameters']['FollowPath'].update(iteration_count=1.0))]:
        p=yaml.safe_load((config/'nav2_cv_cost_accumulator_evidence.yaml').read_text());change(p)
        try:configure_profile(p,baseline,args.output/name)
        except ValueError:passed('profile rejects '+name)
        else:raise AssertionError(name)
    class Request: pass
    sys.modules['rcl_interfaces.srv']=SimpleNamespace(GetParameters=SimpleNamespace(Request=Request))
    controller=profile['controller_server']['ros__parameters']; expected=expected_parameters(controller)
    def values():
        result=[]
        for n in sorted(expected):
            value=expected[n];kind=expected_type(value)
            result.append(SimpleNamespace(type=kind,**{{1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',9:'string_array_value'}[kind]:value}))
        return result
    class Future:
        def done(self):return True
        def result(self):return SimpleNamespace(values=self.values)
    class Client:
        def service_is_ready(self):return True
        def call_async(self,request):assert request.names==sorted(expected);return self.future
    class Node:
        def create_client(self,*args):return self.client
    def check(value):
        future=Future();future.values=value;client=Client();client.future=future;node=Node();node.client=client
        return CostAccumulatorPreflight(node,controller)
    assert check(values()).poll()['verdict']=='CONFIGURED PARAMETER PASS';passed('complete preflight protocol mock')
    for name, bad in [('missing',values()[:-1]),('wrong_type',values()),('wrong_value',values())]:
        if name=='wrong_type':bad[0].type=0
        if name=='wrong_value':
            index=sorted(expected).index('FollowPath.CostCritic.cost_weight');bad[index].double_value=0.
        try:check(bad).poll()
        except RuntimeError:passed('preflight rejects '+name)
        else:raise AssertionError(name)
    (args.output/'result.json').write_text(json.dumps({'verdict':'PASS','checks':len(results),'results':results,
        'limits':'Protocol mocks are not configured controller evidence. Synthetic cost changes are not actual CostCritic scoring.'},indent=2)+'\n')
    print(f'{len(results)} strict reader/association/profile/protocol checks PASS')


if __name__=='__main__':main()
