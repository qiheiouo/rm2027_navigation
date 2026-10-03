#!/usr/bin/env python3
"""Observe rounded CostCritic accumulator changes and strict native joins.

Missing or ambiguous association is UNKNOWN. This never selects a control or
replaces original safety, source-age, task or prediction-support gates.
"""
import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import struct
import yaml
from cost_accumulator_io import (read_observation, pair_unknown_reason, input_bytes,
    context_bytes, COST)
from cost_accumulator_preflight import ORDER, ROLES, configure_profile, expected_parameters
from native_snapshot_io import read_snapshot
from dynamic_consumption_io import read_consumption, cost_relation_exact
from observation_profile_preflight import (SCHEMA as OBSERVATION_SCHEMA, registered_profiles,
    transformed_controller, flatten_parameters, typed_value)


def dynamic_identity(meta, blocks):
    return context_bytes(meta)+b''.join(struct.pack('<2I',*blocks[n]['shape'])+blocks[n]['bytes'] for n in ('x','y','yaw'))


def record_set(directory, prefix, reader, stage=None):
    result={};previous=-float('inf')
    paths=sorted(directory.glob(prefix+'_*.json'),key=lambda p:int(p.stem.split('_')[-1]))
    if not 1<=len(paths)<=1000:raise ValueError('bounded nonempty evidence records required: '+str(directory))
    for path in paths:
        meta,blocks=reader(path);ordinal=meta['ordinal']
        if ordinal!=int(path.stem.split('_')[-1]) or ordinal in result or meta['capture_stamp']<previous:
            raise ValueError('record filename/ordinal/clock identity')
        if stage is not None and meta['stage']!=stage:raise ValueError('wrong stage in observer directory')
        previous=meta['capture_stamp'];result[ordinal]=(meta,blocks)
    return result


def index_records(records, identity):
    result={}
    for ordinal,(meta,blocks) in records.items():
        raw=identity(meta,blocks);key=hashlib.sha256(raw).hexdigest()
        result.setdefault(key,[]).append((ordinal,meta,blocks,raw))
    return result


def exact_matches(index, raw):
    return [entry for entry in index.get(hashlib.sha256(raw).hexdigest(),[]) if entry[3]==raw]


def audit(trial):
    profile=yaml.safe_load((trial/'profile.yaml').read_text())
    follow=profile['controller_server']['ros__parameters']['FollowPath']
    if follow['critics']!=ORDER:raise ValueError('exact registered twelve-critic order required')
    baseline=yaml.safe_load((trial/'installed_inputs/nav2_cv_dynamic_consumption.yaml').read_text())
    policy=json.loads((trial/'cost_accumulator_policy.json').read_text())
    baseline_hash=hashlib.sha256((trial/'installed_inputs/nav2_cv_dynamic_consumption.yaml').read_bytes()).hexdigest()
    if policy['baseline_identity']!=baseline_hash or policy['order']!=ORDER:raise ValueError('baseline policy identity')
    registered=json.loads(json.dumps(profile))
    registered_follow=registered['controller_server']['ros__parameters']['FollowPath']
    for role,leaf in zip(ROLES,('cost_before','cost_after')):
        settings=follow[role];directory=Path(settings['output_directory'])
        if (set(settings)!={'enabled','capture_enabled','output_directory','max_records'} or
                settings['enabled'] is not True or settings['capture_enabled'] is not True or
                type(settings['max_records']) is not int or settings['max_records']!=1000 or
                not directory.is_absolute() or str(directory)!=settings['output_directory'] or
                directory.name!=leaf or settings!=policy['parameters'][role]):
            raise ValueError('registered observer policy mismatch')
        registered_follow[role]={'enabled':True,'capture_enabled':False,'output_directory':'','max_records':1000}
    registered_follow['DynamicObstacleCritic']['consumption_evidence_directory']=''
    registered_follow['NativeCycleSnapshotCritic']['output_directory']=''
    observation = follow['DynamicObstacleCritic'].get('input_schema') == OBSERVATION_SCHEMA
    if observation:
        selection=json.loads((trial/'observation_profile_selection.json').read_text())
        mode=selection['radius_parameters']['obstacle_radius_mode']
        tracker=yaml.safe_load((trial/'installed_inputs/tracker_cv.yaml').read_text())
        guard=yaml.safe_load((trial/'installed_inputs/guard.yaml').read_text())
        selected=registered_profiles(registered,tracker,guard,{
            'controller':yaml.safe_load((trial/'installed_inputs/nav2_cv_cost_accumulator_evidence.yaml').read_text()),
            'tracker':yaml.safe_load((trial/'installed_inputs/tracker_cv_baseline.yaml').read_text()),
            'guard':yaml.safe_load((trial/'installed_inputs/guard_mechanical_footprint.yaml').read_text())},mode)
        if selection['radius_parameters']!=selected['radius_parameters'] or selection['input_schema']!=OBSERVATION_SCHEMA:
            raise ValueError('registered observation selection mismatch')
        baseline=transformed_controller(baseline,mode)
    configure_profile(registered,baseline,trial/'uncreated_validation_output')
    expected=expected_parameters(profile['controller_server']['ros__parameters'])
    if observation:
        readback=json.loads((trial/'observation_profile_readback.json').read_text())
        expected_nodes={
            'controller_server':{**expected,'use_sim_time':True},
            'dynamic_obstacle_tracker_shadow':{**flatten_parameters(tracker['dynamic_obstacle_tracker_shadow']['ros__parameters']),'use_sim_time':True},
            'dynamic_safety_guard':{**flatten_parameters(guard['dynamic_safety_guard']['ros__parameters']),'use_sim_time':True,'output_topic':'/cmd_vel'}}
        expected_types={name:{key:typed_value(value)[0] for key,value in fields.items()} for name,fields in expected_nodes.items()}
        if (readback['verdict']!='CONFIGURED PARAMETER PASS' or
                json.dumps(readback['requested'],sort_keys=True)!=json.dumps(expected_nodes,sort_keys=True) or
                json.dumps(readback['configured'],sort_keys=True)!=json.dumps(expected_nodes,sort_keys=True) or
                readback['configured_types']!=expected_types):
            raise ValueError('actual configured three-node observation preflight mismatch')
    else:
        readback=json.loads((trial/'cost_accumulator_readback.json').read_text())
        if (readback['verdict']!='CONFIGURED PARAMETER PASS' or
                json.dumps(readback['configured'],sort_keys=True)!=json.dumps(expected,sort_keys=True) or
                json.dumps(readback['requested'],sort_keys=True)!=json.dumps(expected,sort_keys=True)):
            raise ValueError('actual configured accumulator preflight mismatch')
    before=record_set(trial/'cost_before','cost',read_observation,'before_CostCritic')
    after=record_set(trial/'cost_after','cost',read_observation,'after_CostCritic')
    native=record_set(trial/'native_cycles','cycle',read_snapshot)
    # Dynamic records name the score clock, not the end-of-copy capture clock.
    dynamic={}
    for path in sorted((trial/'dynamic_scores').glob('score_*.json'),key=lambda p:int(p.stem.split('_')[-1])):
        meta,blocks=read_consumption(path);ordinal=meta['ordinal']
        if ordinal in dynamic or ordinal!=int(path.stem.split('_')[-1]) or not cost_relation_exact(blocks):
            raise ValueError('dynamic ordinal or exact float accumulation relation')
        dynamic[ordinal]=(meta,blocks)
    if not 1<=len(dynamic)<=1000:raise ValueError('dynamic evidence count')
    native_index=index_records(native,input_bytes);dynamic_index=index_records(dynamic,dynamic_identity)
    paired=[];unknown=[];claimed=set();joined_native=set();joined_dynamic=set()
    for ordinal,observed in after.items():
        ameta,ablocks=observed;token=ameta['paired_before_ordinal']
        if token is None or token not in before:
            unknown.append({'after_ordinal':ordinal,'before_ordinal':token,'reason':'no captured committed before token'});continue
        if token in claimed:raise ValueError('two after records claim one before token')
        claimed.add(token);bmeta,bblocks=before[token]
        reason=pair_unknown_reason(before[token],observed)
        if reason:
            unknown.append({'after_ordinal':ordinal,'before_ordinal':token,'reason':reason});continue
        delta=[Fraction(float(a))-Fraction(float(b)) for a,b in zip(ablocks[COST]['values'],bblocks[COST]['values'])]
        row={'before_ordinal':token,'after_ordinal':ordinal,'before_capture_stamp':bmeta['capture_stamp'],
            'after_capture_stamp':ameta['capture_stamp'],'pose_stamp_sec':bmeta['pose_stamp_sec'],
            'pose_stamp_nanosec':bmeta['pose_stamp_nanosec'],'input_identity_sha256':hashlib.sha256(input_bytes(bmeta,bblocks)).hexdigest(),
            'batch':len(delta),'rounded_accumulator_delta':[float(v) for v in delta],
            'exact_delta_rational':[[v.numerator,v.denominator] for v in delta],
            'before_cost_float_payload_sha256':hashlib.sha256(bblocks[COST]['bytes']).hexdigest(),
            'after_cost_float_payload_sha256':hashlib.sha256(ablocks[COST]['bytes']).hexdigest(),
            'zero_delta_rows':sum(v==0 for v in delta),'positive_delta_rows':sum(v>0 for v in delta),
            'negative_delta_rows':sum(v<0 for v in delta),'native_join':'UNKNOWN','dynamic_join':'UNKNOWN'}
        nm=exact_matches(native_index,input_bytes(bmeta,bblocks))
        dm=exact_matches(dynamic_index,dynamic_identity(bmeta,bblocks))
        if len(nm)==1 and not nm[0][1]['fail_flag'] and nm[0][0] not in joined_native:
            nordinal,nmeta,nblocks,_=nm[0]
            if nmeta['capture_stamp']>=ameta['capture_stamp']:
                row.update(native_join='EXACT PUBLIC INPUT BYTES',native_ordinal=nordinal);joined_native.add(nordinal)
        if len(dm)==1 and dm[0][0] not in joined_dynamic:
            dordinal,dmeta,dblocks,_=dm[0]
            if dmeta['score_stamp']>=ameta['capture_stamp']:
                row.update(dynamic_join='EXACT CONSUMED ROLLOUT/CONTEXT',dynamic_ordinal=dordinal)
                row['remaining_original_critic_rounded_delta']=[float(v)-float(c) for v,c in zip(dblocks['costs_before_dynamic']['values'],ablocks[COST]['values'])]
                joined_dynamic.add(dordinal)
        paired.append(row)
    unmatched_before=sorted(set(before)-claimed)
    partial=[]
    for leaf,prefix in [('cost_before','cost'),('cost_after','cost')]:
        for path in sorted((trial/leaf).iterdir()):
            if path.name.endswith('.tmp') or (path.name.endswith(('.bin','.bin.gz')) and
                    not (path.parent/(path.name.split('.bin')[0]+'.json')).is_file()):partial.append(str(path.relative_to(trial)))
    complete=(not unknown and not unmatched_before and not partial and len(paired)==len(before)==len(after) and
        len(joined_native)==len(native)==len(paired) and len(joined_dynamic)==len(dynamic)==len(paired))
    return {'schema':'native_cost_accumulator_audit_v1',
        'verdict':'EXACT OBSERVED ACCUMULATOR/NATIVE/CONSUMPTION JOIN PASS' if complete else 'INCOMPLETE; UNASSOCIATED CONTRIBUTIONS UNKNOWN',
        'before_records':len(before),'after_records':len(after),'valid_observed_pairs':len(paired),
        'native_records':len(native),'dynamic_records':len(dynamic),'joined_native':len(joined_native),'joined_dynamic':len(joined_dynamic),
        'unknown_after_records':unknown,'before_without_after':unmatched_before,'partial_files':partial,
        'pairs_without_native':[r['before_ordinal'] for r in paired if 'native_ordinal' not in r],
        'pairs_without_dynamic':[r['before_ordinal'] for r in paired if 'dynamic_ordinal' not in r],
        'unmatched_native':sorted(set(native)-joined_native),'unmatched_dynamic':sorted(set(dynamic)-joined_dynamic),
        'cycles':paired,'scope':'Actual adjacent f32 CostCritic accumulator changes with exact rational differences. No internal unrounded increment or branch attribution.',
        'limits':['Observer token is local committed pairing state, not an optimizer cycle id.',
            'Full copied public context/map match cannot reconstruct uncaptured SDK cache fields.',
            'Actual risk C++ replay and gamma/weights/SG/control reconstruction remain separate checks.',
            'Raw candidate cost labels are distinct from individual constrained/SG paths and actual aggregate output.',
            'Original physical/raw203/source-age/task gates and unknown real e/B are unchanged.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    report=audit(args.trial)
    with args.output.open('x') as stream:stream.write(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('verdict','before_records','after_records','valid_observed_pairs','joined_native','joined_dynamic')}))
