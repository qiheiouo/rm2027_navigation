#!/usr/bin/env python3
"""Full actual risk/weights/control verification for the new accumulator trial."""
import argparse
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import struct
from dynamic_consumption_io import read_consumption
from cost_accumulator_io import read_observation, COST, pair_unknown_reason


def jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def verify(trial, optimizer, risks):
    observed=json.loads((trial/'cost_accumulator_audit.json').read_text())
    consumed=json.loads((trial/'dynamic_consumption_audit.json').read_text())
    comparison=json.loads((optimizer/'positive_comparison.json').read_text())
    baseline=(optimizer/'sdk_final_output.jsonl').read_bytes().splitlines(keepends=True)
    weights=(optimizer/'weights_output.jsonl').read_bytes().splitlines(keepends=True)
    uniform_lines=(optimizer/'uniform_output.jsonl').read_bytes().splitlines(keepends=True)
    uniform=[json.loads(line) for line in uniform_lines];uniform_header=uniform.pop(0)
    count=comparison['cycles']
    if (comparison['verdict']!='RECONSTRUCTION EXACT' or comparison['inputs_bit_exact']!=count or
            comparison['outputs_double_bit_exact']!=count or comparison['input_max_error']!=0 or comparison['output_max_error']!=0 or
            consumed['verdict']!='EXACT CONSUMPTION/NATIVE JOIN PASS' or consumed['exact_joined_batches']!=count or
            observed['joined_native']!=count or observed['joined_dynamic']!=count or
            len(weights)!=len(baseline) or len(weights)!=count+1 or weights[0]!=baseline[0] or len(uniform)!=count or
            uniform_header!=json.loads(baseline[0])):
        raise ValueError('complete actual native/consumption/weights source chain required')
    risk_records=jsonl(risks);replay={row['ordinal']:row for row in risk_records}
    if len(replay)!=count or len(risk_records)!=count:raise ValueError('dynamic C++ replay coverage')
    cost_rows={row['native_ordinal']:row for row in observed['cycles'] if 'native_ordinal' in row}
    dynamic_rows={row['native_ordinal']:row for row in consumed['cycles']}
    if set(cost_rows)!=set(range(count)) or set(dynamic_rows)!=set(range(count)):
        raise ValueError('complete source joins required')
    results=[];risk_count=0;uniform_exact=0;positive_cost_mass=[]
    marker=b',"weights_reaggregate_bounded_mean_bit_exact"'
    for ordinal,(line,reference) in enumerate(zip(weights[1:],baseline[1:])):
        row=json.loads(line);cost=cost_rows[ordinal];dynamic=dynamic_rows[ordinal]
        if (row['ordinal']!=ordinal or marker not in line or line.split(marker,1)[0]+b'}\n'!=reference or
                row['weight_probe']!='native' or row['weights_reaggregate_bounded_mean_bit_exact'] is not True or
                row['weights_reaggregate_max_error']!=0):raise ValueError('actual weights/SG chain mismatch')
        u=uniform[ordinal]
        uline=uniform_lines[ordinal+1]
        if (u['ordinal']!=ordinal or u['weight_probe']!='uniform_negative' or marker not in uline or
                uline.split(marker,1)[0]+b'}\n'!=reference):raise ValueError('uniform negative probe identity')
        uniform_exact+=u['weights_reaggregate_bounded_mean_bit_exact'] is True
        before=read_observation(trial/'cost_before'/f"cost_{cost['before_ordinal']}.json")
        after=read_observation(trial/'cost_after'/f"cost_{cost['after_ordinal']}.json")
        if pair_unknown_reason(before,after) is not None:raise ValueError('actual CostCritic pair failed')
        delta=[Fraction(float(a))-Fraction(float(b)) for a,b in zip(after[1][COST]['values'],before[1][COST]['values'])]
        if [[d.numerator,d.denominator] for d in delta]!=cost['exact_delta_rational']:
            raise ValueError('rounded CostCritic delta rational mismatch')
        meta,blocks=read_consumption(trial/'dynamic_scores'/f"score_{dynamic['score_ordinal']}.json")
        risk=replay[dynamic['score_ordinal']];batch,steps=blocks['x']['shape']
        if (risk['batch']!=batch or risk['steps']!=steps or risk['risk_double_bit_exact_rows']!=batch or
                risk['after_float_bit_exact_rows']!=batch or len(risk['risk'])!=batch or
                b''.join(struct.pack('<d',v) for v in risk['risk'])!=blocks['dynamic_risk_double']['bytes']):
            raise ValueError('actual consumed risk bytes differ from SDK-side replay')
        risk_count+=batch
        weights_float=row['native_softmax_weights'];gamma_cost=row['native_costs_after_regularization']
        if len(weights_float)!=batch or len(gamma_cost)!=batch or any(not math.isfinite(v) or v<0 for v in weights_float):
            raise ValueError('native weights budget/finite')
        if any(struct.unpack('<f',struct.pack('<f',v))[0]!=v for v in weights_float+gamma_cost):
            raise ValueError('native cost/weights must retain f32 values')
        rational=[Fraction(v) for v in weights_float];total=sum(rational,Fraction())
        if total<=0:raise ValueError('native weights have no positive mass')
        mass=sum((w for w,d in zip(rational,delta) if d>0),Fraction())
        zero=sum((w for w,d in zip(rational,delta) if d==0),Fraction())
        negative=sum((w for w,d in zip(rational,delta) if d<0),Fraction())
        if mass+zero+negative!=total:raise ValueError('exact disjoint weight partition failed')
        positive_cost_mass.append(float(mass/total))
        top=max(range(batch),key=weights_float.__getitem__)
        results.append({'native_ordinal':ordinal,'cost_before_ordinal':cost['before_ordinal'],
            'cost_after_ordinal':cost['after_ordinal'],'dynamic_ordinal':dynamic['score_ordinal'],
            'risk_rows_exact':batch,'rounded_cost_positive_rows':sum(d>0 for d in delta),
            'rounded_cost_zero_rows':sum(d==0 for d in delta),
            'positive_cost_weight_fraction':float(mass/total),
            'exact_weight_partition_total':[total.numerator,total.denominator],
            'exact_positive_cost_weight_mass':[mass.numerator,mass.denominator],
            'highest_weight_raw_row':top,'highest_weight':weights_float[top],
            'highest_weight_rounded_cost_delta':float(delta[top]),
            'highest_weight_actual_dynamic_risk':risk['risk'][top],
            'weights_SG_chain_byte_exact':True})
    return {'schema':'cost_accumulator_full_chain_v1','verdict':'EXACT OBSERVED COST/RISK/NATIVE WEIGHTS/SG CHAIN PASS',
        'cycles':count,'risk_double_and_after_float_exact_rows':risk_count,
        'actual_control_inputs_and_outputs_bit_exact':count,'weights_reaggregation_bit_exact':count,
        'uniform_negative_reaggregation_bit_exact':uniform_exact,
        'rounded_cost_pairs_without_final_output':[row['before_ordinal'] for row in observed['cycles'] if 'native_ordinal' not in row],
        'positive_rounded_cost_weight_fraction_range':[min(positive_cost_mass),max(positive_cost_mass)],
        'exact_weight_partitions':count,'records':results,
        'input_sha256':hashlib.sha256((optimizer/'input.bin').read_bytes()).hexdigest(),
        'scope':'Actual rounded CostCritic delta, exact consumed dynamic risk and numerically verified native weights/actual SG controls on all final-output cycles.',
        'limits':['Two-sided accumulator capture does not reveal unrounded CostCritic increment, private cache or actual query branch.',
            'Positive or zero rounded cost and native weight are not raw path or output safety labels.',
            'Extra observed cost pairs without final outputs are retained, not assigned a native control.',
            'Fresh physical/raw203/task FAILED and unknown real geometry/motion support remain separate.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('trial',type=Path);parser.add_argument('optimizer',type=Path)
    parser.add_argument('risks',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args()
    result=verify(args.trial,args.optimizer,args.risks)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='records'}))
