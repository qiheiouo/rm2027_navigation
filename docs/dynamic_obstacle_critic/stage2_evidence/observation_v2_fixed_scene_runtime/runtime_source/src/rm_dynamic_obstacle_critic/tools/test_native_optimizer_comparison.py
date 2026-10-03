"""Independent comparison must reject malformed evidence and retain exact-bit gates."""
import json
import math
from pathlib import Path
import pytest
from compare_native_optimizer_replay import compare


def fixture(tmp_path):
    plan={'records':[{'ordinal':0,'actual_command':[0.1,0.,-0.2]}],
          'reset_ordinals':[0],'excluded_zero_command_ordinals':[1]}
    records=[{'kind':'native_library','path':'/opt/ros/humble/lib/libmppi_controller.so'},
             {'ordinal':0,'returned_control':[0.1,0.,-0.2],
              'fixed_noise_input_exact':True,'fixed_noise_input_max_error':0.,
              'reset_from_recorded_events':True}]
    schedule=tmp_path/'schedule.json';output=tmp_path/'output.jsonl'
    schedule.write_text(json.dumps(plan))
    return schedule,output,records


def run(schedule,output,records):
    output.write_text(''.join(json.dumps(r)+'\n' for r in records))
    return compare(schedule,output)


def test_exact_and_one_ulp(tmp_path):
    schedule,output,records=fixture(tmp_path)
    assert run(schedule,output,records)['verdict']=='RECONSTRUCTION EXACT'
    records[1]['returned_control'][0]=math.nextafter(.1,math.inf)
    assert run(schedule,output,records)['verdict']=='FAILED'


def test_inexact_inputs_fail_with_exact_output(tmp_path):
    schedule,output,records=fixture(tmp_path)
    records[1]['fixed_noise_input_exact']=False
    assert run(schedule,output,records)['verdict']=='FAILED'


@pytest.mark.parametrize('change', ['short','nan','ordinal','type','count'])
def test_malformed_evidence_rejected(tmp_path,change):
    schedule,output,records=fixture(tmp_path)
    if change=='short':records[1]['returned_control'].pop()
    elif change=='nan':records[1]['returned_control'][0]=math.nan
    elif change=='ordinal':records[1]['ordinal']=1
    elif change=='type':records[1]['fixed_noise_input_exact']='true'
    else:records.pop()
    with pytest.raises(ValueError):run(schedule,output,records)
