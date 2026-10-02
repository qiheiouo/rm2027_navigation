#!/usr/bin/env python3
"""Keep native numerical context witnesses separate from physical input validity."""
import argparse
import json
from pathlib import Path


def assess(full,selected,velocity,controller):
    full=json.loads(full.read_text());selected=json.loads(selected.read_text())
    velocity=json.loads(velocity.read_text());controller=json.loads(controller.read_text())
    rows=selected['records'][0]['proposals'];sampler=rows[11:]
    route=controller['parameters']['odom_topic'][0]['string'] if controller['configured'] else None
    return {'verdict':'PHYSICAL SAFE CONTROL WITNESS NOT ESTABLISHED',
            'scope':'model/SG reconstruction is exact; source measurement validity is a separate required gate',
            'native_numerical_cycles':full['cycles'],'velocity_contract':velocity['verdict'],
            'all_native_inputs_zero':velocity['all_native_inputs_zero'],
            'moving_canonical_source_cycles':velocity['canonical_motion_above_threshold_cycles'],
            'configured_controller_odom_topic':route,'configured_controller_subscriptions':controller['subscriptions'],
            'native_input_context_fixed_counts':full['counts'],
            'selected_source_ordinal':selected['records'][0]['source_ordinal'],
            'selected_native_input_context_sampler_safe_progress':sum(r['conditional_safe_control_with_progress'] for r in sampler),
            'sampler_rows':len(sampler),
            'sampler_conclusion':'This recorded zero-native-input context contains safe individual bounded/SG proposals; it is not evidence that all 300 lack coverage. Valid physically measured-speed coverage remains unproved.',
            'next_required_change':'isolate controller_server.ros__parameters.odom_topic=/odometry/lio, verify configured parameter/subscription and actual nonzero native input during canonical motion, then repeat full strict trial and native witnesses',
            'original_physical_trial':'FAILED','main_feature_or_hardware_acceptance':False}


def main():
    parser=argparse.ArgumentParser()
    for name in ['full','selected','velocity','controller','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args();result=assess(args.full,args.selected,args.velocity,args.controller)
    with args.output.open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='native_input_context_fixed_counts'}))


if __name__=='__main__':main()
