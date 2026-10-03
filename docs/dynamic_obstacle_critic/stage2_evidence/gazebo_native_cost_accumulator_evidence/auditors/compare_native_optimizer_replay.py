#!/usr/bin/env python3
"""Exact native input and source-identified output check; never substitute tolerance."""
import argparse
import gzip
import json
import math
from pathlib import Path
import struct


def compare(schedule,output):
    content=gzip.open(output,'rt').read() if output.suffix=='.gz' else output.read_text()
    plan=json.loads(schedule.read_text());records=[json.loads(line) for line in content.splitlines()]
    if not records or not plan['records']:raise ValueError('native replay empty evidence')
    library=records.pop(0)
    if library.get('kind')!='native_library' or len(records)!=len(plan['records']):
        raise ValueError('native replay library/count')
    matches=[];maximum=0;checked=[]
    for replay,actual in zip(records,plan['records']):
        if replay['ordinal']!=actual['ordinal']:raise ValueError('native replay ordinal')
        for values in (replay['returned_control'],actual['actual_command']):
            if len(values)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
                raise ValueError('native replay command shape/finite')
        if not isinstance(replay['fixed_noise_input_exact'],bool) or not math.isfinite(replay['fixed_noise_input_max_error']):
            raise ValueError('native replay input evidence type/finite')
        exact=all(struct.pack('<d',a)==struct.pack('<d',b) for a,b in zip(replay['returned_control'],actual['actual_command']))
        matches.append(exact);maximum=max(maximum,max(abs(a-b) for a,b in zip(replay['returned_control'],actual['actual_command'])))
        checked.append({'ordinal':replay['ordinal'],'inputs_bit_exact':replay['fixed_noise_input_exact'],'output_double_bit_exact':exact})
    return {'verdict':'RECONSTRUCTION EXACT' if all(matches) and all(r['fixed_noise_input_exact'] for r in records) else 'FAILED',
        'scope':'numerically verified reconstruction under fixed noise/reset association; history is not a direct live snapshot or a safety/coverage certificate',
        'library':library,'cycles':len(records),'inputs_bit_exact':sum(r['fixed_noise_input_exact'] for r in records),
        'input_max_error':max(r['fixed_noise_input_max_error'] for r in records),
        'outputs_double_bit_exact':sum(matches),'output_max_error':maximum,'reset_ordinals':plan['reset_ordinals'],
        'executed_reset_ordinals':[r['ordinal'] for r in records if r['reset_from_recorded_events']],
        'excluded_zero_command_ordinals':plan['excluded_zero_command_ordinals'],'checks':checked}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('schedule',type=Path);parser.add_argument('output',type=Path);parser.add_argument('report',type=Path);args=parser.parse_args()
    result=compare(args.schedule,args.output);args.report.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='checks'}))


if __name__=='__main__':main()
