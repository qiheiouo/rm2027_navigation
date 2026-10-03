#!/usr/bin/env python3
"""Associate actual raw, bounded and SG SDK paths before fixture attribution."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import struct
import tempfile
from analyze_native_factor_aggregation import full_context
from analyze_trial import load_truth_rows
from audit_mechanical_footprint import components
from audit_raw_rollout_objective import geometry
from audit_scan_geometry import fixture
from native_individual_stage_io import read_stages
from native_snapshot_io import read_snapshot
from native_witness_io import FIELDS,records
from prepare_native_witness import prepare
from prepare_processed_cv import verify
from verify_native_factor_raw import verify_record

STAGES=('original_raw','bounded_before_SG','actual_history_after_SG')
EPOCHS=('actual_score_clock','source_pose_clock_sensitivity')


def context(source,witness,prepared,legacy_binary,legacy_json,stage_file):
    for root in [source,witness]:verify(root)
    with tempfile.TemporaryDirectory(prefix='individual_stage_input_') as temporary:
        regenerated=Path(temporary)/'prepared';identity=prepare(source,regenerated,[298])
        for name in ['input.bin','input_identity.json']:
            if (prepared/name).read_bytes()!=(regenerated/name).read_bytes():raise ValueError('individual stage source input changed')
    if legacy_binary.read_bytes()!=gzip.open(witness/'selected/witnesses.bin.gz','rb').read() or legacy_json.read_bytes()!=gzip.open(witness/'selected/witnesses.jsonl.gz','rb').read():raise ValueError('entire original311 witness and SDK JSON must remain byte exact')
    stage=read_stages(stage_file);native,blocks=read_snapshot(source/'native_cycles/cycle_298.json')
    old=list(records(witness/'selected/witnesses.bin.gz'))[0]
    if not all(old[k] for k in ['raw_velocity_pose_exact','aggregate_SG_exact','actual_command_double_bit_exact']):raise ValueError('original SDK proofs missing')
    for name in FIELDS:
        if stage['stages'][0]['blocks'][name]['bytes']!=blocks[name]['bytes']:raise ValueError('original raw stage tensor changed: '+name)
        if stage['stages'][2]['blocks'][name]['bytes']!=struct.pack('<9330f',*old['blocks'][name]):raise ValueError('original afterSG stage tensor changed: '+name)
    for s in stage['stages']:
        for name,axis in [('vx',0),('vy',1),('wz',5)]:
            expected=struct.pack('<f',native['speed'][axis])
            if any(struct.pack('<f',s['blocks'][name]['values'][i*30])!=expected for i in range(s['rows'])):raise ValueError('actual measured prefix changed')
    return identity,stage,native,blocks,old


def analyze(source,witness,raw_map,mass,prepared,legacy_binary,legacy_json,stage_file):
    source,witness,raw_map,mass=[p.resolve() for p in [source,witness,raw_map,mass]]
    for root in [raw_map,mass]:verify(root)
    identity,stage,native,blocks,old=context(source,witness,prepared,legacy_binary,legacy_json,stage_file)
    raw_labels=json.load(gzip.open(raw_map/'report.json.gz','rt'));after_labels=json.load(gzip.open(mass/'report.json.gz','rt'))
    truth=load_truth_rows(source,json.loads((source/'execution.json').read_text()));shapes=components(source/'scene_inputs');scene=fixture(source/'scene_inputs')
    policy=json.loads((source/'policy.json').read_text())
    if policy['raw_costmap_threshold']!=203 or policy['bounds_tolerance']!=5e-5:raise ValueError('original fixture gates required')
    epochs={e:raw_labels['epoch_summaries'][e]['epoch'] for e in EPOCHS};labels={};summaries={};rows=[]
    legacy_labels=json.load(gzip.open(witness/'selected/analysis.json.gz','rt'))['records'][0]['proposals']
    for name,s in zip(STAGES,stage['stages']):
        record={'rows':s['rows'],'model_dt':stage['model_dt'],'initial_yaw':old['initial_yaw'],'blocks':s['blocks']}
        labels[name]={}
        for epoch,start in epochs.items():
            dynamic=geometry(native,s['blocks'],old['initial_yaw'],truth,shapes,scene,start,policy)
            labels[name][epoch]=full_context(source,native,blocks,record,truth,scene,shapes,start,policy,dynamic)
            if name==STAGES[2] and epoch==EPOCHS[1]:
                for i,r in enumerate(labels[name][epoch]):
                    before=legacy_labels[i]
                    for key in ['geometry','gates','progress_x','raw_interval_margin_lower','conditional_safe_control_with_progress']:
                        if r[key]!=before[key]:raise ValueError('all311 original source-pose full fixture labels changed')
            for i in range(300):
                r=labels[name][epoch][i+s['raw_row_offset']]
                if name==STAGES[0]:
                    before=raw_labels['rows'][i]['epochs'][epoch]
                    if r['gates']['frozen_native_raw203_with_interval_reserve']!=before['raw203_conservative_vector_gate_pass'] or abs(r['raw_interval_margin_lower']-before['raw203_conservative_vector_interval_lower_m'])>2e-12:raise ValueError('original all300 raw map labels changed')
                    if dynamic['rows'][i]!=before['frozen_same_raw_physical_dynamic_labels']:raise ValueError('original raw dynamic labels changed')
                elif name==STAGES[2] and r!=after_labels['rows'][i]['individual_full_labels'][epoch]:raise ValueError('all300 original individual SG full fixture labels changed')
        summaries[name]={e:{'map_vector_pass_rows':sum(labels[name][e][i+s['raw_row_offset']]['gates']['frozen_native_raw203_with_interval_reserve'] for i in range(300)),
            'all_computed_fixture_gates_rows':sum(labels[name][e][i+s['raw_row_offset']]['conditional_safe_control_with_progress'] for i in range(300))} for e in EPOCHS}
    for i in range(300):
        stages={name:{'model_row':i+s['raw_row_offset'],'epochs':{e:labels[name][e][i+s['raw_row_offset']] for e in EPOCHS}} for name,s in zip(STAGES,stage['stages'])}
        changes={}
        for a,b in [(0,1),(1,2)]:
            before,after=stage['stages'][a],stage['stages'][b];fields={}
            for field in FIELDS:
                x=before['blocks'][field]['bytes'][(i+before['raw_row_offset'])*120:(i+before['raw_row_offset']+1)*120]
                y=after['blocks'][field]['bytes'][(i+after['raw_row_offset'])*120:(i+after['raw_row_offset']+1)*120]
                fields[field]=sum(x[j:j+4]!=y[j:j+4] for j in range(0,120,4))
            changes[STAGES[a]+'__to__'+STAGES[b]]=fields
        rows.append({'original_raw_row':i,'stages':stages,'changed_float32_values':changes})
    transitions={}
    for a,b in [(0,1),(1,2)]:
        transitions[STAGES[a]+'__to__'+STAGES[b]]={e:{'pass_to_fail':[i for i in range(300) if rows[i]['stages'][STAGES[a]]['epochs'][e]['gates']['frozen_native_raw203_with_interval_reserve'] and not rows[i]['stages'][STAGES[b]]['epochs'][e]['gates']['frozen_native_raw203_with_interval_reserve']],
            'fail_to_pass':[i for i in range(300) if not rows[i]['stages'][STAGES[a]]['epochs'][e]['gates']['frozen_native_raw203_with_interval_reserve'] and rows[i]['stages'][STAGES[b]]['epochs'][e]['gates']['frozen_native_raw203_with_interval_reserve']]} for e in EPOCHS}
    # Explicit fixed raw185 subset: three stages, including distinct auxiliary offsets.
    record={'rows':3,'model_dt':stage['model_dt'],'initial_yaw':old['initial_yaw'],'blocks':{}}
    for field in FIELDS:
        record['blocks'][field]={'values':[v for s in stage['stages'] for v in s['blocks'][field]['values'][(185+s['raw_row_offset'])*30:(186+s['raw_row_offset'])*30]],'shape':[3,30]}
    scalar_input={'native_source_ordinal':298,'epoch_summaries':{e:{k:raw_labels['epoch_summaries'][e][k] for k in ['epoch','samples']} for e in EPOCHS},
        'rows':[{'factor':'raw185_'+name,'full_conditional_labels':rows[185]['stages'][name]['epochs']} for name in STAGES]}
    scalar=verify_record(source,record,scalar_input)
    scalar['registered_original_raw_rows']=[185];scalar['scope']+=' Exact cell scope is three raw185 stage paths, not all300.'
    report={'schema':'native_individual_stages_v1','native_source_ordinal':298,'raw_rows':300,'stages':list(STAGES),'stage_auxiliary_offsets':[0,11,11],
        'references':{name:hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest() for name,root in [('source',source),('witness',witness),('raw_map',raw_map),('mass',mass)]},
        'prepared_input_sha256':identity['input_sha256'],'stage_output_sha256':hashlib.sha256(stage_file.read_bytes()).hexdigest(),
        'original_raw_nine_tensor_bytes_exact':True,'entire_old311_binary_and_JSON_bytes_exact':True,'all300_afterSG_full_labels_exact':True,'all311_source_pose_full_labels_exact':True,
        'epoch_summaries':{e:{k:raw_labels['epoch_summaries'][e][k] for k in ['epoch','horizon','samples','max_sample_interval']} for e in EPOCHS},
        'stage_summaries':summaries,'map_gate_transitions':transitions,'rows':rows,
        'scope':'Same frozen sampled controls, installed constraints and independent actual SG history. Fixture gate sets are descriptive hypotheses, not published or accepted controls.',
        'limits':['All300 map labels use conservative vector bounds; exact blocked-cell distances cover only the three raw185 stage paths.',
            'Pre-SG auxiliary rows retain the original311 layout to preserve native arithmetic. They are not additional sampler candidates.',
            'Individual stage effects do not establish aggregate or closed-loop behavior, individual original critic increments or global optimizer defects.',
            'Physical e/B and complete CostCritic state remain unknown; original task/raw203/age FAILED and composite NOT ESTABLISHED remain.']}
    return report,scalar


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','witness','raw_map','mass','prepared','legacy_binary','legacy_json','stage_file','output']:parser.add_argument(name,type=Path)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('refusing to overwrite stage attribution')
    report,scalar=analyze(*[getattr(args,n) for n in ['source','witness','raw_map','mass','prepared','legacy_binary','legacy_json','stage_file']])
    args.output.mkdir()
    for name,value in [('report.json',report),('raw185_scalar.json',scalar)]:
        with (args.output/name).open('x') as stream:stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'stage_summaries':report['stage_summaries'],'map_gate_transitions':report['map_gate_transitions'],'raw185_scalar':scalar['verdict']}),flush=True)
