#!/usr/bin/env python3
"""Archive small exact cycle records and decode existing observations for review."""
import argparse
import csv
import hashlib
import json
import pathlib
import shutil
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ns(stamp):
    return stamp['sec']*1000000000+stamp['nanosec']


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=pathlib.Path)
    args=parser.parse_args();out=args.output.resolve();root=pathlib.Path(__file__).resolve().parents[2]
    evidence=root/'experiments/r4_runtime_shadow/evidence';evidence.mkdir(exist_ok=True)
    consistency={};observations=[];runs={}
    for scene in ['S0','S1','S2']:
        run=out/scene
        for filename in ['cycles.csv','references.csv','events.json','lease_estimates.csv','summary.json','timeseries.png']:
            shutil.copyfile(run/filename,evidence/(scene+'_'+filename))
        runs[scene]=json.loads((run/'run_manifest.json').read_text())
        public=[];embedded=[];sequences=[];member_counts=[]
        for line in (run/'prediction_messages.jsonl').open():
            data=json.loads(line);m=data['message']
            if data['topic'].endswith('/predictions'):
                public.append(hashlib.sha256(json.dumps(m,sort_keys=True).encode()).hexdigest());continue
            embedded.append(hashlib.sha256(json.dumps(m['prediction'],sort_keys=True).encode()).hexdigest())
            sequences.append(m['sequence']);member_counts.append(len(m['tracks']))
            predictions={t['track_id']:t for t in m['prediction']['tracks']}
            for t in m['tracks']:
                pred=predictions.get(t['track_id']);center=t['centroid_at_observation'];ends=t['local_endpoints']
                observations.append(dict(scene=scene,producer_id=m['producer_id'],producer_generation=m['producer_generation'],
                    receipt_sequence=m['sequence'],prediction_source_ns=ns(m['prediction']['header']['stamp']),
                    prediction_processing_ns=ns(m['prediction']['processing_stamp']),track_id=t['track_id'],
                    observation_ns=ns(t['last_observation_stamp']),association_sequence=t['association_sequence'],
                    centroid_x=center['x'],centroid_y=center['y'],member_count=len(ends),
                    observed_y_min=min((center['y']+p['y'] for p in ends),default=''),
                    observed_y_max=max((center['y']+p['y'] for p in ends),default=''),
                    prediction_vx=pred['velocity']['x'] if pred else '',prediction_vy=pred['velocity']['y'] if pred else ''))
        consistency[scene]=dict(public_messages=len(public),private_messages=len(embedded),
                                public_embedded_multiset_identical=sorted(public)==sorted(embedded),
                                receipt_sequence_strictly_increasing=all(b>a for a,b in zip(sequences,sequences[1:])),
                                nonempty_member_messages=sum(c>0 for c in member_counts),
                                meaning='Recorded public wire values compared with the same private envelope; no prediction recomputation.')
    with (evidence/'observed_members.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(observations[0]),lineterminator='\n');writer.writeheader();writer.writerows(observations)
    failures={}
    for p in sorted(out.glob('*failure*')):
        if not p.is_dir():continue
        rows=list(csv.DictReader((p/'cycles.csv').open())) if (p/'cycles.csv').exists() else []
        failures[p.name]=dict(cycles=len(rows),valid=sum(r['valid']=='1' for r in rows),
                              events=json.loads((p/'events.json').read_text()) if (p/'events.json').exists() else None,
                              manifest=json.loads((p/'run_manifest.json').read_text()) if (p/'run_manifest.json').exists() else
                              json.loads((p/'failure_manifest.json').read_text()) if (p/'failure_manifest.json').exists() else None,
                              behavioral_evidence=False)
    dependencies=json.loads((out/'dependency_provenance.json').read_text())
    provenance=dict(stage='A13 real ROS runtime shadow',algorithm_baseline='e137635ee8c59888ffad853d1cd9ababb8df8ea6',
                    runtime_base_commit='f138c72d772163b16372e3a1ed7261262e0d37b5',
                    source_version='Base commit plus exact per-run uncommitted adapter hashes and caller binary hash below; final Git commit contains these adapters.',
                    branch=subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True).strip(),
                    runs=runs,dependencies=dependencies,preservation=json.loads((out/'preservation_checks.json').read_text()),
                    public_private_consistency=consistency,raw_output_directory=str(out),
                    raw_evidence_hash_index=dict(path=str(out/'evidence_hashes.json'),sha256=sha(out/'evidence_hashes.json')),
                    current_adapter_sources={str(p.relative_to(root)):sha(p) for p in pathlib.Path(__file__).parent.iterdir() if p.is_file()},
                    generated_scene_assets={str(p.relative_to(out)):sha(p) for p in (out/'assets').iterdir() if p.is_file()},
                    logs={str(p.relative_to(out)):sha(p) for p in out.iterdir() if p.is_file() and p.suffix in ['.log','.txt']},
                    verdict=dict(unit_value='Prior finite A12 PASS retained, not rerun',runtime_shadow='FAILED_INPUT_APPLICABILITY; dynamic consumption behavior INCONCLUSIVE',
                                 closed_loop='NOT_EVALUATED; NOT_ELIGIBLE',deployment='NOT_EVALUATED'),
                    scope=dict(behavioral_scenes=3,seconds_per_scene=20,large_paired_experiments=False,
                               new_tracker=False,new_prediction_pipeline=False,new_frontend=False,new_solver=False,
                               R4_robot_command_publishers=0,R4_active_controller=False,A10_enforcement=False,
                               algorithm_changes=False,production_contract_changes=False,hardware=False))
    (evidence/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    (evidence/'startup_failures.json').write_text(json.dumps(failures,indent=2)+'\n')
    shutil.copyfile(out/'summary.json',evidence/'summary.json')
    shutil.copyfile(out/'evidence_hashes.json',evidence/'raw_evidence_hashes.json')
    print(json.dumps(dict(consistency=consistency,observed_rows=len(observations),startup_failures=len(failures))))


if __name__=='__main__':main()
