#!/usr/bin/env python3
"""Freeze real consumed age/velocity and explicitly synthetic support queries."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import yaml
from conditional_cv_support import KEYS
from dynamic_consumption_io import read_consumption
from replay_processed_cv import verify


def prepare(source, output):
    verify(source)
    consumed, _ = read_consumption(source/'dynamic_scores/score_298.json')
    usable = [t for t in consumed['input_used']['tracks'] if t['state'] in (2,3)]
    if len(usable)!=1 or usable[0]['id']!=1 or consumed['ordinal']!=298:
        raise ValueError('registered source298/ID1 context required')
    output.mkdir()
    config = Path(__file__).resolve().parents[1]/'config'
    for name in ['cv_support_assumptions_offline.yaml','ground_robot_extent_prior_offline.yaml']:
        shutil.copyfile(config/name,output/name)
    actual = {'schema':1,'velocity_xy':usable[0]['vxy'],'source_age_sec':consumed['source_age_used'],
              'future_offsets_sec':[0,1,2,3],
              'scope':'Actual source298 consumed velocity/age; all physical support assumptions remain unknown.'}
    (output/'actual_query.json').write_text(json.dumps(actual,indent=2)+'\n')
    synthetic = {'schema':1,'units':'m_s','time_reference':'message_source_epoch',
                 'motion_semantics':'complete_projection_directed_Hausdorff_from_source',
                 'bounds':dict(zip(KEYS,[.125,.25,.5,3.5])),
                 'condition_sources':dict.fromkeys(KEYS,'synthetic:constructive-line-set/test-only')}
    (output/'synthetic_assumptions.yaml').write_text(yaml.safe_dump(synthetic,sort_keys=False))
    query = {'schema':1,'velocity_xy':[-1.5,0],'source_age_sec':.5,'future_offsets_sec':[0,1,2,3],
             'scope':'Synthetic line body, q=-e, CV velocity opposite true translation; no physical target certificate.'}
    (output/'synthetic_query.json').write_text(json.dumps(query,indent=2)+'\n')
    identity = {'source_manifest_sha256':hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest(),
                'source_ordinal':298,'consumed_score_sha256':hashlib.sha256((source/'dynamic_scores/score_298.json').read_bytes()).hexdigest(),
                'source_stamp':consumed['input_used']['stamp_sec']+consumed['input_used']['stamp_nanosec']*1e-9,
                'score_stamp':consumed['score_stamp'],
                'files':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir())},
                'scope':'Actual age/velocity plus unknown conditions; separate synthetic constructive assumption proof only.'}
    (output/'input_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    return identity


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();print(json.dumps(prepare(args.source.resolve(),args.output)),flush=True)
