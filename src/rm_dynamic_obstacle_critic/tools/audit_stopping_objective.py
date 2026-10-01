#!/usr/bin/env python3
"""Summarize sampled stopping-proxy diagnostics and actual controller deadline warnings."""
import argparse
import gzip
import json
from pathlib import Path
import statistics
from trial_io import rows


def analyze(root):
    execution = json.loads((root / 'execution.json').read_text())
    samples = []
    for row in rows(root, 'observations.jsonl'):
        if row['kind'] == 'stopping' and execution['start_sim'] is not None and row['receive_sim'] >= execution['start_sim']:
            samples.extend(s['values'] for s in row['statuses'] if s['reason'] == 'ok')
    times = [float(s['score_ms']) for s in samples]
    passing = [int(float(s['passing_proxies'])) for s in samples]
    log = root / 'launch.log'
    if log.exists():
        text = log.read_text()
    else:
        with gzip.open(root / 'launch.log.gz', 'rt') as stream:
            text = stream.read()
    return {'scope': 'one-second sampled native candidate-proxy diagnostics; no final-command certificate',
        'diagnostic_samples': len(samples), 'score_ms_median': statistics.median(times) if times else None,
        'score_ms_max': max(times) if times else None, 'sampled_score_over_100ms': sum(t > 100 for t in times),
        'passing_proxies_min': min(passing) if passing else None,
        'passing_proxies_median': statistics.median(passing) if passing else None,
        'passing_proxies_max': max(passing) if passing else None,
        'sampled_batches_with_no_passing_proxy': sum(p == 0 for p in passing),
        'whole_run_controller_deadline_warning_count': text.count('Control loop missed its desired rate'),
        'limits': 'score_ms covers this critic only and is sampled at 1Hz. This is not a per-cycle deadline proof or evidence of complete final-control/SG sampler coverage.'}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('trial', type=Path); args = parser.parse_args()
    result = analyze(args.trial)
    (args.trial / 'stopping_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
