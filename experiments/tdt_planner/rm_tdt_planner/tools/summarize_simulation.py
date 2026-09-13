#!/usr/bin/env python3
"""Audit raw trial evidence and summarize a complete or interrupted static matrix."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
from simulation_evidence import analyze

PLANNERS = ('navfn', 'smac2d', 'tdt_astar', 'tdt_qp')


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def read_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def inspect_trial(path):
    report = {'directory': path.name, 'raw_summary_matches': False}
    try:
        meta = json.loads((path/'metadata.json').read_text())
        report.update(meta)
        if meta['planner'] not in PLANNERS or meta['trial'] not in range(1, 6):
            raise ValueError('invalid planner/trial identity')
        if path.name != f"{meta['planner']}_{meta['trial']}":
            raise ValueError('directory/metadata identity mismatch')
        if digest(path/'profile.yaml') != meta['profile_sha256']:
            raise ValueError('frozen profile hash mismatch')
        summary = json.loads((path/'observation/summary.json').read_text())
        report['summary'] = summary
        report['docker_exit'] = int((path/'docker_exit.txt').read_text())
        if 'error' not in summary:
            if summary['goal'] != [4.3, 0.0, 0.0]:
                raise ValueError('goal differs from the static fixture protocol')
            events = read_rows(path/'observation/events.jsonl')
            results = [e for e in events if e.get('event') == 'navigation_result']
            preflights = [e for e in events if e.get('event') == 'preflight']
            if len(results) != 1 or any(results[0].get(k) != summary.get(v) for k, v in
                    (('status', 'action_status'), ('timed_out', 'timed_out'),
                     ('recoveries', 'recoveries'), ('goal', 'goal'))):
                raise ValueError('action result differs from raw event evidence')
            if len(preflights) != 1 or preflights[0].get('result') != summary.get('preflight'):
                raise ValueError('preflight result differs from raw event evidence')
            expected = analyze(read_rows(path/'observation/trajectory.jsonl'),
                               read_rows(path/'observation/commands.jsonl'),
                               events,
                               4.3, 0.0, summary['action_status'], summary['timed_out'], summary['recoveries'])
            expected['evidence_valid'] &= not summary['clock_reversed'] and summary['fixture_spawn_verified']
            expected['static_geometry_and_goal_pass'] &= expected['evidence_valid']
            if any(summary.get(key) != value for key, value in expected.items()):
                raise ValueError('summary differs from raw evidence recomputation')
            report['raw_summary_matches'] = True
        files = [p for p in path.iterdir() if p.is_file()]
        files += [p for p in (path/'observation').iterdir() if p.is_file()]
        report['artifact_sha256'] = {str(p.relative_to(path)): digest(p) for p in sorted(files)}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        report['audit_error'] = str(exc)
    return report


def summarize(root):
    trials = [inspect_trial(p) for p in sorted(root.iterdir()) if p.is_dir() and
              any(p.name.startswith(name+'_') for name in PLANNERS)]
    expected = {(p, t) for p in PLANNERS for t in range(1, 6)}
    actual = [(r.get('planner'), r.get('trial')) for r in trials]
    duplicates = len(set(actual)) != len(actual)
    identities = {json.dumps([r.get('source_commit'), r.get('image_id'), r.get('fixture_sha256')], sort_keys=True)
                  for r in trials if 'source_commit' in r}
    profiles_consistent = all(len({r.get('profile_sha256') for r in trials if r.get('planner') == name}) <= 1
                              for name in PLANNERS)
    consistent = bool(trials and len(identities) == 1 and not duplicates and profiles_consistent and
                      all('audit_error' not in r and r.get('raw_summary_matches') for r in trials))
    complete = set(actual) == expected and len(actual) == 20
    groups = []
    for planner in PLANNERS:
        values = [r for r in trials if r.get('planner') == planner]
        valid = [r['summary'] for r in values if r.get('raw_summary_matches') and r['summary'].get('evidence_valid')]
        passed = [s for s in valid if s['static_geometry_and_goal_pass']]
        groups.append({'planner': planner, 'recorded': len(values), 'valid_evidence': len(valid),
                       'limited_static_pass': len(passed),
                       'mean_cross_track_rms_m_on_passed_runs': statistics.mean(s['cross_track_rms_m'] for s in passed) if passed else None,
                       'mean_preflight_action_wall_ms': statistics.mean(s['preflight']['wall_ms'] for s in valid) if valid else None,
                       'snapshot_rejection_observed': any(s['snapshot_rejection_observed'] for s in valid)})
    return {'schema': 'rm_tdt_planner/static_simulation_matrix/v1', 'complete_20_trial_matrix': complete,
            'consistent_and_audited': consistent, 'missing_trials': sorted(expected-set(actual)),
            'all_recorded_static_checks_pass': bool(complete and consistent and all(
                r['summary'].get('evidence_valid') and r['summary'].get('static_geometry_and_goal_pass') and
                r.get('docker_exit') == 0 for r in trials)),
            'groups': groups, 'trials': trials, 'accepted_for_deployment': False,
            'manual_review_required': ['fixture geometry and costmap marking', 'canonical TF ownership',
                                       'tracking plots and failed-plan behavior', 'resource symptoms and data gaps'],
            'performance_policy': 'Current-device limits are recorded, never an algorithm rejection.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if not args.run_directory.is_dir():
        parser.error('run directory does not exist')
    report = summarize(args.run_directory)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(f"Saved {args.output}; complete={report['complete_20_trial_matrix']}, audited={report['consistent_and_audited']}")
    # Incomplete matrices must still produce a reviewable report, not manufacture a pass.
    return 0 if report['all_recorded_static_checks_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
