#!/usr/bin/env python3
"""Verify recorded-input seed regression; no ROS runtime or Follow call here."""
import argparse
import collections
import csv
import hashlib
import json
import math
import pathlib
import shutil
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BASE = '0266f2ba1b7e708e42e888f6b68e4dfaf8ea5867'


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def valid(row):
    return row['valid'] == '1'


def summary(rows):
    good = [r for r in rows if valid(r)]
    return dict(owned_rows=len(rows), valid=len(good),
                reasons=dict(collections.Counter(r['reason'] for r in rows if not valid(r))),
                retained_history_on_warm_reset=sum(r['reset_seed'] == '0' and r['reset_warm'] == '1' for r in good),
                used_warm=sum(r['used_warm'] == '1' for r in good),
                vx_range=[min(float(r['vx']) for r in good), max(float(r['vx']) for r in good)] if good else None,
                vy_range=[min(float(r['vy']) for r in good), max(float(r['vy']) for r in good)] if good else None,
                dynamic_cost_max=max((float(r['dynamic_cost']) for r in good), default=None))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=pathlib.Path)
    parser.add_argument('a13', type=pathlib.Path)
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[2]
    replay = read_csv(args.output / 'seed_replay.csv')
    results = {}; checks = {}; fig, axes = plt.subplots(3, 2, figsize=(13, 8))
    for i, scene in enumerate(['S0', 'S1', 'S2']):
        original = read_csv(args.a13 / (scene + '_cycles.csv'))
        by_cycle = {r['cycle']: r for r in original}
        events = {e['kind']: e for e in json.loads((args.a13 / (scene + '_events.json')).read_text())['events']}
        start = events['goal_accepted']['ROS_ns']; end = events['observation_end']['ROS_ns']
        native_end = events.get('native_goal_result', {}).get('ROS_ns', end)
        policies = {p: [r for r in replay if r['scene'] == scene and r['policy'] == p]
                    for p in ['legacy', 'separate_seed_warm']}
        old = policies['legacy']; new = policies['separate_seed_warm']
        expected = {r['cycle'] for r in original if all(r[k] for k in ['pose_source_ns', 'receipt_sequence', 'path_stamp_ns', 'progress_input'])}
        assert {r['original_cycle'] for r in old} == expected
        assert [r['original_cycle'] for r in old] == [r['original_cycle'] for r in new]
        assert len(old) == len(expected)
        for a, b in zip(old, new):
            source = by_cycle[a['original_cycle']]
            assert a['valid'] == source['valid'] and a['reason'] == source['reason'], (scene, a)
            assert a['valid'] == b['valid'] and a['reason'] == b['reason'], (scene, a, b)
            assert a['source_epoch_ns'] == b['source_epoch_ns'] == source['acquire_ros_ns']
            if valid(a):
                assert all(abs(float(a[k])-float(source[k])) <= 1e-12 for k in ['vx', 'vy'])
        for policy, rows in policies.items():
            previous = None
            for r in rows:
                assert r['reason'] not in ['recorded digest mismatch', 'rate constraint violation'], r
                if r['reset_seed'] == '0':
                    assert previous and valid(previous) and int(r['original_cycle']) == int(previous['original_cycle']) + 1
                    assert r['seed_stamp_ns'] == previous['source_epoch_ns']
                    assert 0 < int(r['source_epoch_ns']) - int(r['seed_stamp_ns']) <= 100000000
                    for axis in ['vx', 'vy']:
                        assert abs(float(r['seed_' + axis]) - float(previous[axis])) <= 1e-12
                if valid(r):
                    source = by_cycle[r['original_cycle']]
                    assert abs(float(source['measured_wz'])) <= 1e-6
                    for axis, low, high in [('vx', -.5, .8), ('vy', -.5, .5)]:
                        value = float(r[axis]); seed = float(r['seed_' + axis])
                        assert math.isfinite(value) and math.isfinite(seed)
                        assert low - 2e-5 <= value <= high + 2e-5
                        assert abs(value - seed) <= .05002
                    if r['reset_seed'] == '1':
                        assert float(r['seed_vx']) == float(r['seed_vy']) == 0
                        assert r['seed_stamp_ns'] == r['source_epoch_ns'] and r['reset_warm'] == '1'
                    if policy == 'legacy':
                        assert r['reset_seed'] == r['reset_warm']
                previous = r
        goal = lambda rows: [r for r in rows if start <= int(r['source_epoch_ns']) <= end]
        old_goal = goal(old); new_goal = goal(new)
        original_goal = [r for r in original if start <= int(r['acquire_ros_ns']) <= end]
        assert len(original_goal) == 400
        differences = [max(abs(float(a[k]) - float(b[k])) for k in ['vx', 'vy'])
                       for a, b in zip(old_goal, new_goal) if valid(a)]
        scene_result = dict(original_goal_cycles=len(original_goal), excluded_goal_rows=400-len(old_goal),
                            all_owned={p: summary(rows) for p, rows in policies.items()},
                            goal={p: summary(goal(rows)) for p, rows in policies.items()},
                            max_valid_command_component_difference_mps=max(differences, default=None),
                            changed_valid_goal_commands=sum(v > 1e-8 for v in differences))
        if scene != 'S0':
            clear = events['clear_target']['ROS_ns']
            dynamic = [r for r in new_goal if start + 10**9 <= int(r['source_epoch_ns']) < clear]
            assert not any(valid(r) for r in dynamic)
            scene_result['dynamic_event'] = summary(dynamic)
        results[scene] = scene_result
        checks[scene] = dict(legacy_availability_and_reason_mismatch=0, legacy_valid_command_mismatch=0, policy_availability_and_reason_mismatch=0,
                             recorded_digest_mismatch=0, rate_or_velocity_bound_violation=0,
                             invalid_virtual_history_carry=0, owned_rows_per_policy=len(old),
                             original_unowned_rows=len(original)-len(old))
        for policy, rows in policies.items():
            rows = goal(rows)
            time = [(int(r['source_epoch_ns'])-start)/1e9 for r in rows]
            for j, axis in enumerate(['vx', 'vy']):
                axes[i, j].plot(time, [float(r[axis]) if valid(r) else np.nan for r in rows],
                                label=policy, linewidth=1)
        for j, axis in enumerate(['vx', 'vy']):
            ax = axes[i, j]; ax.set_xlim(0, 20); ax.set_ylabel(scene + ' virtual ' + axis + ' (m/s)')
            ax.grid(alpha=.2); ax.legend(fontsize=8)
            if scene != 'S0':
                ax.axvspan(1, (events['clear_target']['ROS_ns']-start)/1e9, color='orange', alpha=.1)
            if native_end < end:
                ax.axvline((native_end-start)/1e9, color='gray', linestyle='--', alpha=.5)
    for ax in axes[-1]:
        ax.set_xlabel('Original ROS seconds since native goal accepted')
    fig.suptitle('A14 offline seed replay — original measured state; proposals have no actuation authority')
    fig.tight_layout(); fig.savefig(args.output / 'seed_replay.png', dpi=140); plt.close(fig)
    record = dict(base_commit=BASE, algorithm_baseline='e137635e', checks=checks, scenes=results,
                  interpretation='Recorded-input regression PASS only. No new ROS run, runtime behavior PASS, source lease PASS or closed-loop eligibility.',
                  owned_replay_rows=len(replay), original_goal_denominator=1200)
    (args.output / 'seed_replay_summary.json').write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    a13_provenance = json.loads((args.a13 / 'provenance.json').read_text())
    # Keep historical evidence tied to its historical commit, even after the caller patch.
    historical = {}
    for p in sorted(args.a13.iterdir()):
        if not p.is_file():
            continue
        relative = str(p.resolve().relative_to(root))
        expected_bytes = subprocess.check_output(['git', 'show', BASE + ':' + relative], cwd=root)
        assert p.read_bytes() == expected_bytes, relative
        historical[relative] = sha(p)
    frozen = json.loads((root / 'docs/dynamic_navigation/r4_runtime_shadow_checkpoint_sources.json').read_text())
    for asset in frozen['frozen_assets']:
        assert sha(root / asset['path']) == asset['sha256'], asset['path']
    for scene in ['S0', 'S1', 'S2']:
        manifest = json.loads((args.output / 'decode_manifest.json').read_text())[scene]
        for p, digest in manifest['input_files'].items():
            assert sha(root / p) == digest, p
    libraries = {}
    for entry in a13_provenance['dependencies']['linked_and_installed_files']:
        path = pathlib.Path(entry['path'])
        if path.is_relative_to(root):
            assert sha(path) == entry['sha256'], path
            libraries[str(path)] = entry['sha256']
    current_sources = list((pathlib.Path(__file__).parent).glob('*.py')) + list(pathlib.Path(__file__).parent.glob('*.cpp')) + [
        root / 'experiments/r4_input_applicability_audit/CMakeLists.txt',
        root / 'experiments/r4_runtime_shadow/shadow.cpp', root / 'experiments/r4_runtime_shadow/shadow_seed.hpp']
    generated = [args.output / p for p in ['seed_replay.csv', 'seed_replay_summary.json', 'seed_replay.png',
                                          'replay_build/replay', 'caller_build/r4_shadow', 'replay_build.log', 'replay_ldd.txt']]
    provenance = dict(stage='A14 offline seed replay verification', base_commit=BASE,
                      image=a13_provenance['dependencies']['image'], frozen_asset_count=len(frozen['frozen_assets']),
                      sources={str(p.resolve().relative_to(root)): sha(p) for p in current_sources},
                      generated={str(p.resolve().relative_to(root)): sha(p) for p in generated},
                      historical_a13_evidence=historical, unchanged_workspace_dependencies=libraries,
                      execution=dict(new_ROS_nodes=0, new_scenes=0, frozen_Follow_API_used=True,
                                     acquisition_clock='fresh offline steady clock, not original runtime acquisition',
                                     actual_commands_used_as_seed=False, algorithm_changes=False),
                      regression_checks='PASS')
    (args.output / 'seed_replay_provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    evidence = pathlib.Path(__file__).parent / 'evidence'
    for name in ['seed_replay.csv', 'seed_replay_summary.json', 'seed_replay.png', 'seed_replay_provenance.json']:
        shutil.copyfile(args.output / name, evidence / name)
    print(json.dumps(dict(regression='PASS', owned_replay_rows=len(replay), scene_checks=checks), indent=2))


if __name__ == '__main__':
    main()
