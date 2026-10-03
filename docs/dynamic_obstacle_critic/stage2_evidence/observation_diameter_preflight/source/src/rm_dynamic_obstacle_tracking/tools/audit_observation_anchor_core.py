#!/usr/bin/env python3
"""Deterministic default-mode regression against the frozen original core.

This offline tool is the only importer of the frozen reference. Runtime
consumers continue to use public ROS messages only. Synthetic traces do not
provide real measurement, association or motion guarantees.
"""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import random
import sys

from rm_dynamic_obstacle_tracking import core

REFERENCE_SHA = 'dcb12364af706d3b5d9db5e0e37363b550bfb8740bcdf99bcd6bfc908b4cb413'


def encode(value):
    return json.dumps(asdict(value), sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode()


def run(reference_path):
    if hashlib.sha256(reference_path.read_bytes()).hexdigest() != REFERENCE_SHA:
        raise ValueError('frozen 41e96399 core identity mismatch')
    spec = importlib.util.spec_from_file_location('frozen_observation_reference', reference_path)
    reference = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = reference; spec.loader.exec_module(reference)
    digest = hashlib.sha256()
    counts = dict(default_updates=0, observation_mode_updates=0,
                  observed_tracks=0, coasting_tracks=0, resets=0)
    rng = random.Random(20261003)
    for global_assignment in (False, True):
        options = dict(association_gate=.7, process_noise=3., measurement_noise=.08,
            min_hits_to_confirm=3, min_displacement_to_confirm=.15,
            use_global_assignment=global_assignment, max_coast_time_sec=.6,
            prediction_steps=30, prediction_dt=.1, max_update_dt=.05)
        old = reference.MultiObjectTracker(**options)
        default = core.MultiObjectTracker(**options)
        unclipped = core.MultiObjectTracker(**options, velocity_decay_tau=0., max_prediction_speed=0.)
        observation = core.MultiObjectTracker(**options, velocity_decay_tau=0., max_prediction_speed=0.,
                                             public_anchor_mode='last_observation_cv')
        anchors = {}
        stamp = 1700000000.125
        for frame in range(1200):
            stamp += .9 if frame % 79 == 0 else .1
            if frame % 137 == 0: stamp -= 2.
            if frame % 131 == 0: stamp -= .1  # repeated source epoch when regular dt was .1
            detections = []
            for index in range(3):
                if rng.random() < .25: continue
                detections.append((index*1.5+(frame%60)*.025+rng.uniform(-.08, .08),
                                   index*.35+rng.uniform(-.12, .12),
                                   rng.uniform(.1, .7), rng.uniform(.1, .5)))
            def converted(module):
                return [module.Detection(module.Point2D(x, y), sx, sy, 8)
                        for x, y, sx, sy in detections]
            baseline = old.update(converted(reference), stamp)
            actual = default.update(converted(core), stamp)
            if encode(actual) != encode(baseline):
                raise AssertionError(f'default mismatch at global={global_assignment}, frame={frame}')
            digest.update(encode(actual)); counts['default_updates'] += 1
            a = unclipped.update(converted(core), stamp)
            b = observation.update(converted(core), stamp)
            assert (a.created, a.deleted, a.time_reset) == (b.created, b.deleted, b.time_reset)
            counts['resets'] += b.time_reset
            live = {track.track_id for track in b.tracks}
            anchors = {key: value for key, value in anchors.items() if key in live}
            for filtered, observed in zip(a.tracks, b.tracks):
                lhs, rhs = asdict(filtered), asdict(observed)
                for key in ('position', 'prediction'):
                    lhs.pop(key); rhs.pop(key)
                assert lhs == rhs, (global_assignment, frame, filtered.track_id)
                if observed.misses == 0:
                    assert (observed.position.x, observed.position.y) in [(d[0], d[1]) for d in detections]
                    anchors[observed.track_id] = (observed.position, stamp)
                    counts['observed_tracks'] += 1
                else:
                    anchor, epoch = anchors[observed.track_id]
                    elapsed = stamp-epoch
                    assert observed.position == core.Point2D(anchor.x+observed.velocity.x*elapsed,
                                                            anchor.y+observed.velocity.y*elapsed)
                    counts['coasting_tracks'] += 1
                for k, point in enumerate(observed.prediction):
                    future = (k+1)*.1
                    assert point == core.Point2D(observed.position.x+observed.velocity.x*future,
                                                observed.position.y+observed.velocity.y*future)
            counts['observation_mode_updates'] += 1
    return dict(schema=1, status='PASS', reference_commit='41e96399',
        reference_core_sha256=REFERENCE_SHA, default_trace_sha256=digest.hexdigest(),
        counts=counts, seed=20261003,
        scope='Synthetic source-time and default numerical regression only; no physical or deployment certificate.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('reference_core', type=Path)
    parser.add_argument('new_report', type=Path)
    args = parser.parse_args()
    if args.new_report.exists():
        raise FileExistsError(args.new_report)
    report = run(args.reference_core)
    args.new_report.write_text(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print(json.dumps(report, sort_keys=True))
