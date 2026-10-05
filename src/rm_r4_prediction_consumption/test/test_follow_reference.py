"""Four bounded math comparisons; no tracker/frontend/execution imports.

Explicit test-only adaptations of fixed A02: actual limits/body/cruise/reserve.
Python OSQP 1.0.5 remains a reference, never a production dependency or ABI.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from types import ModuleType, SimpleNamespace


def main(probe, dependencies, output):
    root = Path(__file__).resolve().parents[3]
    expected = {
        'follow.py': 'f4ace3ff3bbef007593c822f401023d74ec084a2f02f89557f1865a4a9110df4',
        'contracts.py': 'f3891d6c0a4fc86b846b12d02db2195e8678db3636094aa205af4619c149c716',
        'soft_field.py': '89a89623f4ad979d0ad074ef96ed65d5d119553721389bbed047a576c13e5d2e',
    }
    folder = root/'experiments/r4_hws_prediction_consumption/r4_hws'
    for name, sha in expected.items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest() == sha
    sys.path[:0] = [str(dependencies), str(folder.parent)]
    import numpy as np
    import osqp
    from r4_hws.contracts import CycleSnapshot, PreparedRoute
    from r4_hws.soft_field import TemporalSoftField

    # The prototype also hard-codes unit target rate in its seed/projection.
    # Adapt all these constants explicitly; keep the fixed source untouched.
    source_text = (folder/'follow.py').read_text()
    old = 'math.hypot(.8, .5)'
    assert source_text.count(old) == 1
    adaptations = {
        old: 'math.hypot(max(-cfg.velocity_lower[0], cfg.velocity_upper[0]), '
             'max(-cfg.velocity_lower[1], cfg.velocity_upper[1]))',
        'target-velocity, -limit, limit':
            'target-velocity, -limit*np.asarray(cfg.command_rate), limit*np.asarray(cfg.command_rate)',
        'last-interval': 'last-interval*np.asarray(cfg.command_rate)',
        'last+interval': 'last+interval*np.asarray(cfg.command_rate)',
    }
    adapted = source_text
    for before, after in adaptations.items():
        assert adapted.count(before) == 1
        adapted = adapted.replace(before, after)
    reference = ModuleType('r4_hws.follow_reference')
    reference.__package__ = 'r4_hws'
    sys.modules[reference.__name__] = reference
    exec(compile(adapted, str(folder/'follow.py')+' [test actual reserve]', 'exec'), reference.__dict__)

    class ActualBody(TemporalSoftField):
        physical_half_extents = (.32, .27)
        robot_padding = .02

    reference.TemporalSoftField = ActualBody
    native = json.loads(subprocess.check_output([probe], text=True))
    records = []
    source_ns = 1_820_000_000_173_456_789
    for actual in native:
        scenario = actual['scenario']
        points = ((-1., 0.), (0., 0.), (1., 0.))
        if scenario == 1:
            points = tuple(reversed(points))
        route = PreparedRoute(points, (0., 1., 2.), (tuple(actual['bounds']),)*3, 'math-path', 'math-map', 4)
        tracks = ()
        if scenario == 3:
            tracks = (SimpleNamespace(origin=(-.1, 0.), cells=((0, 0), (2, 0), (3, 0)),
                resolution=.05, anchor=(.7, 0.), velocity=(0., 0.), source_ns=source_ns,
                observation_ns=source_ns-100_000_000, track_id=7),)
        solver = reference.FollowMPC()
        config = asdict(reference.FollowConfig())
        config.update(velocity_lower=(-.3, -.5), velocity_upper=(.5, .5), command_rate=(.8, .8),
                      cruise=.4, progress_upper=.5)
        solver.config = SimpleNamespace(**config)
        snapshot = CycleSnapshot(1, source_ns, time.perf_counter_ns(), source_ns, source_ns,
            (0., 0., actual['yaw'], 0., 0., 0.), (0., 0., 0.), source_ns, 2, source_ns,
            tracks, route, 1., tuple(actual['bounds']), 'reference-math-only')
        solved = solver.solve(snapshot)
        assert solved.status == 'follow', (scenario, solved.reason, solved.solver_status)
        control_error = float(np.max(np.abs(np.asarray(actual['controls'])-solved.controls)))
        stage_error = float(np.max(np.abs(np.asarray(actual['stages'])-solved.states)))
        nominal_error = abs(actual['nominal_dynamic_cost']-solved.nominal_dynamic_cost)
        cost_error = abs(actual['solved_dynamic_cost']-solved.solved_dynamic_cost)
        assert control_error < 2e-5 and stage_error < 2e-5, (scenario, control_error, stage_error)
        assert nominal_error < 1e-9 and cost_error < 1e-4, (scenario, nominal_error, cost_error)
        records.append(dict(scenario=scenario, control_error=control_error, stage_error=stage_error,
            nominal_dynamic_cost_error=nominal_error, solved_dynamic_cost_error=cost_error,
            cpp_solver_s=actual['solver_s'], cpp_elapsed_s=actual['elapsed_s'],
            cpp_iterations=actual['iterations'], python_solver_s=solved.solver_s))
    assert len(records) == 4
    evidence = dict(scope='four numerical references only; no closed loop or runtime owner',
        cpp_osqp='0.6.3', python_osqp=osqp.__version__, adaptations=[
            'actual velocity and target rate bounds, seed and projection', 'actual body and padding',
            'explicit cruise/progress limits', 'reserve from actual velocity maxima'],
        reference_source_sha256=expected, cases=records,
        test_only_source_substitutions=adaptations,
        imported_harness_modules=['contracts', 'soft_field', 'follow reference only'],
        tracker_frontend_execution_imported=False)
    output.write_text(json.dumps(evidence, indent=2)+'\n')
    print(json.dumps(evidence, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('probe')
    parser.add_argument('dependencies', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    main(args.probe, args.dependencies, args.output)
