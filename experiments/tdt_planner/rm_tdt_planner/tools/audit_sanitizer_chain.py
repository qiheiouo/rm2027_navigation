#!/usr/bin/env python3
"""Verify the actual sanitizer compile/loader boundary before executing GTest.

Reads three compilation databases and preprocesses their C++ flags. It does not
change Eigen alignment, suppress errors, or execute the planner.
"""
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys


def output(args, cwd=None):
    return subprocess.check_output(args, cwd=cwd, text=True, stderr=subprocess.STDOUT)


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def audit(root, result):
    prefix = (root / 'deps').resolve()
    required = {'Solver.cpp', 'minimumSnap.cpp', 'planner.cpp',
                'test_planner.cpp', 'test_solver_lifetime.cpp', 'osqp.c', 'qdldl.c'}
    seen = set()
    signatures = []
    names = ('EIGEN_MALLOC_ALREADY_ALIGNED', 'EIGEN_DEFAULT_ALIGN_BYTES',
             'EIGEN_MAX_ALIGN_BYTES', 'EIGEN_MAX_STATIC_ALIGN_BYTES',
             'EIGEN_IDEAL_MAX_ALIGN_BYTES')
    result['compilation_units'] = []
    for component in ('deps-build/osqp', 'deps-build/osqp-eigen', 'core'):
        database = root / component / 'compile_commands.json'
        for entry in json.loads(database.read_text()):
            args = entry.get('arguments') or shlex.split(entry['command'])
            source = Path(entry['file'])
            seen.add(source.name)
            require('-fsanitize=address,undefined' in args, 'Missing sanitizer: ' + str(source))
            require('-fno-sanitize-recover=all' in args, 'Recovering sanitizer: ' + str(source))
            require(not any(a.startswith('-fno-sanitize=') for a in args), 'Disabled checks: ' + str(source))
            unit = {'source': str(source), 'command': args}
            result['compilation_units'].append(unit)
            if source.suffix not in ('.cc', '.cpp', '.cxx'):
                continue
            preprocessor = []
            i = 0
            while i < len(args):
                arg = args[i]
                if arg == '-o':
                    i += 2
                    continue
                if arg not in ('-c', entry['file']):
                    preprocessor.append(arg)
                i += 1
            text = output(preprocessor + ['-dM', '-E', '-x', 'c++', '-include',
                                          'Eigen/Core', '/dev/null'], cwd=entry['directory'])
            macros = {}
            for line in text.splitlines():
                parts = line.split(maxsplit=2)
                if len(parts) == 3 and parts[0] == '#define':
                    macros[parts[1]] = parts[2]
            require(macros.get('__SANITIZE_ADDRESS__') == '1', 'ASan macro missing: ' + str(source))
            require(macros.get('EIGEN_MALLOC_ALREADY_ALIGNED') == '0', 'Unexpected Eigen allocation: ' + str(source))
            unit['eigen_macros'] = {name: macros[name] for name in names}
            signatures.append(unit['eigen_macros'])
    require(required <= seen, 'Missing compilation units: ' + str(sorted(required - seen)))
    require(signatures and all(s == signatures[0] for s in signatures), 'Eigen allocation/alignment differs across C++ units')
    executable = root / 'core/test_planner'
    loaded = output(['ldd', str(executable)])
    (root / 'ldd.txt').write_text(loaded)
    require('not found' not in loaded, 'Missing dynamic library; see ldd.txt')
    result['solver_libraries'] = {}
    for library in ('libOsqpEigen', 'libosqp'):
        match = re.search(r'^\s*' + library + r'\.so[^ ]* => (\S+)', loaded, re.M)
        require(match is not None, 'Solver shared library missing: ' + library)
        resolved = Path(match.group(1)).resolve()
        require(prefix in resolved.parents, 'Solver loaded outside new prefix: ' + str(resolved))
        symbols = output(['nm', '-D', str(resolved)])
        require('__asan_init' in symbols and '__ubsan_handle_' in symbols,
                'Solver library is not instrumented with ASan and UBSan: ' + str(resolved))
        result['solver_libraries'][library] = {
            'path': str(resolved), 'sha256': hashlib.sha256(resolved.read_bytes()).hexdigest(),
            'asan_and_ubsan_symbols': True}
    result['eigen_allocation_consistent'] = True
    result['passed'] = True


def main():
    root = Path(sys.argv[1]).resolve()
    result = {'schema': 'rm_tdt_planner/sanitizer_chain/v1', 'passed': False}
    try:
        audit(root, result)
    except Exception as error:
        result['error'] = str(error)
    with (root / 'chain_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print('Sanitizer chain audit:', result.get('error', 'passed'), flush=True)
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
