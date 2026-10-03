#!/usr/bin/env python3
"""Reject coordinated binary/metadata edits against independent physical truth."""
import argparse
import json
from pathlib import Path
import shutil
import struct
from verify_cv_factor_truth import verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source', 'prepared', 'output']:
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    args.output.mkdir()
    checks = {'genuine_factor_truth': verify(args.source, args.prepared)['verdict']=='PASS'}
    for name in ['future_center', 'source_center', 'local_slope', 'baseline_velocity', 'retained_velocity', 'nonfinite_future']:
        probe = args.output/name
        shutil.copytree(args.prepared, probe)
        identity = json.loads((probe/'input_identity.json').read_text())
        data = bytearray((probe/'factor_input.bin').read_bytes())
        if name in ['future_center', 'nonfinite_future']:
            value = identity['future_centers'][0][1]+.1 if name=='future_center' else float('nan')
            identity['future_centers'][0][1] = value
            struct.pack_into('<d', data, 228+8, value)
        else:
            factor, axis = {'source_center': (1, 0), 'local_slope': (2, 3),
                            'baseline_velocity': (0, 2), 'retained_velocity': (1, 2)}[name]
            value = identity['factors'][factor]['xy_vxy_size'][axis]+.1
            identity['factors'][factor]['xy_vxy_size'][axis] = value
            struct.pack_into('<d', data, 36+factor*48+axis*8, value)
        (probe/'input_identity.json').write_text(json.dumps(identity, indent=2)+'\n')
        (probe/'factor_input.bin').write_bytes(data)
        try:
            verify(args.source, probe)
        except ValueError as error:
            checks[name+'_rejected'] = True
            reason = str(error)
        else:
            checks[name+'_rejected'] = False
            reason = 'unexpected acceptance'
        (probe/'rejection.json').write_text(json.dumps({'probe': name, 'reason': reason}, indent=2)+'\n')
    report = {'verdict': 'PASS' if all(checks.values()) else 'FAILED', 'checks': checks,
              'scope': 'Coordinated hypothesis metadata/binary tampering; no new physical bound or online acceptance.'}
    (args.output/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)
    return 0 if all(checks.values()) else 1


if __name__=='__main__':
    raise SystemExit(main())
