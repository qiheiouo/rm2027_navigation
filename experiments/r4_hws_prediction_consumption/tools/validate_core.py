"""Run explicit-source offline probes and write their complete evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import scipy
import osqp
from r4_hws.probes import prepared_open_route, wait_release_probe
from r4_hws.fault_probe import run_fault_probe


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frontend', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    frontend_hash = hashlib.sha256(args.frontend.read_bytes()).hexdigest()
    route = prepared_open_route(args.frontend)
    evidence = dict(scope='offline_contract_and_numerical_checks_only',
                    python=sys.version, numpy=np.__version__, scipy=scipy.__version__, osqp=osqp.__version__,
                    frontend=dict(path=str(args.frontend.resolve()), sha256=frontend_hash, plan_id=route.plan_id),
                    wait_release=wait_release_probe(route), independent_output=run_fault_probe())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(evidence, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: {a: b for a, b in v.items() if a != 'rows'} for k, v in evidence.items()
                      if k in ('wait_release', 'independent_output')}, indent=2))


if __name__ == '__main__': main()
