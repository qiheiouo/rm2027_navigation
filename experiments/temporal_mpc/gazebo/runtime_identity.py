#!/usr/bin/env python3
"""Pre-start runtime provenance; no device access or network."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import sys
import time

if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('output');args=p.parse_args()
    root=Path(__file__).resolve().parents[3]
    files=[root/'build/temporal_mpc_ros2/frontend',root/'build/temporal_mpc_ros2/install/rm_temporal_mpc_controller/lib/libtemporal_mpc_controller.so']
    target=Path(args.output)
    if target.exists():raise SystemExit('refuse identity overwrite')
    result=dict(captured_before_process_start=True,wall_ns=time.monotonic_ns(),python=sys.version,
                kernel=platform.release(),cpu_affinity=sorted(os.sched_getaffinity(0)),
                ros_distro=os.environ.get('ROS_DISTRO'),rmw=os.environ.get('RMW_IMPLEMENTATION','default'),
                dependencies={name:importlib.metadata.version(name) for name in ('numpy','scipy','osqp')},
                binaries={str(f.relative_to(root)):dict(bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in files},
                scope='Actual files before launch. Container image identity is recorded externally; no hard real-time guarantee.')
    target.write_text(json.dumps(result,indent=2)+'\n')
