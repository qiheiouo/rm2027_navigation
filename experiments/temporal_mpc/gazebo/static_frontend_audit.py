#!/usr/bin/env python3
"""Actual T-DT topology/SFC checks, separate from dynamic performance."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from temporal_mpc.frontend import prepare_route
from temporal_mpc.contracts import ContractError
from static_grid import grid_for

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('frontend');parser.add_argument('output');args=parser.parse_args()
    results=[]
    for scenario in ('crossing','course'):
        grid=grid_for(scenario);costs=np.where(grid>=65,254,0).astype(np.uint8)
        try:
            route=prepare_route(args.frontend,costs,.05,(-1.,-3.),(0.,0.),(5.6,0.))
            result=dict(scenario=scenario,route_certified=True,path=route.path.tolist(),
                        centre_y_bounds=[float(np.min(route.bounds[:,2])),float(np.max(route.bounds[:,3]))],
                        path_y_extreme=float(np.max(np.abs(route.anchors[:,1]))),
                        static_grid_sha256=hashlib.sha256(grid.tobytes()).hexdigest(),document=route.document)
        except ContractError as error:result=dict(scenario=scenario,route_certified=False,reason=str(error))
        if scenario=='course':
            try:
                prepare_route(args.frontend,costs,.05,(-1.,-3.),(0.,0.),(5.6,0.),supplied_path=[(0.,0.),(5.6,0.)])
                result['unsafe_gap_shortcut_rejected']=False
            except ContractError:result['unsafe_gap_shortcut_rejected']=True
        results.append(result)
    Path(args.output).write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in r.items() if k not in ('document','path')} for r in results],indent=2))
