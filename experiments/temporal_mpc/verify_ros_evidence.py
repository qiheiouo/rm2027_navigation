#!/usr/bin/env python3
"""Read-only consistency/causality audit of real recorded Nav2 integration.

No simulator truth or physical navigation verdict can be recovered here.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def verify(directory):
    records=[json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
    summary=json.loads((directory/'summary.json').read_text())
    commands=[p for p in records if p['kind']=='command']
    health=[p for p in records if p['kind']=='health']
    selection=[p for p in records if p['kind']=='selection']
    solver=[p for p in records if p['kind']=='solver']
    assert len(commands)==summary['command_count'] and commands
    velocities=np.asarray([p['v'] for p in commands]);assert np.isfinite(velocities).all()
    assert np.max(np.abs(velocities[:,0]))<=.80001 and np.max(np.abs(velocities[:,1]))<=.50001
    assert np.max(np.abs(velocities[:,2]))<1e-6
    primary=[p['wall_s'] for p in commands if not p['stage'].startswith('direct_') and p['stage'] not in ('final_cancel','finished')]
    direct=[p['wall_s'] for p in commands if p['stage'].startswith('direct_')]
    gaps=np.r_[np.diff(primary),np.diff(direct)]
    assert len(gaps) and min(gaps)>0 and max(gaps)<.075
    assert math.isclose(max(gaps),summary['max_active_command_gap_s'],abs_tol=1e-9)
    execution=[p for p in health if p['executed']]
    assert execution and max(p['elapsed_s'] for p in execution)<.01
    assert any(p['stage']=='explicit_MPC' and p['ready'] for p in execution)
    assert max(p['iterations'] for p in solver)<=400
    assert any(p['id']=='FollowPathTemporalMPC' for p in selection)
    assert any(p['stage']=='explicit_MPPI' and p['id']=='FollowPathMPPI' for p in selection)
    audit=[]
    for case in summary['fault_cases']:
        stage='fault_'+case['fault'];items=[p for p in records if p['stage']==stage]
        assert items
        start=items[0]['wall_s']
        before=[p for p in selection if p['wall_s']<start]
        assert before and before[-1]['id']=='FollowPathTemporalMPC',case['fault']
        accepted=[p for p in execution if p['stage']=='rearm_'+case['fault'] and p['ready']]
        assert accepted and 0<=start-accepted[-1]['wall_s']<.15,case['fault']
        assert any(p['kind']=='selection' and p['id']=='FollowPathMPPI' for p in items)
        assert any(p['kind']=='health' and not p['ready'] for p in items)
        assert any(p['kind']=='command' for p in items)
        assert 0<case['fallback_latency_s']<.6
        audit.append({'fault':case['fault'],'MPC_active_before':True,'new_fallback_transition':True,'commands_continue':True})
    killed=[p for p in records if p['kind']=='worker_killed']
    assert len(killed)==1
    after=[p for p in commands if p['stage']=='direct_worker_killed_brake']
    assert len(after)>=15 and max(abs(v) for v in after[-1]['v'])<1e-6
    assert any(p['stage']=='direct_worker_killed_brake' and not p['ready'] for p in execution)
    assert summary['statuses']['navigation_result']==5 # intentional cancel, not navigation success
    assert summary['publisher_names']==['controller_server'] and len(summary['publisher_gids'])==1
    log=(directory/'controller.log').read_text()
    for needle in ('Created controller : FollowPathMPPI','Created controller : FollowPathTemporalMPC','Passing new path to controller.'):
        assert needle in log
    assert not summary['dynamic_navigation_acceptance']
    return {'recorded_stream_audit_pass':True,'fault_cases':audit,'commands':len(commands),
            'active_max_gap_s':float(max(gaps)),'worker_exit_stop':True,
            'unique_publisher_scope':'harness endpoint monitoring summary plus actual dual-class load log; not independently replayable DDS graph',
            'physical_navigation_acceptance':False}


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=verify(a.directory);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
