"""A25 bounded baseline clearance calibration; no R4 or runtime implementation."""
import json,pathlib,shutil,statistics,subprocess,sys
import yaml
from analyze import trial
root=pathlib.Path(__file__).resolve().parents[2]
command=sys.argv[1];out=pathlib.Path(sys.argv[2]).resolve()
def run(repeat,mode,profile):
    directory=out/'runs'/f'S2_{mode}_{repeat:02d}'
    if not directory.exists():
        with (out/f'{directory.name}_runner.log').open('w') as log:
            subprocess.run(['bash',str(root/'experiments/r4_gazebo_comparison/run.sh'),str(out),f'S2:{mode}:{repeat}',profile],cwd=root,stdout=log,stderr=subprocess.STDOUT,check=False)
    if not (directory/'events.json').exists(): raise RuntimeError(f'incomplete run preserved: {directory.name}')
    result=trial(directory)
    print(directory.name,result['finished'],result.get('min_dynamic_clearance_m'),flush=True)
    if result['startup'] or not result['success']: raise RuntimeError(f'first failure preserved; classify before continuation: {directory.name}')
    return result
def write(name,factor):
    config=yaml.safe_load((out/'assets/matched_B0_nav2.yaml').read_text())
    config['local_costmap']['local_costmap']['ros__parameters']['inflation_layer'].update(inflation_radius=.8,cost_scaling_factor=factor)
    (out/f'assets/{name}_B0_nav2.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
if command=='prepare':
    previous=root/'build/r4_matched_comparison_20261006';out.mkdir(exist_ok=False)
    for name in ('assets','research_install'): shutil.copytree(previous/name,out/name,symlinks=True)
    shutil.copy2(previous/'assets/matched_R4_nav2.yaml',out/'assets/pareto_R4_nav2.yaml')
    plan=dict(reference_commit='1c0b6d73',algorithm_commit='ccd3eac4',scene='S2',target_median_clearance_m=[.28,.32],calibration_candidates_max=4,calibration_trials_per_candidate=3,baseline_local_inflation_radius=.8,cost_scaling_factor_initial=3.,cost_scaling_factor_bounds=[.75,6.],finite_initial_per_mode=5,finite_max_per_mode=10,efficiency_relative_threshold=.1,paired_win_threshold_initial=4,paired_win_threshold_max=8,worst_clearance_tolerance_m=.05,r4_changed=False,baseline_allowed_changes=['local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius','local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor'])
    (out/'protocol.json').write_text(json.dumps(plan,indent=2)+'\n')
elif command=='calibrate':
    if (out/'freeze.json').exists(): raise RuntimeError('already frozen; no further calibration')
    factor=3.;lo=.75;hi=6.;reports=[]
    for index in range(1,5):
        profile=f'calibration_pareto_{index}';write(profile,factor)
        results=[run((index-1)*3+k,'B0',profile) for k in range(1,4)]
        median=statistics.median(r['min_dynamic_clearance_m'] for r in results)
        report=dict(candidate=index,profile=profile,cost_scaling_factor=factor,inflation_radius=.8,median_clearance_m=median,clearance_m=[r['min_dynamic_clearance_m'] for r in results],runs=[r['run'] for r in results])
        reports.append(report);(out/'calibration.json').write_text(json.dumps(reports,indent=2)+'\n')
        print('candidate',index,'factor',factor,'clearance median',median,flush=True)
        if .28<=median<=.32:
            shutil.copy2(out/f'assets/{profile}_B0_nav2.yaml',out/'assets/pareto_B0_nav2.yaml')
            (out/'freeze.json').write_text(json.dumps(report,indent=2)+'\n')
            print('frozen immediately on target clearance',flush=True);break
        if median<.28: hi=factor
        else: lo=factor
        factor=(lo+hi)/2
    else: print('Modify: bounded calibration did not reach target',flush=True)
elif command=='batch':
    assert (out/'freeze.json').exists(),'clearance target must be frozen first'
    count=int(sys.argv[3]) if len(sys.argv)>3 else 5;assert count in (5,10)
    if count==10:
        decision=json.loads((out/'pareto_audit.json').read_text())
        assert decision['verdict']=='Modify' and decision['extension_informative'],'extension must address a recorded decision uncertainty'
    schedule=[f'S2:{mode}:{repeat}' for repeat in range(101,101+count) for mode in (('B0','R4') if repeat%2 else ('R4','B0'))]
    replacements=json.loads((out/'replacement_runs.json').read_text()) if (out/'replacement_runs.json').exists() else {}
    schedule=[replacements.get(spec,spec) for spec in schedule]
    (out/'schedule.json').write_text(json.dumps(schedule,indent=2)+'\n')
    for spec in schedule:
        _,mode,repeat=spec.split(':');run(int(repeat),mode,'pareto')
    print('fixed-profile finite comparison complete',flush=True)
else: raise ValueError(command)
