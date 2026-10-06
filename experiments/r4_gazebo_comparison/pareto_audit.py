"""Offline A25 preregistered Pareto decision and parameter checks."""
import json,pathlib,sys
import yaml
root=pathlib.Path(sys.argv[1]);reference=root.parent/'r4_matched_comparison_20261006'
def flatten(value,prefix=''):
    if isinstance(value,dict):
        return {k:v for name,child in value.items() for k,v in flatten(child,prefix+'.'+name if prefix else name).items()}
    return {prefix:value}
def diff(a,b):
    a=flatten(a);b=flatten(b)
    return {k:{'before':a.get(k),'after':b.get(k)} for k in sorted(a.keys()|b.keys()) if a.get(k)!=b.get(k)}
plan=json.loads((root/'protocol.json').read_text());summary=json.loads((root/'summary.json').read_text())
configs={mode:yaml.safe_load((root/f'assets/pareto_{mode}_nav2.yaml').read_text()) for mode in ('B0','R4')}
changes=diff(yaml.safe_load((reference/'assets/matched_B0_nav2.yaml').read_text()),configs['B0'])
assert set(changes)<=set(plan['baseline_allowed_changes']),changes
assert (root/'assets/pareto_R4_nav2.yaml').read_bytes()==(reference/'assets/matched_R4_nav2.yaml').read_bytes(),'R4 A24 configuration changed'
errors=[]
for p in sorted((root/'runs').iterdir()):
    if not (p/'manifest.json').exists(): continue
    m=json.loads((p/'manifest.json').read_text())
    if m['phase']!='finite': continue
    differences=diff(configs[m['mode']],yaml.safe_load((p/'nav2.yaml').read_text()))
    unexpected={k:v for k,v in differences.items() if k not in ('controller_server.ros__parameters.FollowPath.research_mode','controller_server.ros__parameters.FollowPath.research_log')}
    if unexpected: errors.append({'run':p.name,'unexpected':unexpected})
assert not errors,errors
groups={g['mode']:g for g in summary['groups'] if g['scene']=='S2'};b=groups['B0'];r=groups['R4'];n=b['n'];assert n==r['n'] and n in (5,10)
runs={a['run']:a for a in summary['runs'] if a['phase']=='finite' and not a['startup']}
replacements=json.loads((root/'replacement_runs.json').read_text()) if (root/'replacement_runs.json').exists() else {}
pairs=[]
for repeat in range(101,101+n):
    br=runs[replacements.get(f'S2:B0:{repeat}',f'S2:B0:{repeat}').replace(':','_')]
    rr=runs[replacements.get(f'S2:R4:{repeat}',f'S2:R4:{repeat}').replace(':','_')]
    pairs.append(dict(baseline_run=br['run'],r4_run=rr['run'],baseline_arrival_s=br['arrival_s'],r4_arrival_s=rr['arrival_s'],baseline_clearance_m=br['min_dynamic_clearance_m'],r4_clearance_m=rr['min_dynamic_clearance_m'],baseline_wait_s=br['wait_s'],r4_wait_s=rr['wait_s'],baseline_longest_stall_s=br['longest_stall_s'],r4_longest_stall_s=rr['longest_stall_s'],baseline_reversals=br['forward_reversals'],r4_reversals=rr['forward_reversals'],baseline_backtrack_m=br['maximum_backtrack_m'],r4_backtrack_m=rr['maximum_backtrack_m']))
band=all(.28<=g['min_clearance_m']['p50']<=.32 for g in (b,r))
worst_gap=abs(b['min_clearance_m']['min']-r['min_clearance_m']['min']);worst_ok=worst_gap<=.05
success=all(g['success']==n and g['contacts']==0 for g in (b,r))
time_ratio=b['arrival_s']['p50']/r['arrival_s']['p50'] if b['arrival_s'] and r['arrival_s'] else None
required=4 if n==5 else 8
faster_b=sum(p['baseline_arrival_s'] is not None and p['r4_arrival_s'] is not None and p['baseline_arrival_s']<p['r4_arrival_s'] for p in pairs)
faster_r=sum(p['baseline_arrival_s'] is not None and p['r4_arrival_s'] is not None and p['r4_arrival_s']<p['baseline_arrival_s'] for p in pairs)
def stability_bad(p,mode):
    return p[f'{mode}_reversals']>=2 or p[f'{mode}_backtrack_m']>=.05
bad={mode:sum(stability_bad(p,mode) for p in pairs) for mode in ('baseline','r4')}
stall_b=sum(p['baseline_longest_stall_s']-p['r4_longest_stall_s']>=1 for p in pairs)
stall_r=sum(p['r4_longest_stall_s']-p['baseline_longest_stall_s']>=1 for p in pairs)
minimum_stability_gap=2 if n==5 else 4
r_stabler=bad['baseline']-bad['r4']>=minimum_stability_gap or stall_b>=minimum_stability_gap
b_stabler=bad['r4']-bad['baseline']>=minimum_stability_gap or stall_r>=minimum_stability_gap
verdict='Modify';reason='Finite clearance/success/efficiency evidence does not establish the preregistered Pareto decision.'
if success and band and worst_ok:
    if (time_ratio<=.9 and faster_b>=required and not r_stabler):
        verdict='Stop';reason='Baseline is >=10% faster with paired agreement at matched clearance; R4 has no stability advantage.'
    elif (time_ratio>=1/.9 and faster_r>=required) or (r_stabler and not b_stabler and time_ratio>=1/1.1):
        verdict='Go';reason='R4 is materially faster or more stable without material efficiency loss at matched clearance.'
if not band: reason='Independent finite sample did not retain both median clearances in the target band; do not retune.'
elif not worst_ok: reason='Median clearance matches, but >5cm worst-clearance gap prevents claiming equal safety.'
extension=n==5 and verdict=='Modify' and success and worst_ok and all(.26<=g['min_clearance_m']['p50']<=.34 and g['min_clearance_m']['min']<=.32 and g['min_clearance_m']['max']>=.28 for g in (b,r))
report=dict(verdict=verdict,reason=reason,n_per_mode=n,finite_target_band_pass=band,worst_clearance_difference_m=worst_gap,worst_clearance_comparable=worst_ok,all_success_zero_contact=success,baseline_over_r4_arrival_ratio=time_ratio,baseline_faster_pairs=faster_b,r4_faster_pairs=faster_r,pair_agreement_required=required,baseline_stabler=b_stabler,r4_stabler=r_stabler,stability_bad_runs=bad,baseline_extra_stall_pairs=stall_b,r4_extra_stall_pairs=stall_r,extension_informative=extension,baseline_changes=changes,r4_a24_parameters_byte_equal=True,trial_config_errors=errors,freeze=json.loads((root/'freeze.json').read_text()),pairs=pairs,limits='Gazebo S2 only; matched sampled mechanical clearance, not identical risk probabilities, true random seeds or a complete Pareto frontier.')
(root/'pareto_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
