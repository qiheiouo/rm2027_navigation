"""Associate native controller rate warnings with monotonic observer clocks."""
import pathlib,json,re,sys,bisect
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'experiments/stvl_c83_endtoend'));from causal_replay import lines
sys.path.insert(0,str(ROOT/'experiments/stvl_execution_realtime'));from mapping import rows
out=[]
for p in sorted((ROOT/'build/c83_cache').glob('*/runs/*')):
 if not (p/'events.json').exists():continue
 a=lines(p/'paired_timing.jsonl');receipt=rows(p/'command_receipts.csv');sys_t=[int(x['system_ns']) for x in receipt];warnings=[]
 for line in (p/'launch.log').read_text().splitlines():
  if 'Control loop missed its desired rate' not in line:continue
  m=re.search(r'\[WARN\] \[(\d+)\.(\d+)\]',line)
  if not m:continue
  system=int(m[1])*10**9+int(m[2].ljust(9,'0')[:9]);idx=min(range(len(sys_t)),key=lambda i:abs(sys_t[i]-system));r=receipt[idx];steady=int(r['steady_ns'])+system-int(r['system_ns']);near=min(a,key=lambda x:abs(x['wrapper_pre_return_steady_ns']-steady))
  warnings.append(dict(system_ns=system,estimated_steady_ns=steady,nearest_cycle_id=near['cycle_id'],cycle_necessary_ms=near['T_total_necessary_boundary_ms'],cycle_wrapper_ms=near['T_actual_wrapper_pre_return_ms'],wrapper_return_to_warning_ms=(steady-near['wrapper_pre_return_steady_ns'])/1e6))
 out.append(dict(cohort=p.parent.parent.name,warnings=warnings,warning_count=len(warnings),scope='native server WallRate warnings; not exact outer-loop duration or hardware deadline rate'))
(ROOT/'experiments/stvl_c83_endtoend/evidence/native_rate_warnings.json').write_text(json.dumps(out,indent=2)+'\n');print(out)
