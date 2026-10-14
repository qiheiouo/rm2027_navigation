"""Small transparent reader for losslessly compressed owned diagnostic streams."""
import pathlib,gzip,json
COHORTS=['before','diag','repair','heldout_repair','crash_debug','crash_alignment','repair_abi','heldout_abi','heldout_abi_retry','repair_odom','heldout_odom']

def jsonlines(path):
 path=pathlib.Path(path);actual=path if path.exists() else pathlib.Path(str(path)+'.gz');result=[]
 if not actual.exists():return result
 opener=gzip.open if actual.suffix=='.gz' else open
 with opener(actual,'rt') as stream:
  for line in stream:
   try:result.append(json.loads(line))
   except json.JSONDecodeError:pass # original bytes remain archived; counts are reported separately
 return result
