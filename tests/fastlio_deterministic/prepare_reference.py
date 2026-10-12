"""Generate a legal-storage numerical reference from the fixed original Git body.

Only its two clear calls become resize(input count). Geometry, selection,
compaction order, residual and Jacobian expressions stay byte-identical.
This is an explicit test control, not the undefined original executable.
"""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

repo, destination = map(Path, sys.argv[1:])
pin = 'e7864a63a7e2a1ad62ec8fd75fcb9149db08e321'
source = subprocess.check_output(['git', '-C', str(repo), 'show', pin+':src/laserMapping_bundle.cpp']).decode()
start = source.index('void h_share_model(')
end = source.index('\nclass LaserMappingBundleNode', start)
method = source[start:end].strip()
assert method.count('laserCloudOri->clear();') == 1
assert method.count('corr_normvect->clear();') == 1
reference = method.replace('void h_share_model(', 'void reference_h_share_model(', 1)
reference = reference.replace('laserCloudOri->clear();', 'laserCloudOri->resize(feats_down_size);')
reference = reference.replace('corr_normvect->clear();', 'corr_normvect->resize(feats_down_size);')
destination.write_text(reference+'\n')
destination.with_suffix('.json').write_text(json.dumps({
    'pin': pin, 'original_method_sha256': hashlib.sha256(method.encode()).hexdigest(),
    'test_control': 'Rename function; replace ONLY two clear calls with resize(input count). No math or threshold changes.',
}, indent=2)+'\n')
