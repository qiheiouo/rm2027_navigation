import hashlib
import json
from pathlib import Path
import shutil
import sys
import matplotlib
import numpy
import yaml

repo = Path('/home/qihei/rm2027_navigation')
base = Path('/tmp/rm_dynamic_critic_v1')
out = repo/'docs/dynamic_obstacle_critic/stage2_evidence/ground_robot_extent_prior'
out.mkdir()

def copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise FileExistsError(target)
    shutil.copyfile(source, target)

package = repo/'src/rm_dynamic_obstacle_critic'
modules = ['robot_extent_prior', 'test_robot_extent_prior', 'audit_ground_extent_prior',
           'plot_ground_extent_prior', 'replay_ground_extent_prior', 'audit_consumed_cv_support',
           'analyze_trial', 'audit_scan_geometry', 'dynamic_consumption_io', 'trial_io', 'fixture_robot_geometry']
for name in modules:
    copy(package/'tools'/(name+'.py'), out/'source/tools'/(name+'.py'))
copy(package/'config/ground_robot_extent_prior_offline.yaml', out/'source/config/ground_robot_extent_prior_offline.yaml')
copy(base/'ground_robot_extent_prior_analysis.json', out/'analysis.json')
for path in (base/'ground_extent_prior_figures_retry').iterdir():
    copy(path, out/'figures'/path.name)
for name in ['ground_robot_extent_prior_analysis.log', 'ground_robot_extent_prior_tests_final.log',
             'ground_extent_prior_tests_with_overflow.log', 'ground_extent_prior_plot.log',
             'ground_extent_prior_plot_retry.log', 'ground_extent_prior_legacy_cv_audit.log',
             'ground_extent_prior_legacy_cv_regression.json']:
    copy(base/name, out/'verification'/name)
copy(Path(__file__), out/'verification/freeze_ground_extent_prior.py')
source = out.parent/'gazebo_dynamic_consumption_evidence'
manual = out.parent/'robot_extent_manual_review'
provenance = {'source_archive': '../gazebo_dynamic_consumption_evidence',
    'source_manifest_sha256': hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest(),
    'manual_review_archive': '../robot_extent_manual_review',
    'manual_review_manifest_sha256': hashlib.sha256((manual/'manifest.json').read_bytes()).hexdigest(),
    'runtime_source_checkpoint': '9f73d72', 'preregistration_checkpoint': '920ce39',
    'scope': 'All four ground robot classes, offline only, 2026 conditional reference.',
    'runtime_parameters_changed': False, 'tf_or_ros_interfaces_changed': False,
    '2027_applicability_verified': False, 'original_task_and_raw203_verdict': 'FAILED',
    'visual_review': 'Final PNG inspected; labels, ranges and coverage counts are readable.',
    'negative_or_failed_invocations': ['Matplotlib 3.6.3 rejected tick_labels; fixed to supported labels, original log retained.']}
(out/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
(out/'verification/environment.json').write_text(json.dumps({'python': sys.version, 'matplotlib': matplotlib.__version__,
    'numpy': numpy.__version__, 'pyyaml': yaml.__version__, 'geometry_seed': 20261002,
    'geometry_cases': 5000, 'roundoff_allowance_unit_test_only': 1e-13}, indent=2)+'\n')
files = {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest()
         for path in sorted(out.rglob('*')) if path.is_file()}
(out/'manifest.json').write_text(json.dumps({'scope': 'Conditional offline current extent and future CV support, no online promotion.', 'files': files}, indent=2)+'\n')
print(json.dumps({'files': len(files), 'bytes': sum(path.stat().st_size for path in out.rglob('*') if path.is_file()),
                  'manifest_sha256': hashlib.sha256((out/'manifest.json').read_bytes()).hexdigest()}))
