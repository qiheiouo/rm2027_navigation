"""A predicted box hit alone cannot label a disagreeing measured return."""
import importlib.util
from pathlib import Path
import sys

tools=Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0,str(tools))
spec=importlib.util.spec_from_file_location('surface_audit',tools/'audit_observed_surface_geometry.py')
audit=importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_disagreeing_return_is_unknown_even_when_nearest_box_is_actor():
    label=audit.label_return(1.1,(1.,'actor'),.03)
    assert label['status']=='RANGE_MISMATCH_UNKNOWN' and label['nearest_box']=='actor'


def test_no_predicted_hit_does_not_discard_observed_endpoint():
    assert audit.label_return(1.,None,.03)['status']=='NO_EXPECTED_BOX'


def test_range_agreement_labels_support_only_matching_nearest_box():
    assert audit.label_return(1.01,(1.,'actor'),.03)['status']=='actor'
    assert audit.label_return(1.01,(1.,'static'),.03)['status']=='static'
