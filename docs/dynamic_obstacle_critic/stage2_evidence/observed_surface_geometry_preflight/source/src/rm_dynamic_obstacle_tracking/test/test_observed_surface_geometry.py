"""Endpoint/identity and blind-region counterexamples; no hidden geometry prior."""
import math
from dataclasses import replace
import pytest
from rm_dynamic_obstacle_tracking.observed_surface_geometry import SurfaceConfig,extract_observed_surface,propagate_nominal


def extract(points,indices=None,ranges=None,config=SurfaceConfig()):
    indices=list(range(len(points))) if indices is None else indices
    ranges=[1.]*max(1,1+max(indices,default=-1)) if ranges is None else ranges
    return extract_observed_surface(points,indices,ranges=ranges,source_stamp_ns=19_000_000_001,range_min=.1,range_max=8,config=config)


def test_single_observation_stays_a_point_with_hidden_unknown():
    s=extract([(1,2)])
    assert s.elements[0].start==s.elements[0].end==(1,2)
    assert s.elements[0].source_indices==(0,) and s.hidden_space_status=='UNKNOWN'
    assert not s.support_is_certified


def test_l_shape_keeps_both_faces_without_closing_hidden_back():
    points=[(1,0),(1,.1),(1,.2),(1.1,.2),(1.2,.2)]
    s=extract(points)
    assert len(s.elements)==4
    assert {e.start for e in s.elements}|{e.end for e in s.elements}==set(points)
    assert all(e.interior_evidence=='LOCAL_CONTINUITY_ASSUMPTION' for e in s.elements)
    assert s.elements[-1].end==points[-1] and s.elements[0].start==points[0]


def test_missing_beam_inside_one_spatial_cluster_is_not_bridged():
    s=extract([(1,0),(1,.1),(1,.2)],[2,3,5])
    assert [e.source_indices for e in s.elements]==[(2,3),(5,)]
    assert s.breaks[0][2]==('source_index_gap',)


def test_invalid_unselected_return_cannot_become_free_or_a_chord():
    s=extract([(1,0),(1,.1)],[0,2],[1,math.nan,1])
    assert len(s.elements)==2 and all(e.kind=='observed_point' for e in s.elements)


def test_depth_jump_and_disconnected_surfaces_keep_all_endpoints():
    s=extract([(1,0),(1,.1),(2,.1)],[0,1,2],[1,1,2])
    assert [e.source_indices for e in s.elements]==[(0,1),(2,)]
    assert s.breaks[0][2]==('range_discontinuity','spatial_discontinuity')


def test_source_order_permutation_does_not_change_geometry():
    a=extract([(1,0),(1,.1),(1,.2)])
    b=extract([(1,.2),(1,0),(1,.1)],[2,0,1])
    assert a==b


def test_scan_seam_is_not_closed():
    s=extract([(1,0),(1,.1)],[0,719])
    assert all(e.kind=='observed_point' for e in s.elements)


def test_pose_frame_rigid_transform_preserves_topology_and_measurements():
    points=[(1,0),(1,.1),(1,.2)];a=extract(points);phi=.7;c,s=math.cos(phi),math.sin(phi)
    move=lambda p:(100+c*p[0]-s*p[1],-20+s*p[0]+c*p[1])
    b=extract([move(p) for p in points]);assert [e.source_indices for e in a.elements]==[e.source_indices for e in b.elements]
    assert [e.start for e in b.elements]==[move(e.start) for e in a.elements]


def test_future_validity_includes_source_age_and_never_clears_unknown():
    s=extract([(1,0),(1,.1)]);p=propagate_nominal(s,(.2,-.1),source_age_seconds=.2,future_seconds=.3)
    assert p.status=='NOMINAL_SHORT_CV_UNCERTIFIED' and p.elements[0].start==(1.1,-.05)
    p=propagate_nominal(s,(.2,-.1),source_age_seconds=.200001,future_seconds=.3)
    assert p.status=='EXPIRED_GEOMETRY_UNKNOWN_TREND_ONLY' and p.elements
    assert not p.support_is_certified and p.hidden_space_status=='UNKNOWN' and not p.rotation_modeled


def test_empty_scan_stays_unknown():
    p=propagate_nominal(extract([]),(0,0),source_age_seconds=0,future_seconds=0)
    assert not p.elements and p.status=='NO_OBSERVATION_UNKNOWN'


@pytest.mark.parametrize('indices',[[0,0],[True,1],[-1,1]])
def test_bad_source_identity_rejects_whole_cluster(indices):
    with pytest.raises(ValueError):extract([(1,0),(1,.1)],indices,[1,1])


def test_incomplete_budget_is_rejected_instead_of_truncating():
    with pytest.raises(ValueError,match='budget'):extract([(1,0),(1,.1),(1,.2)],config=replace(SurfaceConfig(),max_elements=1))


def test_nonfinite_selected_return_is_not_occupied_evidence():
    with pytest.raises(ValueError,match='member'):extract([(1,0)],[0],[math.inf])


@pytest.mark.parametrize('age,future',[(-.001,.1),(.1,-.1),(math.nan,0)])
def test_bad_time_does_not_get_valid_future_label(age,future):
    with pytest.raises(ValueError):propagate_nominal(extract([(1,0)]),(0,0),source_age_seconds=age,future_seconds=future)
