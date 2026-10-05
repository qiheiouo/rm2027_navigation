"""Association/provenance regressions; no A02 tracker or frontend imports."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types

import pytest

from rm_dynamic_obstacle_tracking.core import (
    Detection, MultiObjectTracker, Point2D, cluster_detections_with_members,
    cluster_points, filter_detections_near_static,
)
from rm_dynamic_obstacle_tracking.observed_members import ObservedMemberStore

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def canonical():
    intake = json.loads((ROOT / "docs/dynamic_navigation/r4_minimal_adapter_sources.json").read_text())
    path = "src/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py"
    source = subprocess.check_output(["git", "-C", str(ROOT), "show", intake["source_commit"]+":"+path])
    expected = next(s["source_sha256"] for s in intake["intake_files"] if s["source"] == path)
    assert hashlib.sha256(source).hexdigest() == expected
    module = types.ModuleType("r4_pinned_canonical_tracker")
    sys.modules[module.__name__] = module
    exec(compile(source, intake["source_commit"]+":"+path, "exec"), module.__dict__)
    return module


@pytest.mark.parametrize("anchor", ["filtered", "last_observation_cv"])
@pytest.mark.parametrize("global_assignment", [False, True])
def test_opt_in_export_preserves_pinned_tracking_and_prediction(canonical, anchor, global_assignment):
    params = dict(public_anchor_mode=anchor, use_global_assignment=global_assignment,
                  min_displacement_to_confirm=.15)
    if anchor == "last_observation_cv":
        params.update(velocity_decay_tau=0., max_prediction_speed=0.)
    old, new = canonical.MultiObjectTracker(**params), MultiObjectTracker(**params)
    epoch = 1_820_000_000_123_456_789
    for frame in range(32):
        epoch += 100_000_000
        if frame == 22:
            epoch -= 500_000_000
        positions = [(.03*frame, .05), (1.2-.02*frame, -.7)]
        if frame % 7 in (4, 5):
            positions = positions[:1]
        if frame % 2:
            positions.reverse()
        detections = [Detection(Point2D(x, y), .2, .3, 3) for x, y in positions]
        old_detections = [canonical.Detection(canonical.Point2D(x, y), .2, .3, 3) for x, y in positions]
        before = old.update(old_detections, epoch/1e9, source_stamp_ns=epoch)
        after = new.update(detections, epoch/1e9, source_stamp_ns=epoch, capture_associations=True)
        current = asdict(after)
        current.pop("associations")
        assert current == asdict(before)
        assert sorted(i for _, i in after.associations) == list(range(len(detections)))
        assert len({tid for tid, _ in after.associations}) == len(detections)


def test_cluster_and_static_filter_keep_members_without_new_clustering(canonical):
    points = [Point2D(0., 0.), Point2D(.04, .01), Point2D(.08, 0.),
              Point2D(1., 0.), Point2D(1.04, .01), Point2D(1.08, 0.)]
    pairs = cluster_detections_with_members(points, .2, 3, 1.5)
    original = canonical.cluster_points([canonical.Point2D(p.x, p.y) for p in points], .2, 3, 1.5)
    assert [asdict(d) for d, _ in pairs] == [asdict(d) for d in original]
    assert [d for d, _ in pairs] == cluster_points(points, .2, 3, 1.5)

    class Map:
        def distance_to_occupied(self, point, threshold):
            return .1 if point.x < .5 else 1.

    accepted = filter_detections_near_static([d for d, _ in pairs], Map(), .25)
    by_identity = {id(d): members for d, members in pairs}
    assert len(accepted) == 1 and accepted[0] is pairs[1][0]
    assert set(by_identity[id(accepted[0])]) == {3, 4, 5}


def tracker():
    return MultiObjectTracker(public_anchor_mode="last_observation_cv", velocity_decay_tau=0.,
                              max_prediction_speed=0., min_hits_to_confirm=1)


def frame(tracking, store, ns, sequence, x=0., ids=(3, 11, 15), empty=False):
    points = [] if empty else [Point2D(x, 0.), Point2D(x+.03, .01), Point2D(x+.06, 0.)]
    pairs = cluster_detections_with_members(points, .2, 3, 1.5)
    detections, members = [d for d, _ in pairs], [m for _, m in pairs]
    update = tracking.update(detections, ns/1e9, source_stamp_ns=ns, capture_associations=True)
    batch = store.update(update, detections, members, points, () if empty else ids, ns, sequence)
    return update, batch


def test_reordering_uses_existing_assignment_not_nearest_posthoc_match():
    tracking, store = tracker(), ObservedMemberStore()
    ns = 10_000_000_123
    a = Detection(Point2D(0., 0.), .2, .2, 1)
    b = Detection(Point2D(1., 0.), .2, .2, 1)
    first = tracking.update([a, b], ns/1e9, source_stamp_ns=ns, capture_associations=True)
    id_a, id_b = (tid for tid, _ in first.associations)
    ns += 100_000_000
    reordered = tracking.update([b, a], ns/1e9, source_stamp_ns=ns, capture_associations=True)
    batch = store.update(reordered, [b, a], [(0,), (1,)], [b.centroid, a.centroid], [80, 9], ns, 1)
    assert batch.complete
    records = {r.track_id: r for r in batch.records}
    assert records[id_a].source_member_ids == (9,)
    assert records[id_b].source_member_ids == (80,)


def test_coasting_retains_measured_epoch_and_local_members():
    tracking, store = tracker(), ObservedMemberStore()
    ns = 1_820_000_000_123_456_789
    frame(tracking, store, ns, 1)
    update, observed = frame(tracking, store, ns+100_000_000, 2, x=.1, ids=(40, 42, 57))
    coasted, coast_members = frame(tracking, store, ns+200_000_000, 3, empty=True)
    assert observed.complete and coast_members.complete
    assert coast_members.records == observed.records
    record = coast_members.records[0]
    assert record.observation_ns == ns+100_000_000
    assert record.association_sequence == 2
    assert set(record.source_member_ids) == {40, 42, 57}
    assert coasted.tracks[0].position != update.tracks[0].position


def test_reset_discards_old_members_and_track_identity():
    tracking, store = tracker(), ObservedMemberStore()
    _, first = frame(tracking, store, 10_000_000_001, 1)
    update, second = frame(tracking, store, 9_000_000_003, 2, ids=(90, 92, 98))
    assert update.time_reset and second.complete
    assert second.records[0].track_id != first.records[0].track_id
    assert second.records[0].observation_ns == 9_000_000_003
    assert set(second.records[0].source_member_ids) == {90, 92, 98}


def test_default_does_not_export_association():
    update = tracker().update([Detection(Point2D(0., 0.), .2, .2, 1)], 10.)
    assert update.associations == ()


@pytest.mark.parametrize("ids", [(3, 3, 15), (True, 11, 15), (-1, 11, 15)])
def test_invalid_original_member_identity_is_not_a_complete_shape(ids):
    _, batch = frame(tracker(), ObservedMemberStore(), 10_000_000_001, 1, ids=ids)
    assert not batch.complete and batch.reason == "members_invalid" and not batch.records


def test_point_budget_never_truncates_a_shape():
    _, batch = frame(tracker(), ObservedMemberStore(max_points=2), 10_000_000_001, 1)
    assert not batch.complete and batch.reason == "members_budget" and not batch.records


def test_budget_includes_retained_coasting_members():
    tracking, store = tracker(), ObservedMemberStore(max_points=5)
    _, first = frame(tracking, store, 10_000_000_001, 1)
    assert first.complete
    update, batch = frame(tracking, store, 10_100_000_001, 2, x=2.)
    assert len(update.tracks) == 2
    assert not batch.complete and batch.reason == "members_budget" and not batch.records


def test_missing_assignment_fails_closed_and_keeps_public_result():
    tracking, store = tracker(), ObservedMemberStore()
    detection = Detection(Point2D(0., 0.), .2, .2, 1)
    update = tracking.update([detection], 10., source_stamp_ns=10_000_000_000)
    batch = store.update(update, [detection], [(0,)], [detection.centroid], [7], 10_000_000_000, 1)
    assert not batch.complete and batch.reason == "association_incomplete"
    assert len(update.tracks) == 1


def test_invalid_receipt_clears_private_cache_instead_of_reusing_old_members():
    tracking, store = tracker(), ObservedMemberStore()
    _, first = frame(tracking, store, 10_000_000_001, 1)
    assert first.complete
    _, duplicate = frame(tracking, store, 10_100_000_001, 1, empty=True)
    assert duplicate.reason == "source_identity"
    _, later = frame(tracking, store, 10_200_000_001, 2, empty=True)
    assert not later.complete and later.reason == "members_missing"


def test_nonfinite_member_is_not_measured_support():
    tracking, store = tracker(), ObservedMemberStore()
    detection = Detection(Point2D(0., 0.), .2, .2, 1)
    update = tracking.update([detection], 10., source_stamp_ns=10_000_000_000, capture_associations=True)
    batch = store.update(update, [detection], [(0,)], [Point2D(float("nan"), 0.)], [7],
                         10_000_000_000, 1)
    assert not batch.complete and batch.reason == "members_invalid"
