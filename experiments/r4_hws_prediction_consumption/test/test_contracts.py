from dataclasses import FrozenInstanceError
import pytest
from r4_hws.contracts import ContractError, ReceiptOrder
from r4_hws.observed_shape import ObservedShapeTracker
from r4_hws.tracker_core import Point2D
from r4_hws.soft_field import TemporalSoftField


def observed(x=1., y=0.):
    return [Point2D(x-.08, y), Point2D(x, y+.02), Point2D(x+.08, y)]


def test_deep_freeze_and_next_cycle_prediction(make_cycle):
    tracker = ObservedShapeTracker()
    public, shapes, _ = tracker.update(observed(), 1_000_000_000, 1)
    state = [0., 0., 0., 0., 0., 0.]
    snapshot = make_cycle(public, shapes, state=state)
    public['tracks'][0]['position'] = (99., 99.)
    shapes['tracks'][0]['cells'] = []
    state[0] = 99.
    assert snapshot.state[0] == 0. and snapshot.tracks[0].anchor[0] == 1.
    assert snapshot.tracks[0].cells and len(snapshot.digest) == 64
    with pytest.raises(FrozenInstanceError):
        snapshot.epoch_ns = 2
    with pytest.raises(TypeError):
        snapshot.tracks[0].cells[0][0] = 2
    newer_public, newer_shapes, _ = tracker.update(observed(1.1), 1_100_000_000, 2)
    newer = make_cycle(newer_public, newer_shapes, epoch=1_100_000_000)
    assert newer.tracks[0].anchor != snapshot.tracks[0].anchor


@pytest.mark.parametrize('fault', ['source_stale', 'observation_stale', 'source_future', 'wrong_frame',
                                  'shape_epoch', 'shape_anchor', 'missing_shape', 'duplicate_id',
                                  'not_complete', 'wrong_cv_display', 'missing_field', 'missing_cells',
                                  'state_TTL', 'TF_epoch', 'last_command_TTL', 'yaw_rate', 'nan', 'track_budget'])
def test_fail_closed_contract(make_cycle, fault):
    public, shapes, _ = ObservedShapeTracker().update(observed(), 1_000_000_000, 1)
    extra = {}
    if fault == 'source_stale': extra['epoch'] = 1_400_000_001
    if fault == 'observation_stale': public['tracks'][0]['observation_ns'] = 599_999_999
    if fault == 'source_future': extra['epoch'] = 999_999_999
    if fault == 'wrong_frame': public['frame'] = 'odom'
    if fault == 'shape_epoch': shapes['source_ns'] += 1
    if fault == 'shape_anchor': shapes['tracks'][0]['anchor'] = (1.01, 0.)
    if fault == 'missing_shape': shapes['tracks'] = []
    if fault == 'duplicate_id':
        public['tracks'] *= 2; public['total_track_count'] = 2; shapes['tracks'] *= 2
    if fault == 'not_complete': public['complete'] = False
    if fault == 'wrong_cv_display': public['tracks'][0]['prediction'] = [(9., 9.)]*15
    if fault == 'missing_field': del public['schema']
    if fault == 'missing_cells': shapes['tracks'][0]['cells'] = []
    if fault == 'state_TTL': extra['state_ns'] = 849_999_999
    if fault == 'TF_epoch': extra['tf_ns'] = 999_999_999
    if fault == 'last_command_TTL': extra['last_command_ns'] = 849_999_999
    if fault == 'yaw_rate': extra['state'] = (0., 0., 0., 0., 0., .1)
    if fault == 'nan': extra['state'] = (float('nan'), 0., 0., 0., 0., 0.)
    if fault == 'track_budget':
        public['tracks'] *= 5; shapes['tracks'] *= 5; public['total_track_count'] = 5
    with pytest.raises(ContractError):
        make_cycle(public, shapes, **extra)


def test_receipt_order_invalidates_usable_cache(make_cycle):
    order = ReceiptOrder()
    snap = make_cycle()
    assert order.accept(snap) is snap
    with pytest.raises(ContractError): order.accept(snap)
    assert order.usable is None


def test_coasting_age_compensated_once(make_cycle):
    tracker = ObservedShapeTracker()
    for i in range(5):
        public, shapes, _ = tracker.update(observed(1.+.1*i), 1_000_000_000+i*100_000_000, i)
    public, shapes, _ = tracker.update([], 1_500_000_000, 5)
    snap = make_cycle(public, shapes, epoch=1_650_000_000)
    track = snap.tracks[0]
    assert track.state == 'coasting' and track.observation_ns == 1_400_000_000
    assert track.association_sequence == 4
    field = TemporalSoftField(snap)
    first = field.translated_cells(0, 0)
    future = field.translated_cells(0, 2)
    assert first[0, 0] == pytest.approx(track.anchor[0]+track.origin[0]+(track.cells[0][0]+.5)*.05+.15*track.velocity[0])
    assert future[0, 0]-first[0, 0] == pytest.approx(.1*track.velocity[0])
    # A second observation-age addition would advance by .1*v and fail above.


def test_stationary_tentative_shape_is_not_hidden(make_cycle):
    tracker = ObservedShapeTracker()
    for i in range(5): public, shapes, _ = tracker.update(observed(), 1_000_000_000+i*100_000_000, i)
    snap = make_cycle(public, shapes, epoch=1_400_000_000)
    assert snap.tracks[0].state == 'tentative'
    field = TemporalSoftField(snap)
    assert (field.translated_cells(0, 0) == field.translated_cells(0, 30)).all()
    assert field.sample((1., 0.), 0).residual == 8.


def test_members_do_not_follow_reordered_detection_indices():
    tracker = ObservedShapeTracker()
    tracker.update(observed(1.)+observed(3.), 1_000_000_000, 1, [10, 11, 12, 30, 31, 32])
    public, shapes, update = tracker.update(observed(3.05)+observed(1.05), 1_100_000_000, 2, [33, 34, 35, 13, 14, 15])
    by_id = {shape['track_id']: shape for shape in shapes['tracks']}
    assert dict(update.associations) == {1: 1, 2: 0}
    assert set(by_id[1]['member_ids']) == {13, 14, 15}
    assert set(by_id[2]['member_ids']) == {33, 34, 35}
    assert by_id[1]['detection_index'] == 1
