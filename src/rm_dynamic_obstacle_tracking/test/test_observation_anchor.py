"""The public mode must not change association, filter velocity or lifecycle."""
import math

import pytest

from rm_dynamic_obstacle_tracking.core import Detection, MultiObjectTracker, Point2D, TrackState


def observation_tracker(**kwargs):
    return MultiObjectTracker(public_anchor_mode="last_observation_cv",
        velocity_decay_tau=0.0, max_prediction_speed=0.0, **kwargs)


def detection(x, y=0.0):
    return Detection(Point2D(x, y), 0.3, 0.2, 8)


def test_real_detection_anchor_replaces_filtered_position_without_changing_filter():
    common = dict(association_gate=2., min_hits_to_confirm=1,
                  measurement_noise=2., process_noise=.5)
    filtered = MultiObjectTracker(velocity_decay_tau=0., max_prediction_speed=0., **common)
    observed = observation_tracker(**common)
    for stamp, x, y in [(1., 0., 0.), (1.1, .7, -.3), (1.2, .4, .2)]:
        a = filtered.update([detection(x, y)], stamp).tracks[0]
        b = observed.update([detection(x, y)], stamp).tracks[0]
        assert b.position == Point2D(x, y)
        assert (a.velocity, a.track_id, a.state, a.size_x, a.size_y,
                a.observations, a.misses) == (b.velocity, b.track_id, b.state,
                b.size_x, b.size_y, b.observations, b.misses)
    assert a.position != b.position


def test_coasting_uses_real_epoch_gap_instead_of_clipped_filter_dt():
    tracker = observation_tracker(association_gate=2., min_hits_to_confirm=1,
                                 max_update_dt=.01, max_coast_time_sec=1.)
    tracker.update([detection(0.)], 10.)
    observed = tracker.update([detection(.3, -.2)], 10.1).tracks[0]
    for stamp in (10.2, 10.8):
        coasted = tracker.update([], stamp).tracks[0]
        assert coasted.state == TrackState.COASTING
        assert coasted.last_observation_timestamp == 10.1
        assert coasted.position == Point2D(.3 + coasted.velocity.x*(stamp-10.1),
                                          -.2 + coasted.velocity.y*(stamp-10.1))
        assert coasted.velocity == observed.velocity
        for k, p in enumerate(coasted.prediction):
            future = (k+1)*.1
            assert p == Point2D(coasted.position.x + coasted.velocity.x*future,
                               coasted.position.y + coasted.velocity.y*future)


def test_reacquisition_and_reset_never_keep_an_old_observation_anchor():
    tracker = observation_tracker(association_gate=2., min_hits_to_confirm=1,
                                 max_coast_time_sec=.6)
    first = tracker.update([detection(0.)], 10.).tracks[0]
    tracker.update([], 10.1)
    recovered = tracker.update([detection(.25)], 10.2).tracks[0]
    assert recovered.track_id == first.track_id
    assert recovered.position == Point2D(.25, 0.)
    assert recovered.last_observation_timestamp == 10.2
    for stamp in (9., 9.):  # both backward and equal epochs reset existing tracks
        reset = tracker.update([detection(-.7)], stamp)
        assert reset.time_reset and reset.deleted == 1
        assert reset.tracks[0].position == Point2D(-.7, 0.)
        assert reset.tracks[0].last_observation_timestamp == stamp
        assert reset.tracks[0].track_id != recovered.track_id
        recovered = reset.tracks[0]


def test_long_gap_expires_anchor_before_matching():
    tracker = observation_tracker(min_hits_to_confirm=1, max_coast_time_sec=.2)
    old = tracker.update([detection(0.)], 1.).tracks[0]
    new = tracker.update([detection(0.)], 1.5)
    assert (new.created, new.deleted) == (1, 1)
    assert new.tracks[0].track_id != old.track_id
    assert new.tracks[0].position == Point2D(0., 0.)


def test_integer_source_duration_prevents_float_epoch_cancellation():
    tracker = observation_tracker(association_gate=2., min_hits_to_confirm=1)
    initial = 1700000000*10**9+123456789
    tracker.update([detection(0.)], initial/1e9, source_stamp_ns=initial)
    observed_ns = initial+100000000
    observed = tracker.update([detection(.3)], observed_ns/1e9,
                              source_stamp_ns=observed_ns).tracks[0]
    coast_ns = initial+500000000
    coast = tracker.update([], coast_ns/1e9, source_stamp_ns=coast_ns).tracks[0]
    exact_duration = (coast_ns-observed_ns)/1e9
    assert exact_duration != coast_ns/1e9-observed_ns/1e9
    assert coast.position.x == .3+observed.velocity.x*exact_duration
    with pytest.raises(ValueError):
        tracker.update([], coast_ns/1e9, source_stamp_ns=coast_ns+100000000)


@pytest.mark.parametrize('kwargs', [dict(public_anchor_mode='center'),
    dict(public_anchor_mode='last_observation_cv'),
    dict(public_anchor_mode='last_observation_cv', velocity_decay_tau=0.),
    dict(public_anchor_mode='last_observation_cv', max_prediction_speed=0.),
    dict(public_anchor_mode='last_observation_cv', velocity_decay_tau=math.nan,
         max_prediction_speed=0.)])
def test_unknown_mode_and_ambiguous_display_configuration_are_rejected(kwargs):
    with pytest.raises(ValueError):
        MultiObjectTracker(**kwargs)
