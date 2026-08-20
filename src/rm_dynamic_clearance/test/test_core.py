import math

import pytest

from rm_dynamic_clearance.core import (
    ADMISSION_DYNAMIC_CLEARANCE,
    ADMISSION_NONE,
    ClearanceConfig,
    ClearanceContractError,
    Decision,
    IntentSegment,
    Point2D,
    PredictedTrack,
    PredictionFrame,
    TRACK_COASTING,
    TRACK_CONFIRMED,
    TRACK_TENTATIVE,
    evaluate_dynamic_clearance,
)


REVISION = "a" * 64
REGION_SHA = "b" * 64


def _segment(
    start=2.0,
    end=4.0,
    *,
    admission=ADMISSION_DYNAMIC_CLEARANCE,
    blocked=False,
    speed=None,
):
    return IntentSegment(
        start_distance=start,
        end_distance=end,
        region_ids=("dog_hole_a",),
        blocked=blocked,
        max_linear_speed=speed,
        admission_policy=admission,
        traversal_policy=1,
    )


def _track(
    *,
    track_id=7,
    state=TRACK_CONFIRMED,
    x=3.0,
    y=0.0,
    steps=10,
    size=0.4,
):
    return PredictedTrack(
        track_id=track_id,
        state=state,
        position=Point2D(x, y),
        velocity=Point2D(0.0, 0.0),
        size_x=size,
        size_y=size,
        last_observation_stamp=100.0,
        observation_count=4,
        miss_count=0,
        prediction=tuple(Point2D(x, y) for _ in range(steps)),
    )


def _prediction(*tracks, stamp=100.0, dt=1.0, steps=10):
    return PredictionFrame(
        frame_id="map",
        source_stamp=stamp,
        processing_stamp=stamp + 0.01,
        prediction_dt=dt,
        prediction_steps=steps,
        complete=True,
        total_track_count=len(tracks),
        tracks=tuple(tracks),
    )


def _evaluate(*, segments=None, prediction=None, now=100.1, config=None, **kwargs):
    arguments = {
        "path_frame": "map",
        "path_points": (Point2D(0.0, 0.0), Point2D(4.0, 0.0)),
        "robot_position": Point2D(0.0, 0.0),
        "path_revision": REVISION,
        "annotated_path_revision": REVISION,
        "annotated_path_length": 4.0,
        "region_set_sha256": REGION_SHA,
        "segments": (_segment(),) if segments is None else segments,
        "prediction": _prediction() if prediction is None else prediction,
        "now": now,
        "config": ClearanceConfig() if config is None else config,
    }
    arguments.update(kwargs)
    return evaluate_dynamic_clearance(**arguments)


def test_empty_fresh_prediction_is_clear():
    report = _evaluate()
    assert report.decision == Decision.CLEAR
    assert report.reason == "clear"
    assert report.minimum_clearance is None
    assert report.sample_count == 21


def test_confirmed_overlap_is_blocked_and_binds_track():
    report = _evaluate(prediction=_prediction(_track()))
    assert report.decision == Decision.BLOCKED
    assert report.reason == "predicted_overlap"
    assert report.blocking_track_ids == (7,)
    assert report.minimum_clearance < 0.0
    assert report.region_ids == ("dog_hole_a",)
    assert report.traversal_policy == 1


def test_tentative_overlap_is_unknown_not_clear_or_blocked():
    report = _evaluate(prediction=_prediction(_track(state=TRACK_TENTATIVE)))
    assert report.decision == Decision.UNKNOWN
    assert report.reason == "tentative_overlap"
    assert report.blocking_track_ids == (7,)


def test_coasting_track_is_conservatively_included_by_default():
    track = _track(state=TRACK_COASTING, y=0.75, size=0.2)
    included = _evaluate(prediction=_prediction(track))
    excluded = _evaluate(
        prediction=_prediction(track),
        config=ClearanceConfig(include_coasting=False),
    )
    assert included.decision == Decision.BLOCKED
    assert excluded.decision == Decision.CLEAR


@pytest.mark.parametrize(
    ("stamp", "now", "reason"),
    [(99.0, 100.0, "prediction_stale"), (101.0, 100.0, "prediction_from_future")],
)
def test_prediction_time_bounds_return_unknown(stamp, now, reason):
    report = _evaluate(prediction=_prediction(stamp=stamp), now=now)
    assert report.decision == Decision.UNKNOWN
    assert report.reason == reason


def test_short_prediction_horizon_is_unknown_not_clear():
    prediction = _prediction(dt=0.1, steps=2)
    report = _evaluate(prediction=prediction)
    assert report.decision == Decision.UNKNOWN
    assert report.reason == "prediction_horizon_short"


def test_truncated_prediction_frame_is_unknown_not_clear():
    prediction = _prediction()
    prediction = PredictionFrame(
        frame_id=prediction.frame_id,
        source_stamp=prediction.source_stamp,
        processing_stamp=prediction.processing_stamp,
        prediction_dt=prediction.prediction_dt,
        prediction_steps=prediction.prediction_steps,
        complete=False,
        total_track_count=1,
        tracks=(),
    )
    report = _evaluate(prediction=prediction)
    assert report.decision == Decision.UNKNOWN
    assert report.reason == "prediction_track_limit"


def test_semantic_block_is_blocked_without_dynamic_track():
    report = _evaluate(segments=(_segment(blocked=True),))
    assert report.decision == Decision.BLOCKED
    assert report.reason == "semantic_region_blocked"


def test_no_admission_segment_is_explicitly_clear():
    report = _evaluate(segments=(_segment(admission=ADMISSION_NONE),))
    assert report.decision == Decision.CLEAR
    assert report.reason == "no_admission_required"
    assert report.region_ids == ()


def test_admission_outside_lookahead_is_not_evaluated():
    report = _evaluate(config=ClearanceConfig(lookahead_distance=1.0))
    assert report.decision == Decision.UNKNOWN
    assert report.reason == "admission_beyond_lookahead"


def test_annotated_speed_cap_changes_entry_eta():
    slow = IntentSegment(
        start_distance=0.0,
        end_distance=2.0,
        region_ids=("slow",),
        blocked=False,
        max_linear_speed=0.5,
        admission_policy=ADMISSION_NONE,
        traversal_policy=0,
    )
    report = _evaluate(segments=(slow, _segment()))
    assert report.entry_eta == pytest.approx(4.0)


def test_current_route_progress_reduces_remaining_eta():
    report = _evaluate(robot_position=Point2D(1.0, 0.1))
    assert report.route_progress == pytest.approx(1.0)
    assert report.entry_eta == pytest.approx(1.0)


def test_already_inside_admission_region_is_unknown():
    report = _evaluate(robot_position=Point2D(2.5, 0.0))
    assert report.decision == Decision.UNKNOWN
    assert report.reason == "already_in_admission_region"


def test_duplicate_path_points_do_not_break_arc_length_sampling():
    report = _evaluate(
        path_points=(
            Point2D(0.0, 0.0),
            Point2D(0.0, 0.0),
            Point2D(4.0, 0.0),
            Point2D(4.0, 0.0),
        )
    )
    assert report.decision == Decision.CLEAR


def test_interpolation_uses_eta_frame_not_current_position():
    moving = PredictedTrack(
        track_id=9,
        state=TRACK_CONFIRMED,
        position=Point2D(3.0, 3.0),
        velocity=Point2D(0.0, -1.0),
        size_x=0.2,
        size_y=0.2,
        last_observation_stamp=100.0,
        observation_count=5,
        miss_count=0,
        prediction=tuple(Point2D(3.0, 3.0 - index) for index in range(1, 11)),
    )
    report = _evaluate(prediction=_prediction(moving), now=100.0)
    assert report.decision == Decision.BLOCKED
    assert report.blocking_track_ids == (9,)


@pytest.mark.parametrize(
    "override",
    [
        {"annotated_path_revision": "c" * 64},
        {"annotated_path_length": 3.9},
        {"path_frame": ""},
        {"region_set_sha256": "bad"},
        {"robot_position": Point2D(0.0, 2.0)},
    ],
)
def test_path_and_annotation_contracts_fail_closed(override):
    with pytest.raises(ClearanceContractError):
        _evaluate(**override)


def test_duplicate_track_id_and_wrong_prediction_length_are_rejected():
    with pytest.raises(ClearanceContractError, match="duplicate"):
        _evaluate(prediction=_prediction(_track(), _track()))
    with pytest.raises(ClearanceContractError, match="length mismatch"):
        _evaluate(prediction=_prediction(_track(steps=9)))


def test_ambiguous_self_crossing_projection_is_rejected():
    with pytest.raises(ClearanceContractError, match="ambiguous"):
        _evaluate(
            path_points=(
                Point2D(-1.0, 0.0),
                Point2D(1.0, 0.0),
                Point2D(0.0, -1.0),
                Point2D(0.0, 1.0),
            ),
            robot_position=Point2D(0.0, 0.0),
        )


def test_nonfinite_geometry_and_bad_config_are_rejected():
    with pytest.raises(ClearanceContractError, match="finite"):
        _evaluate(
            path_points=(Point2D(0.0, 0.0), Point2D(math.nan, 0.0))
        )
    with pytest.raises(ClearanceContractError, match="sample_resolution"):
        _evaluate(config=ClearanceConfig(sample_resolution=0.0))


def test_sampling_budget_is_bounded():
    with pytest.raises(ClearanceContractError, match="sampling exceeds"):
        evaluate_dynamic_clearance(
            path_frame="map",
            path_points=(Point2D(0.0, 0.0), Point2D(2_000.0, 0.0)),
            robot_position=Point2D(0.0, 0.0),
            path_revision=REVISION,
            annotated_path_revision=REVISION,
            annotated_path_length=2_000.0,
            region_set_sha256=REGION_SHA,
            segments=(_segment(start=0.1, end=2_000.0),),
            prediction=_prediction(dt=10.0, steps=1_000),
            now=100.0,
            config=ClearanceConfig(
                nominal_speed=1.0,
                sample_resolution=0.1,
                lookahead_distance=2_000.0,
            ),
        )


def test_track_sample_product_is_bounded():
    tracks = tuple(
        _track(track_id=index + 1, x=1_000.0, steps=100)
        for index in range(101)
    )
    with pytest.raises(ClearanceContractError, match="comparisons"):
        evaluate_dynamic_clearance(
            path_frame="map",
            path_points=(Point2D(0.0, 0.0), Point2D(100.0, 0.0)),
            robot_position=Point2D(0.0, 0.0),
            path_revision=REVISION,
            annotated_path_revision=REVISION,
            annotated_path_length=100.0,
            region_set_sha256=REGION_SHA,
            segments=(_segment(start=0.1, end=100.0),),
            prediction=_prediction(*tracks, dt=1.0, steps=100),
            now=100.0,
            config=ClearanceConfig(
                sample_resolution=100.0 / 9_999.0,
                lookahead_distance=100.0,
            ),
        )
