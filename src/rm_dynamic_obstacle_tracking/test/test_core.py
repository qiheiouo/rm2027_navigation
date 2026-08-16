import math

from rm_dynamic_obstacle_tracking.core import (
    Detection,
    MultiObjectTracker,
    OccupancyMap,
    Point2D,
    TrackState,
    cluster_points,
    dynamic_candidates,
    planar_rotation_matrix,
)


def _detection(x: float, y: float = 0.0) -> Detection:
    return Detection(Point2D(x, y), 0.3, 0.2, 8)


def test_static_map_subtraction_rejects_wall_and_unknown() -> None:
    data = [0] * 25
    data[2 * 5 + 2] = 100
    data[4 * 5 + 4] = -1
    occupancy = OccupancyMap(5, 5, 1.0, 0.0, 0.0, 0.0, data)

    candidates = dynamic_candidates(
        [Point2D(2.5, 2.5), Point2D(0.5, 0.5), Point2D(4.5, 4.5)],
        occupancy,
        static_distance_threshold=0.6,
        require_known_free=True,
    )

    assert candidates == [Point2D(0.5, 0.5)]


def test_map_origin_yaw_is_respected() -> None:
    occupancy = OccupancyMap(
        width=2,
        height=1,
        resolution=1.0,
        origin_x=10.0,
        origin_y=20.0,
        origin_yaw=math.pi / 2.0,
        data=[100, 0],
    )
    assert occupancy.value_at_world(Point2D(9.5, 20.5)) == 100
    assert occupancy.value_at_world(Point2D(9.5, 21.5)) == 0


def test_planar_point_transform_uses_full_quaternion() -> None:
    half_angle = math.pi / 4.0
    rotation_xx, rotation_xy, rotation_yx, rotation_yy = planar_rotation_matrix(
        (0.0, math.sin(half_angle), 0.0, math.cos(half_angle)),
    )
    transformed = Point2D(
        2.0 + rotation_xx * 1.0 + rotation_xy * 0.0,
        3.0 + rotation_yx * 1.0 + rotation_yy * 0.0,
    )

    assert math.isclose(transformed.x, 2.0, abs_tol=1.0e-9)
    assert math.isclose(transformed.y, 3.0, abs_tol=1.0e-9)


def test_euclidean_clustering_rejects_noise_and_large_components() -> None:
    points = [
        Point2D(0.00, 0.00),
        Point2D(0.08, 0.02),
        Point2D(0.16, 0.01),
        Point2D(2.0, 2.0),
        Point2D(3.0, 0.0),
        Point2D(3.1, 0.0),
        Point2D(4.2, 0.0),
    ]
    detections = cluster_points(points, tolerance=0.12, min_points=3, max_extent=0.8)
    assert len(detections) == 1
    assert detections[0].point_count == 3
    assert math.isclose(detections[0].centroid.x, 0.08)


def test_tracker_confirms_coasts_and_deletes() -> None:
    tracker = MultiObjectTracker(
        min_hits_to_confirm=2,
        tentative_max_misses=0,
        max_coast_time_sec=0.25,
        prediction_steps=3,
    )
    first = tracker.update([_detection(0.0)], 1.0)
    assert first.tracks[0].state == TrackState.TENTATIVE
    second = tracker.update([_detection(0.1)], 1.1)
    assert second.tracks[0].state == TrackState.CONFIRMED
    assert len(second.tracks[0].prediction) == 3

    coast = tracker.update([], 1.2)
    assert coast.tracks[0].state == TrackState.COASTING
    tracker.update([], 1.3)
    deleted = tracker.update([], 1.4)
    assert deleted.tracks == ()
    assert deleted.deleted == 1


def test_tracker_estimates_constant_velocity_and_keeps_id() -> None:
    tracker = MultiObjectTracker(
        association_gate=0.5,
        process_noise=0.5,
        measurement_noise=0.01,
        min_hits_to_confirm=2,
        prediction_steps=5,
        prediction_dt=0.2,
        velocity_decay_tau=0.0,
    )
    track_id = None
    snapshot = None
    for index in range(12):
        update = tracker.update([_detection(index * 0.1)], 1.0 + index * 0.1)
        snapshot = update.tracks[0]
        track_id = snapshot.track_id if track_id is None else track_id
        assert snapshot.track_id == track_id
    assert snapshot is not None
    assert snapshot.state == TrackState.CONFIRMED
    assert 0.75 < snapshot.velocity.x < 1.25
    assert snapshot.prediction[-1].x > snapshot.position.x + 0.7


def test_time_reset_invalidates_old_tracks() -> None:
    tracker = MultiObjectTracker(min_hits_to_confirm=1)
    first = tracker.update([_detection(0.0)], 10.0)
    reset = tracker.update([_detection(1.0)], 9.0)
    assert reset.time_reset
    assert reset.deleted == 1
    assert reset.tracks[0].track_id != first.tracks[0].track_id


def test_brief_occlusion_preserves_confirmed_track_id() -> None:
    tracker = MultiObjectTracker(
        association_gate=0.5,
        min_hits_to_confirm=2,
        max_coast_time_sec=0.5,
    )
    tracker.update([_detection(0.0)], 1.0)
    confirmed = tracker.update([_detection(0.1)], 1.1).tracks[0]
    assert confirmed.state == TrackState.CONFIRMED

    coasted = tracker.update([], 1.2).tracks[0]
    reacquired = tracker.update([_detection(0.25)], 1.3).tracks[0]

    assert coasted.state == TrackState.COASTING
    assert reacquired.track_id == confirmed.track_id
    assert reacquired.misses == 0


def test_long_input_gap_expires_track_before_association() -> None:
    tracker = MultiObjectTracker(
        min_hits_to_confirm=1,
        max_coast_time_sec=0.6,
    )
    initial = tracker.update([_detection(0.0)], 1.0)

    resumed = tracker.update([_detection(0.0)], 10.0)

    assert resumed.deleted == 1
    assert resumed.created == 1
    assert resumed.tracks[0].track_id != initial.tracks[0].track_id


def test_two_separated_targets_keep_distinct_ids() -> None:
    tracker = MultiObjectTracker(
        association_gate=0.5,
        min_hits_to_confirm=1,
        process_noise=0.5,
        measurement_noise=0.01,
    )
    initial = tracker.update([_detection(-1.0), _detection(1.0)], 1.0)
    initial_ids = [track.track_id for track in initial.tracks]

    updated = tracker.update([_detection(-0.9), _detection(0.9)], 1.1)

    assert [track.track_id for track in updated.tracks] == initial_ids
    assert all(track.state == TrackState.CONFIRMED for track in updated.tracks)


def test_confirmed_coasting_timeout_is_time_based() -> None:
    tracker = MultiObjectTracker(
        min_hits_to_confirm=2,
        max_coast_time_sec=0.5,
    )
    tracker.update([_detection(0.0)], 1.0)
    confirmed = tracker.update([_detection(0.1)], 1.1)
    assert confirmed.tracks[0].state == TrackState.CONFIRMED

    update = confirmed
    for index in range(1, 21):
        update = tracker.update([], 1.1 + 0.02 * index)
    assert update.tracks[0].state == TrackState.COASTING

    deleted = tracker.update([], 1.61)
    assert deleted.tracks == ()
    assert deleted.deleted == 1
