"""Small isolated message/node loopback, with no Nav2 or chassis processes."""
import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.time import Time
from rm_competition_interfaces.msg import DynamicObstaclePredictionArray, DynamicObstaclePrediction
from rm_r4_interfaces.msg import ObservedPredictionEnvelope
from sensor_msgs.msg import LaserScan

from rm_dynamic_obstacle_tracking.dynamic_obstacle_tracker_node import DynamicObstacleTrackerNode
from rm_dynamic_obstacle_tracking.observed_members import MemberBatch, MemberRecord, make_envelope
from rm_dynamic_obstacle_tracking.core import Point2D


@pytest.fixture
def runtime(request):
    enabled = request.param
    topic = "/r4_members_check/run_"+uuid.uuid4().hex
    args = ["--ros-args", "-p", "predictions_topic:="+topic+"/prediction"]
    if enabled:
        for value in ["observed_members.enabled:=true", "observed_members.topic:="+topic+"/members",
                      "prediction.anchor_mode:=last_observation_cv", "prediction.velocity_decay_tau:=0.0",
                      "prediction.max_speed:=0.0", "tracker.min_hits_to_confirm:=1",
                      "tracker.min_displacement_to_confirm:=0.0"]:
            args += ["-p", value]
    rclpy.init(args=args)
    tracker = probe = executor = None
    try:
        tracker = DynamicObstacleTrackerNode()
        probe = Node("r4_members_probe", use_global_arguments=False)
        messages, envelopes = [], []
        public = probe.create_subscription(DynamicObstaclePredictionArray, topic+"/prediction", messages.append, 10)
        private = probe.create_subscription(ObservedPredictionEnvelope, topic+"/members", envelopes.append, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(tracker)
        executor.add_node(probe)
        yield tracker, executor, public, private, messages, envelopes
    finally:
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if tracker is not None:
            tracker.destroy_node()
        rclpy.shutdown()


def wait_for(executor, predicate, limit=3.):
    deadline = time.monotonic()+limit
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.01)
    assert predicate(), "ROS loopback discovery/receipt timeout"


def prepare_scan_input(tracker):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = grid.info.height = 80
    grid.info.resolution = .1
    grid.info.origin.position.x = grid.info.origin.position.y = -4.
    grid.info.origin.orientation.w = 1.
    grid.data = [0]*(80*80)
    tracker._on_map(grid)
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.child_frame_id = "r4_test_scan"
    transform.transform.rotation.w = 1.
    tracker._tf_buffer.set_transform_static(transform, "r4_members_check")


def scan(epoch_ns, offset=0., empty=False):
    message = LaserScan()
    message.header.frame_id = "r4_test_scan"
    message.header.stamp = Time(nanoseconds=epoch_ns).to_msg()
    message.angle_min = -.1
    message.angle_increment = .04
    message.range_min, message.range_max = .05, 3.
    message.ranges = [float("inf")]*6 if empty else [float("nan"), 1.+offset, 1.02+offset,
                                                    float("inf"), 1.03+offset, 4.]
    return message


@pytest.mark.parametrize("runtime", [True], indirect=True)
def test_one_tracker_builds_public_and_members_once_with_exact_wire(runtime):
    tracker, executor, public, private, messages, envelopes = runtime
    prepare_scan_input(tracker)
    wait_for(executor, lambda: tracker.count_publishers(public.topic_name) == 1
             and tracker.count_publishers(private.topic_name) == 1)
    calls = []
    original = tracker._tracker.update

    def counted(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    tracker._tracker.update = counted
    source = tracker.get_clock().now().nanoseconds-200_000_000
    for index, (offset, empty) in enumerate([(0., False), (.1, False), (0., True)]):
        tracker._on_scan(scan(source+index*50_000_000, offset, empty))
        wait_for(executor, lambda: len(messages) == index+1 and len(envelopes) == index+1)
        envelope, prediction = envelopes[-1], messages[-1]
        assert envelope.complete and envelope.reason == "ok"
        assert serialize_message(envelope.prediction) == serialize_message(prediction)
        assert deserialize_message(serialize_message(envelope), ObservedPredictionEnvelope) == envelope
        assert envelope.prediction.schema == prediction.SCHEMA_OBSERVATION_ANCHOR
        assert envelope.sequence == index+1
        assert set(envelope.tracks[0].source_member_ids) == {1, 2, 4}
        assert envelope.producer_generation == 0
    assert len(calls) == 3
    assert envelopes[-1].tracks == envelopes[-2].tracks
    assert envelopes[-1].prediction.tracks[0].position != envelopes[-2].prediction.tracks[0].position
    old_instance = envelopes[-1].producer_id
    tracker._on_scan(scan(source+25_000_000))
    wait_for(executor, lambda: len(envelopes) == 4)
    assert envelopes[-1].complete and envelopes[-1].producer_generation == 1
    assert envelopes[-1].producer_id == old_instance and envelopes[-1].sequence == 4
    assert envelopes[-1].tracks[0].track_id != envelopes[-2].tracks[0].track_id
    assert envelopes[-1].tracks[0].last_observation_stamp == Time(nanoseconds=source+25_000_000).to_msg()
    assert len(calls) == 4


@pytest.mark.parametrize("runtime", [False], indirect=True)
def test_default_has_only_legacy_public_output_and_no_velocity_owner(runtime):
    tracker, executor, public, private, messages, envelopes = runtime
    prepare_scan_input(tracker)
    wait_for(executor, lambda: tracker.count_publishers(public.topic_name) == 1)
    assert tracker._members_pub is None and tracker.count_publishers(private.topic_name) == 0
    tracker._on_scan(scan(tracker.get_clock().now().nanoseconds-50_000_000))
    wait_for(executor, lambda: len(messages) == 1)
    assert messages[0].schema == messages[0].SCHEMA and not envelopes
    topics = tracker.get_publisher_names_and_types_by_node(tracker.get_name(), tracker.get_namespace())
    assert not any("geometry_msgs/msg/Twist" in types for _, types in topics)


def test_envelope_rejects_incomplete_or_unbound_public_value():
    public = DynamicObstaclePredictionArray()
    public.schema = public.SCHEMA_OBSERVATION_ANCHOR
    public.authority = public.AUTHORITY_SHADOW_ONLY
    public.complete = False
    result = make_envelope(public, MemberBatch(True, "ok"), "test", 0, 1)
    assert not result.complete and result.reason == "public_incomplete" and not result.tracks
    public.complete = True
    public.schema = public.SCHEMA
    assert make_envelope(public, MemberBatch(True, "ok"), "test", 0, 1).reason == "public_contract"
    public.schema = public.SCHEMA_OBSERVATION_ANCHOR
    record = MemberRecord(3, 10_000_000_001, 1, 0, Point2D(0., 0.), (Point2D(0., 0.),), (4,))
    result = make_envelope(public, MemberBatch(True, "ok", (record,)), "test", 0, 1)
    assert not result.complete and result.reason == "track_identity"


def test_observation_mismatch_discards_all_partial_members():
    public = DynamicObstaclePredictionArray()
    public.schema = public.SCHEMA_OBSERVATION_ANCHOR
    public.authority = public.AUTHORITY_SHADOW_ONLY
    public.header.stamp = Time(nanoseconds=10_100_000_003).to_msg()
    public.complete, public.total_track_count = True, 2
    records = []
    for tid in (3, 4):
        track = DynamicObstaclePrediction()
        track.track_id = tid
        track.last_observation_stamp = Time(nanoseconds=10_000_000_001).to_msg()
        public.tracks.append(track)
        records.append(MemberRecord(tid, 10_000_000_001 if tid == 3 else 10_000_000_002,
            1, 0, Point2D(0., 0.), (Point2D(0., 0.),), (tid,)))
    result = make_envelope(public, MemberBatch(True, "ok", tuple(records)), "test", 0, 1)
    assert not result.complete and result.reason == "observation_identity" and not result.tracks
