import threading

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from rm_map_tools.mapping_session_node import MappingSessionNode
from rm_map_tools.pointcloud_sampler_node import PointCloudSamplerNode


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _sampler_without_ros_runtime():
    sampler = object.__new__(PointCloudSamplerNode)
    sampler._lock = threading.Lock()
    sampler._latest = None
    sampler._received_sequence = 0
    sampler._published_sequence = 0
    sampler._enabled = False
    sampler._publisher = _CapturePublisher()
    return sampler


def test_sampler_pause_resume_drops_paused_and_stale_clouds():
    sampler = _sampler_without_ros_runtime()

    before_start = PointCloud2()
    sampler._cloud_callback(before_start)
    sampler._publish_latest()
    assert sampler._publisher.messages == []

    sampler._status_callback(Bool(data=True))
    sampler._publish_latest()
    assert sampler._publisher.messages == []

    active = PointCloud2()
    sampler._cloud_callback(active)
    sampler._publish_latest()
    sampler._publish_latest()
    assert sampler._publisher.messages == [active]

    sampler._status_callback(Bool(data=False))
    received_while_paused = PointCloud2()
    sampler._cloud_callback(received_while_paused)
    sampler._publish_latest()
    assert sampler._publisher.messages == [active]

    sampler._status_callback(Bool(data=True))
    sampler._publish_latest()
    assert sampler._publisher.messages == [active]

    received_after_resume = PointCloud2()
    sampler._cloud_callback(received_after_resume)
    sampler._publish_latest()
    assert sampler._publisher.messages == [active, received_after_resume]


def test_mapping_services_publish_recording_state():
    session = object.__new__(MappingSessionNode)
    session._recording = True
    session._recording_publisher = _CapturePublisher()

    stop_response = session._stop(Trigger.Request(), Trigger.Response())
    assert stop_response.success
    assert stop_response.message == "mapping recording paused"
    assert session._recording is False
    assert session._recording_publisher.messages[-1].data is False

    start_response = session._start(Trigger.Request(), Trigger.Response())
    assert start_response.success
    assert start_response.message == "mapping recording enabled"
    assert session._recording is True
    assert session._recording_publisher.messages[-1].data is True
