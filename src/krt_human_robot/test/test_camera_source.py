"""Tests for ROS camera transport selection and decoding."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CompressedImage, Image

from krt_human_robot.behaviors.core.actions import camera_source


class FakeNode:
    """Capture subscription construction without starting an executor."""

    def __init__(self):
        self.message_type = None
        self.topic = None
        self.callback = None
        self.subscription = object()
        self.destroyed = []

    def create_subscription(self, message_type, topic, callback, _qos):
        self.message_type = message_type
        self.topic = topic
        self.callback = callback
        return self.subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


def make_config(transport="compressed", timeout=0.02):
    """Return the ROS camera settings used by these focused tests."""
    return SimpleNamespace(
        camera_ros_transport=transport,
        camera_ros_warmup_seconds=timeout,
        camera_ros_qos_depth=5,
        camera_ros_node_name="test_camera_source",
    )


def make_compressed_message(frame):
    """Encode a frame into the wire representation produced by the driver."""
    ok, encoded = cv2.imencode(".jpg", frame)
    assert ok
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


def test_ros_camera_source_decodes_compressed_topic_and_closes(monkeypatch):
    """Compressed transport appends the suffix, decodes, and destroys its sub."""
    node = FakeNode()
    monkeypatch.setattr(camera_source, "_ensure_ros_context", lambda _cfg: node)
    spec = camera_source.CameraSpec(ros_topic="/left_gripper/image_raw")
    source = camera_source.RosCameraSource("left_palm", spec, make_config())
    frame = np.zeros((8, 12, 3), dtype=np.uint8)
    frame[:, :, 2] = 200

    assert node.message_type is CompressedImage
    assert node.topic == "/left_gripper/image_raw/compressed"
    node.callback(make_compressed_message(frame))
    decoded = source.grab_frame()
    source.close()

    assert decoded.shape == frame.shape
    assert node.destroyed == [node.subscription]


def test_ros_camera_source_uses_explicit_compressed_topic(monkeypatch):
    """A dedicated JPEG topic avoids activating an image_transport plugin."""
    node = FakeNode()
    monkeypatch.setattr(camera_source, "_ensure_ros_context", lambda _cfg: node)
    spec = camera_source.CameraSpec(
        ros_topic="/camera/color/image_raw",
        ros_compressed_topic="/camera/color/image_jpeg",
    )

    camera_source.RosCameraSource("head", spec, make_config())

    assert node.message_type is CompressedImage
    assert node.topic == "/camera/color/image_jpeg"


def test_ros_camera_source_rejects_invalid_jpeg_then_times_out(monkeypatch):
    """An invalid compressed payload never becomes a usable cached frame."""
    node = FakeNode()
    monkeypatch.setattr(camera_source, "_ensure_ros_context", lambda _cfg: node)
    spec = camera_source.CameraSpec(ros_topic="/right_gripper/image_raw")
    source = camera_source.RosCameraSource(
        "right_palm", spec, make_config(timeout=0.001)
    )
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = b"not-a-jpeg"

    node.callback(msg)

    with pytest.raises(RuntimeError, match="未收到帧"):
        source.grab_frame()


def test_ros_camera_source_rejects_empty_payload_before_opencv(monkeypatch):
    """An empty compressed message never reaches OpenCV's decoder."""
    node = FakeNode()
    decode_calls = []
    monkeypatch.setattr(camera_source, "_ensure_ros_context", lambda _cfg: node)
    monkeypatch.setattr(
        camera_source.cv2,
        "imdecode",
        lambda *args: decode_calls.append(args),
    )
    source = camera_source.RosCameraSource(
        "head", camera_source.CameraSpec(ros_topic="/camera/color/image_raw"),
        make_config(timeout=0.001),
    )
    msg = CompressedImage()
    msg.format = "jpeg"

    node.callback(msg)

    assert decode_calls == []
    with pytest.raises(RuntimeError, match="未收到帧"):
        source.grab_frame()


@pytest.mark.parametrize(
    ("camera_id", "topic"),
    (
        ("head", "/camera/camera/color/image_raw"),
        ("left_palm", "/left_gripper/image_raw"),
        ("right_palm", "/right_gripper/image_raw"),
    ),
)
def test_ros_camera_source_reads_all_raw_cameras(monkeypatch, camera_id, topic):
    """All configured cameras use Image subscriptions and return one frame."""
    node = FakeNode()
    monkeypatch.setattr(camera_source, "_ensure_ros_context", lambda _cfg: node)
    source = camera_source.RosCameraSource(
        camera_id, camera_source.CameraSpec(ros_topic=topic),
        make_config(transport="raw"),
    )
    assert node.message_type is Image
    assert node.topic == topic

    msg = Image()
    msg.height, msg.width = 2, 3
    msg.encoding, msg.step = "bgr8", 9
    msg.data = bytes(18)
    node.callback(msg)

    assert source.grab_frame().shape == (2, 3, 3)
    source.close()
