"""Behavior tests for the USB camera publisher."""

from unittest.mock import Mock

import cv2
from builtin_interfaces.msg import Time
import numpy as np
from std_msgs.msg import Header

from hand_camera_driver import usb_camera_node


def test_encode_jpeg_message_preserves_header_and_is_decodable():
    """JPEG output keeps source metadata and contains a valid color image."""
    frame = np.zeros((8, 12, 3), dtype=np.uint8)
    frame[:, :, 1] = 180
    header = Header()
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    header.frame_id = "left_hand_camera_frame"

    msg = usb_camera_node.encode_jpeg_message(frame, header, jpeg_quality=70)

    assert msg.header == header
    assert msg.format == "jpeg"
    decoded = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded is not None
    assert decoded.shape == frame.shape


def test_publish_frame_skips_jpeg_encoding_without_subscribers(monkeypatch):
    """Raw publishing does not pay JPEG cost when compressed output is unused."""
    node = object.__new__(usb_camera_node.UsbCameraNode)
    node.bridge = Mock()
    node.publisher = Mock()
    node.compressed_publisher = Mock()
    node.compressed_publisher.get_subscription_count.return_value = 0
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=42)
    node.frame_id = "left_hand_camera_frame"
    node.jpeg_quality = 70
    frame = np.zeros((8, 12, 3), dtype=np.uint8)
    raw_msg = Mock()
    raw_msg.header = Header()
    node.bridge.cv2_to_imgmsg.return_value = raw_msg
    encode = Mock(side_effect=AssertionError("JPEG encoding must be skipped"))
    monkeypatch.setattr(usb_camera_node, "encode_jpeg_message", encode)

    node._publish_frame(frame)

    node.publisher.publish.assert_called_once_with(raw_msg)
    encode.assert_not_called()
    node.compressed_publisher.publish.assert_not_called()


def test_main_does_not_shutdown_an_already_stopped_context(monkeypatch):
    """SIGINT cleanup avoids calling shutdown twice on the ROS context."""
    node = Mock()
    monkeypatch.setattr(usb_camera_node, "UsbCameraNode", Mock(return_value=node))
    monkeypatch.setattr(usb_camera_node.rclpy, "init", Mock())
    monkeypatch.setattr(
        usb_camera_node.rclpy,
        "spin",
        Mock(side_effect=KeyboardInterrupt),
    )
    monkeypatch.setattr(usb_camera_node.rclpy, "ok", Mock(return_value=False))
    shutdown = Mock(side_effect=AssertionError("ROS context is already stopped"))
    monkeypatch.setattr(usb_camera_node.rclpy, "shutdown", shutdown)

    usb_camera_node.main()

    node.destroy_node.assert_called_once_with()
    shutdown.assert_not_called()
