"""Behavior tests for the D435 compressed image relay."""

from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import Image

from hand_camera_driver import compressed_image_relay


def test_relay_publishes_without_compressed_subscribers(monkeypatch):
    """The relay keeps publishing so late DDS subscribers receive a frame."""
    node = object.__new__(compressed_image_relay.CompressedImageRelay)
    node.publisher = Mock()
    node.publisher.get_subscription_count.return_value = 0
    node.jpeg_quality = 70
    frame = Mock()
    decode = Mock(return_value=frame)
    compressed = Mock()
    encode = Mock(return_value=compressed)
    monkeypatch.setattr(compressed_image_relay, "image_message_to_bgr", decode)
    monkeypatch.setattr(compressed_image_relay, "encode_jpeg_message", encode)

    source = Image()
    node._image_callback(source)

    decode.assert_called_once_with(source)
    encode.assert_called_once_with(frame, source.header, 70)
    node.publisher.publish.assert_called_once_with(compressed)


def test_relay_preserves_header_when_compressed_output_is_requested(monkeypatch):
    """The relay publishes encoded data with the source timestamp and frame."""
    node = object.__new__(compressed_image_relay.CompressedImageRelay)
    node.publisher = Mock()
    node.publisher.get_subscription_count.return_value = 1
    node.jpeg_quality = 70
    frame = Mock()
    decode = Mock(return_value=frame)
    source = Image()
    source.header.stamp.sec = 123
    source.header.frame_id = "camera_color_optical_frame"
    compressed = Mock()
    encode = Mock(return_value=compressed)
    monkeypatch.setattr(compressed_image_relay, "image_message_to_bgr", decode)
    monkeypatch.setattr(compressed_image_relay, "encode_jpeg_message", encode)

    node._image_callback(source)

    decode.assert_called_once_with(source)
    encode.assert_called_once_with(frame, source.header, 70)
    node.publisher.publish.assert_called_once_with(compressed)


def test_rgb_image_conversion_handles_row_padding():
    """The relay converts RealSense RGB8 data without cv_bridge/OpenCV ABI mixing."""
    source = Image()
    source.width = 2
    source.height = 1
    source.encoding = "rgb8"
    source.step = 8
    source.data = bytes([255, 0, 0, 0, 255, 0, 99, 99])

    frame = compressed_image_relay.image_message_to_bgr(source)

    np.testing.assert_array_equal(
        frame,
        np.array([[[0, 0, 255], [0, 255, 0]]], dtype=np.uint8),
    )
