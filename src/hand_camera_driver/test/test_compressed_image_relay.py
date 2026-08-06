"""Behavior tests for the D435 compressed image relay."""

from unittest.mock import Mock

from sensor_msgs.msg import Image

from hand_camera_driver import compressed_image_relay


def test_relay_skips_decoding_without_compressed_subscribers(monkeypatch):
    """The local raw stream incurs no conversion cost while unused remotely."""
    node = object.__new__(compressed_image_relay.CompressedImageRelay)
    node.publisher = Mock()
    node.publisher.get_subscription_count.return_value = 0
    node.bridge = Mock()
    node.jpeg_quality = 70

    node._image_callback(Image())

    node.bridge.imgmsg_to_cv2.assert_not_called()
    node.publisher.publish.assert_not_called()


def test_relay_preserves_header_when_compressed_output_is_requested(monkeypatch):
    """The relay publishes encoded data with the source timestamp and frame."""
    node = object.__new__(compressed_image_relay.CompressedImageRelay)
    node.publisher = Mock()
    node.publisher.get_subscription_count.return_value = 1
    node.bridge = Mock()
    node.jpeg_quality = 70
    frame = Mock()
    node.bridge.imgmsg_to_cv2.return_value = frame
    source = Image()
    source.header.stamp.sec = 123
    source.header.frame_id = "camera_color_optical_frame"
    compressed = Mock()
    encode = Mock(return_value=compressed)
    monkeypatch.setattr(compressed_image_relay, "encode_jpeg_message", encode)

    node._image_callback(source)

    node.bridge.imgmsg_to_cv2.assert_called_once_with(source, "bgr8")
    encode.assert_called_once_with(frame, source.header, 70)
    node.publisher.publish.assert_called_once_with(compressed)
