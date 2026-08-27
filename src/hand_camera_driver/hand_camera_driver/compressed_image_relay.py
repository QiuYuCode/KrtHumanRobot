#!/usr/bin/env python3
"""Publish a dedicated JPEG transport for a local raw ROS image topic."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image


def image_message_to_bgr(msg: Image) -> np.ndarray:
    """Decode common 8-bit ROS image encodings without loading cv_bridge."""
    encoding = msg.encoding.strip().lower()
    channels = 1 if encoding == "mono8" else 3
    if encoding not in {"bgr8", "rgb8", "mono8"}:
        raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")

    row_bytes = int(msg.width) * channels
    step = int(msg.step)
    expected_size = step * int(msg.height)
    if step < row_bytes or len(msg.data) < expected_size:
        raise ValueError(
            f"Invalid image buffer: {msg.width}x{msg.height}, "
            f"step={step}, bytes={len(msg.data)}"
        )

    rows = np.frombuffer(msg.data, dtype=np.uint8, count=expected_size).reshape(
        int(msg.height), step
    )
    pixels = rows[:, :row_bytes]
    if encoding == "mono8":
        gray = pixels.reshape(int(msg.height), int(msg.width))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    frame = pixels.reshape(int(msg.height), int(msg.width), 3)
    if encoding == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(frame)


def encode_jpeg_message(
    frame: np.ndarray, header, jpeg_quality: int
) -> CompressedImage:
    """Encode a BGR frame while preserving the source ROS header."""
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)],
    )
    if not ok:
        raise RuntimeError("Failed to encode camera frame as JPEG")
    msg = CompressedImage()
    msg.header = header
    msg.format = "jpeg"
    msg.data = encoded.tobytes()
    return msg


class CompressedImageRelay(Node):
    """Continuously compress raw images for reliable late DDS subscribers."""

    def __init__(self) -> None:
        super().__init__("compressed_image_relay")
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "output_topic", "/camera/color/image_raw/compressed"
        )
        self.declare_parameter("jpeg_quality", 70)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.publisher = self.create_publisher(
            CompressedImage, output_topic, qos_profile_sensor_data
        )
        input_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            input_qos,
        )
        self.get_logger().info(
            f"Relaying {input_topic} -> {output_topic} on demand "
            f"(JPEG quality={self.jpeg_quality})"
        )

    def _image_callback(self, msg: Image) -> None:
        frame = image_message_to_bgr(msg)
        compressed = encode_jpeg_message(
            frame,
            msg.header,
            self.jpeg_quality,
        )
        self.publisher.publish(compressed)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CompressedImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
