#!/usr/bin/env python3
"""Publish a V4L2 USB camera as raw and on-demand compressed images."""

from __future__ import annotations

import os
import re

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def encode_jpeg_message(frame, header, jpeg_quality: int) -> CompressedImage:
    """Encode a BGR frame as a JPEG ROS message with the supplied header."""
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


class UsbCameraNode(Node):
    """Small OpenCV-backed USB camera publisher."""

    def __init__(self) -> None:
        super().__init__("usb_camera_node")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "usb_camera_frame")
        self.declare_parameter("topic", "image_raw")
        self.declare_parameter("compressed_topic", "")
        self.declare_parameter("jpeg_quality", 70)

        self.device = self.get_parameter("device").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value
        compressed_topic = self.get_parameter("compressed_topic").value
        if not compressed_topic:
            compressed_topic = f"{topic}/compressed"
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, topic, qos_profile_sensor_data
        )
        self.compressed_publisher = self.create_publisher(
            CompressedImage, compressed_topic, qos_profile_sensor_data
        )
        self.cap = None
        self._warned_closed = False
        self._open()

        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self._tick)

    def _capture_source(self) -> int | str:
        if isinstance(self.device, str) and self.device.isdigit():
            return int(self.device)
        if isinstance(self.device, str) and os.path.exists(self.device):
            real_device = os.path.realpath(self.device)
            match = re.fullmatch(r"/dev/video(\d+)", real_device)
            if match:
                return int(match.group(1))
            return real_device
        return self.device

    def _open(self) -> bool:
        if self.cap is not None:
            self.cap.release()
        source = self._capture_source()
        self.cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            if not self._warned_closed:
                self.get_logger().error(
                    f"Cannot open camera: {self.device} (source={source})"
                )
                self._warned_closed = True
            return False

        self._warned_closed = False
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_logger().info(
            f"Publishing {self.device} ({source}) -> {self.publisher.topic_name} "
            f"({self.width}x{self.height}@{self.fps:g}, frame={self.frame_id})"
        )
        return True

    def _tick(self) -> None:
        if self.cap is None or not self.cap.isOpened():
            self._open()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn(f"Read failed, reopening {self.device}")
            self._open()
            return

        self._publish_frame(frame)

    def _publish_frame(self, frame) -> None:
        """Publish raw output and encode compressed output only when requested."""
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

        if self.compressed_publisher.get_subscription_count() == 0:
            return
        compressed_msg = encode_jpeg_message(
            frame,
            msg.header,
            self.jpeg_quality,
        )
        self.compressed_publisher.publish(compressed_msg)

    def destroy_node(self) -> None:
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UsbCameraNode()
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
