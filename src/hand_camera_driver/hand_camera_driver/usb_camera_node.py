#!/usr/bin/env python3
"""Publish a V4L2 USB camera as sensor_msgs/Image."""

from __future__ import annotations

import os
import re

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


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

        self.device = self.get_parameter("device").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, 10)
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

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
