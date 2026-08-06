#!/usr/bin/env python3
"""Publish an on-demand JPEG transport for a local raw ROS image topic."""

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from hand_camera_driver.usb_camera_node import encode_jpeg_message


class CompressedImageRelay(Node):
    """Compress raw images only while the output has subscribers."""

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
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            CompressedImage, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Relaying {input_topic} -> {output_topic} on demand "
            f"(JPEG quality={self.jpeg_quality})"
        )

    def _image_callback(self, msg: Image) -> None:
        if self.publisher.get_subscription_count() == 0:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
