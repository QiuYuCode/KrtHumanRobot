from __future__ import annotations

import rclpy
from rclpy.node import Node


class VoiceVolumeNode(Node):
    """音量控制节点骨架，后续接入 PulseAudio/ALSA。"""

    def __init__(self) -> None:
        super().__init__("voice_volume")
        self.get_logger().info("voice_volume skeleton started")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceVolumeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
