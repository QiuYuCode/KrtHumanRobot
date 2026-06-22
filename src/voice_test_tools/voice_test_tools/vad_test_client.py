from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from voice_interfaces.msg import VoiceVadEvent

from voice_test_tools.common import shutdown_node


class VadTestClient(Node):
    """Wait for a complete speech_start/speech_end VAD sequence."""

    def __init__(self) -> None:
        super().__init__("vad_test_client")
        self.declare_parameter("timeout_sec", 20.0)
        self.speech_started = False
        self.speech_ended = False
        self.subscription = self.create_subscription(
            VoiceVadEvent, "/voice/vad/events", self._on_event, 20
        )
        self.get_logger().info("等待 VAD 事件，请说一句话后保持安静……")

    def _on_event(self, message: VoiceVadEvent) -> None:
        self.get_logger().info(
            f"VAD 事件: type={message.event_type} "
            f"confidence={message.confidence:.3f}"
        )
        if message.event_type == "speech_start":
            self.speech_started = True
        elif message.event_type == "speech_end" and self.speech_started:
            self.speech_ended = True
            self.get_logger().info("VAD 测试成功: speech_start -> speech_end")


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = VadTestClient()
    deadline = time.monotonic() + float(node.get_parameter("timeout_sec").value)
    try:
        while rclpy.ok() and not node.speech_ended and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.speech_ended:
            node.get_logger().error(
                "VAD 测试超时，未收到完整的 speech_start/speech_end"
            )
            return 1
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        shutdown_node(node)
