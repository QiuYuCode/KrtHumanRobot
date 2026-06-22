from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from voice_interfaces.msg import VoiceKwsEvent

from voice_test_tools.common import shutdown_node


class KwsTestClient(Node):
    """Wait for one keyword event and report it."""

    def __init__(self) -> None:
        super().__init__("kws_test_client")
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("expected_keyword", "")
        self.event: VoiceKwsEvent | None = None
        self.subscription = self.create_subscription(
            VoiceKwsEvent, "/voice/kws/events", self._on_event, 20
        )
        self.get_logger().info("等待 KWS 事件，请说唤醒词……")

    def _on_event(self, message: VoiceKwsEvent) -> None:
        expected = str(self.get_parameter("expected_keyword").value).strip()
        if expected and message.keyword != expected:
            self.get_logger().warning(
                f"收到非目标唤醒词: {message.keyword}，继续等待 {expected}"
            )
            return
        self.event = message
        self.get_logger().info(
            f"KWS 测试成功: keyword={message.keyword} score={message.score:.3f}"
        )


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = KwsTestClient()
    deadline = time.monotonic() + float(node.get_parameter("timeout_sec").value)
    try:
        while rclpy.ok() and node.event is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.event is None:
            node.get_logger().error("KWS 测试超时，未收到唤醒词事件")
            return 1
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        shutdown_node(node)
