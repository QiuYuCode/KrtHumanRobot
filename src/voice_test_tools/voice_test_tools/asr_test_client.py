from __future__ import annotations

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from voice_interfaces.action import RecognizeStream

from voice_test_tools.common import shutdown_node


class AsrTestClient(Node):
    """Start one streaming ASR recognition session."""

    def __init__(self) -> None:
        super().__init__("asr_test_client")
        self.declare_parameter("backend", "local")
        self.declare_parameter("server_timeout_sec", 5.0)
        self.declare_parameter("result_timeout_sec", 20.0)
        self.client = ActionClient(self, RecognizeStream, "/voice/asr/stream")

    def feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"ASR 中间结果: {feedback.partial_text} "
            f"confidence={feedback.partial_confidence:.3f}"
        )

    def run(self) -> int:
        server_timeout = float(self.get_parameter("server_timeout_sec").value)
        if not self.client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error("ASR action 不可用: /voice/asr/stream")
            return 1

        goal = RecognizeStream.Goal()
        goal.backend = str(self.get_parameter("backend").value)
        self.get_logger().info("ASR 已启动，请说一句话后保持安静……")
        goal_future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=server_timeout)
        if not goal_future.done():
            self.get_logger().error("发送 ASR goal 超时")
            return 1
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("ASR goal 被拒绝")
            return 1

        result_future = goal_handle.get_result_async()
        result_timeout = float(self.get_parameter("result_timeout_sec").value)
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=result_timeout)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            self.get_logger().error("ASR 结果超时")
            return 1

        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f"ASR 测试失败: {result.error_message}")
            return 1
        self.get_logger().info(
            f"ASR 测试成功: text={result.text!r} "
            f"backend={result.backend_used} confidence={result.confidence:.3f}"
        )
        return 0


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = AsrTestClient()
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        shutdown_node(node)
