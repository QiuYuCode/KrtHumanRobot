from __future__ import annotations

import rclpy
from rclpy.node import Node
from voice_interfaces.srv import SynthesizeSpeech

from voice_test_tools.common import call_service, shutdown_node


class TtsTestClient(Node):
    """Request one TTS synthesis and playback operation."""

    def __init__(self) -> None:
        super().__init__("tts_test_client")
        self.declare_parameter("text", "你好，这是语音合成代码测试")
        self.declare_parameter("language", "zh-CN")
        self.declare_parameter("style", "")
        self.declare_parameter("priority", 0)
        self.declare_parameter("timeout_sec", 30.0)
        self.client = self.create_client(
            SynthesizeSpeech, "/voice/tts/synthesize"
        )

    def run(self) -> int:
        request = SynthesizeSpeech.Request()
        request.text = str(self.get_parameter("text").value)
        request.language = str(self.get_parameter("language").value)
        request.style = str(self.get_parameter("style").value)
        request.priority = int(self.get_parameter("priority").value)
        timeout = float(self.get_parameter("timeout_sec").value)
        response = call_service(self, self.client, request, timeout)
        if not response.accepted:
            self.get_logger().error(f"TTS 测试失败: {response.error_message}")
            return 1
        self.get_logger().info(
            f"TTS 测试成功: request_id={response.request_id} "
            f"estimated_duration={response.estimated_duration_sec:.2f}s"
        )
        return 0


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = TtsTestClient()
    try:
        return node.run()
    except (RuntimeError, TimeoutError) as exc:
        node.get_logger().error(str(exc))
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        shutdown_node(node)
