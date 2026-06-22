from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from voice_interfaces.srv import PlayMedia

from voice_test_tools.common import call_service, shutdown_node


class PlayTestClient(Node):
    """Request playback of one local WAV media file."""

    def __init__(self) -> None:
        super().__init__("play_test_client")
        self.declare_parameter("file_path", "")
        self.declare_parameter("priority", 10)
        self.declare_parameter("preempt_lower_priority", True)
        self.declare_parameter("timeout_sec", 10.0)
        self.client = self.create_client(PlayMedia, "/voice/media/play")

    def run(self) -> int:
        file_path = Path(str(self.get_parameter("file_path").value)).expanduser()
        if not file_path.is_absolute() or not file_path.is_file():
            self.get_logger().error(f"必须提供存在的 WAV 绝对路径: {file_path}")
            return 1

        request = PlayMedia.Request()
        request.file_path = str(file_path)
        request.priority = int(self.get_parameter("priority").value)
        request.preempt_lower_priority = bool(
            self.get_parameter("preempt_lower_priority").value
        )
        timeout = float(self.get_parameter("timeout_sec").value)
        response = call_service(self, self.client, request, timeout)
        if not response.accepted:
            self.get_logger().error(f"媒体播放测试失败: {response.error_message}")
            return 1
        self.get_logger().info(
            f"媒体播放测试成功: request_id={response.request_id} "
            f"duration={response.duration_sec:.2f}s"
        )
        return 0


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = PlayTestClient()
    try:
        return node.run()
    except (RuntimeError, TimeoutError) as exc:
        node.get_logger().error(str(exc))
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        shutdown_node(node)
