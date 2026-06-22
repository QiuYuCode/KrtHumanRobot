from __future__ import annotations

import rclpy
from rclpy.node import Node
from voice_interfaces.srv import GetVolume, SetMute, SetVolume

from voice_test_tools.common import call_service, shutdown_node


class VolumeTestClient(Node):
    """Get or change the system playback volume through ROS services."""

    def __init__(self) -> None:
        super().__init__("volume_test_client")
        self.declare_parameter("operation", "get")
        self.declare_parameter("volume", 0.5)
        self.declare_parameter("timeout_sec", 5.0)
        self.get_client = self.create_client(GetVolume, "/voice/volume/get")
        self.set_client = self.create_client(SetVolume, "/voice/volume/set")
        self.mute_client = self.create_client(SetMute, "/voice/volume/mute")

    def run(self) -> int:
        operation = str(self.get_parameter("operation").value).strip().lower()
        timeout = float(self.get_parameter("timeout_sec").value)
        if operation == "get":
            return self._get(timeout)
        if operation == "set":
            return self._set(timeout)
        if operation in {"mute", "unmute"}:
            return self._mute(operation == "mute", timeout)
        self.get_logger().error(
            f"不支持的 operation={operation!r}，可选 get/set/mute/unmute"
        )
        return 1

    def _get(self, timeout: float) -> int:
        response = call_service(
            self, self.get_client, GetVolume.Request(), timeout
        )
        if not response.success:
            self.get_logger().error(f"读取音量失败: {response.error_message}")
            return 1
        self.get_logger().info(
            f"音量读取成功: volume={response.volume:.0%} muted={response.muted}"
        )
        return 0

    def _set(self, timeout: float) -> int:
        request = SetVolume.Request()
        request.volume = float(self.get_parameter("volume").value)
        response = call_service(self, self.set_client, request, timeout)
        if not response.success:
            self.get_logger().error(f"设置音量失败: {response.error_message}")
            return 1
        self.get_logger().info(f"音量设置成功: volume={response.volume:.0%}")
        return 0

    def _mute(self, muted: bool, timeout: float) -> int:
        request = SetMute.Request()
        request.muted = muted
        response = call_service(self, self.mute_client, request, timeout)
        if not response.success:
            self.get_logger().error(f"设置静音失败: {response.error_message}")
            return 1
        self.get_logger().info(f"静音设置成功: muted={response.muted}")
        return 0


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = VolumeTestClient()
    try:
        return node.run()
    except (RuntimeError, TimeoutError) as exc:
        node.get_logger().error(str(exc))
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        shutdown_node(node)
