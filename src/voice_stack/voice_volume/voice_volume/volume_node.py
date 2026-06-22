from __future__ import annotations

import os
import re
import subprocess

import rclpy
from rclpy.node import Node
from voice_interfaces.srv import GetVolume, SetMute, SetVolume


class VoiceVolumeNode(Node):
    """Control the default PulseAudio/PipeWire sink through ROS services."""

    def __init__(self) -> None:
        super().__init__("voice_volume")
        self._set_service = self.create_service(
            SetVolume, "/voice/volume/set", self._handle_set_volume
        )
        self._get_service = self.create_service(
            GetVolume, "/voice/volume/get", self._handle_get_volume
        )
        self._mute_service = self.create_service(
            SetMute, "/voice/volume/mute", self._handle_set_mute
        )
        self.get_logger().info(
            "voice_volume ready: services=/voice/volume/set,get,mute"
        )

    @staticmethod
    def _run_pactl(*arguments: str) -> str:
        environment = os.environ.copy()
        environment["LC_ALL"] = "C"
        completed = subprocess.run(
            ["pactl", *arguments],
            check=True,
            capture_output=True,
            text=True,
            timeout=3.0,
            env=environment,
        )
        return completed.stdout.strip()

    @classmethod
    def _read_state(cls) -> tuple[float, bool]:
        volume_output = cls._run_pactl("get-sink-volume", "@DEFAULT_SINK@")
        match = re.search(r"(\d+)%", volume_output)
        if match is None:
            raise RuntimeError(f"无法解析 pactl 音量输出: {volume_output}")
        mute_output = cls._run_pactl("get-sink-mute", "@DEFAULT_SINK@")
        muted = mute_output.rsplit(":", 1)[-1].strip().lower() == "yes"
        return int(match.group(1)) / 100.0, muted

    def _handle_set_volume(
        self, request: SetVolume.Request, response: SetVolume.Response
    ) -> SetVolume.Response:
        try:
            volume = min(1.0, max(0.0, float(request.volume)))
            self._run_pactl(
                "set-sink-volume", "@DEFAULT_SINK@", f"{round(volume * 100)}%"
            )
            actual_volume, _ = self._read_state()
            response.success = True
            response.volume = actual_volume
            response.error_message = ""
            self.get_logger().info(f"系统音量已设置: {actual_volume:.0%}")
        except Exception as exc:
            response.success = False
            response.volume = 0.0
            response.error_message = str(exc)
            self.get_logger().error(f"设置系统音量失败: {exc}")
        return response

    def _handle_get_volume(
        self, request: GetVolume.Request, response: GetVolume.Response
    ) -> GetVolume.Response:
        del request
        try:
            response.volume, response.muted = self._read_state()
            response.success = True
            response.error_message = ""
        except Exception as exc:
            response.success = False
            response.volume = 0.0
            response.muted = False
            response.error_message = str(exc)
            self.get_logger().error(f"读取系统音量失败: {exc}")
        return response

    def _handle_set_mute(
        self, request: SetMute.Request, response: SetMute.Response
    ) -> SetMute.Response:
        try:
            self._run_pactl(
                "set-sink-mute", "@DEFAULT_SINK@", "1" if request.muted else "0"
            )
            _, actual_muted = self._read_state()
            response.success = True
            response.muted = actual_muted
            response.error_message = ""
            self.get_logger().info(f"系统静音状态已设置: {actual_muted}")
        except Exception as exc:
            response.success = False
            response.muted = False
            response.error_message = str(exc)
            self.get_logger().error(f"设置系统静音失败: {exc}")
        return response


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
