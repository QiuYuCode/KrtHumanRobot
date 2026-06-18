from __future__ import annotations

from typing import Any

import rclpy
import sounddevice as sd
from rclpy.node import Node
from voice_interfaces.msg import VoiceAudioFrame

from voice_audio_capture.audio_devices import configure_audio_devices


class VoiceCaptureNode(Node):
    """麦克风采集节点，发布 `/voice/audio/raw`。"""

    def __init__(self) -> None:
        super().__init__("voice_audio_capture")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("chunk_size", 1600)
        self.declare_parameter("channels", 1)
        self.declare_parameter("dtype", "float32")
        self.declare_parameter("frame_id", "mic")
        self.declare_parameter("input_device_hint", "")

        configure_audio_devices(
            input_hint=str(self.get_parameter("input_device_hint").value),
            configure_input=True,
            configure_output=False,
            log_info=self.get_logger().info,
            log_warning=self.get_logger().warning,
        )

        self._pub = self.create_publisher(VoiceAudioFrame, "/voice/audio/raw", 30)
        self._stream: sd.RawInputStream | None = None
        self._start_stream()
        self.get_logger().info("voice_audio_capture ready: topic=/voice/audio/raw")

    def _start_stream(self) -> None:
        sample_rate = int(self.get_parameter("sample_rate").value)
        chunk_size = int(self.get_parameter("chunk_size").value)
        channels = int(self.get_parameter("channels").value)
        dtype = str(self.get_parameter("dtype").value)

        def _callback(indata: bytes, frames: int, time_info: Any, status: Any) -> None:
            del frames, time_info
            if status:
                self.get_logger().warning(f"audio callback status: {status}")
            msg = VoiceAudioFrame()
            msg.stamp = self.get_clock().now().to_msg()
            msg.frame_id = str(self.get_parameter("frame_id").value)
            msg.encoding = dtype
            msg.sample_rate = sample_rate
            msg.channels = channels
            msg.data = list(bytes(indata))
            self._pub.publish(msg)

        self._stream = sd.RawInputStream(
            samplerate=sample_rate,
            channels=channels,
            dtype=dtype,
            blocksize=chunk_size,
            callback=_callback,
        )
        self._stream.start()

    def destroy_node(self) -> bool:
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
