from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
import sherpa_onnx
from voice_interfaces.msg import VoiceAudioFrame, VoiceVadEvent


class VoiceAudioProcessNode(Node):
    """音频处理节点：VAD 事件发布 + processed 音频透传。"""

    def __init__(self) -> None:
        super().__init__("voice_audio_process")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("vad_model_path", "")
        self.declare_parameter("vad_threshold", 0.5)
        self.declare_parameter("vad_min_silence_duration", 0.25)
        self.declare_parameter("vad_min_speech_duration", 0.25)

        self._vad = self._create_vad()
        self._vad_window_size = 512
        if self._vad is not None:
            self._vad_window_size = int(self._vad.config.silero_vad.window_size)
        self._buffer = np.array([], dtype=np.float32)
        self._in_speech = False

        self._raw_sub = self.create_subscription(
            VoiceAudioFrame, "/voice/audio/raw", self._on_raw_audio, 30
        )
        self._processed_pub = self.create_publisher(
            VoiceAudioFrame, "/voice/audio/processed", 30
        )
        self._vad_pub = self.create_publisher(VoiceVadEvent, "/voice/vad/events", 20)
        self.get_logger().info("voice_audio_process ready: topics=/voice/vad/events,/voice/audio/processed")

    def _create_vad(self):
        model_path = str(self.get_parameter("vad_model_path").value or "").strip()
        if not model_path:
            self.get_logger().warning("vad_model_path 未配置，VAD 停用")
            return None
        try:
            vad_config = sherpa_onnx.VadModelConfig()
            vad_config.sample_rate = int(self.get_parameter("sample_rate").value)
            vad_config.silero_vad.model = model_path
            vad_config.silero_vad.threshold = float(self.get_parameter("vad_threshold").value)
            vad_config.silero_vad.min_silence_duration = float(
                self.get_parameter("vad_min_silence_duration").value
            )
            vad_config.silero_vad.min_speech_duration = float(
                self.get_parameter("vad_min_speech_duration").value
            )
            return sherpa_onnx.VoiceActivityDetector(vad_config, buffer_size_in_seconds=30)
        except Exception as exc:
            self.get_logger().error(f"VAD 初始化失败: {exc}")
            return None

    def _publish_vad_event(self, event_type: str, confidence: float = 1.0) -> None:
        event = VoiceVadEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.event_type = event_type
        event.confidence = float(confidence)
        self._vad_pub.publish(event)

    def _on_raw_audio(self, msg: VoiceAudioFrame) -> None:
        self._processed_pub.publish(msg)
        if self._vad is None:
            return
        if msg.encoding == "float32":
            samples = np.frombuffer(bytes(msg.data), dtype=np.float32)
        elif msg.encoding == "pcm16":
            pcm = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32)
            samples = pcm / 32768.0
        else:
            return
        self._buffer = np.concatenate([self._buffer, samples])
        while len(self._buffer) >= self._vad_window_size:
            block = self._buffer[: self._vad_window_size]
            self._buffer = self._buffer[self._vad_window_size :]
            self._vad.accept_waveform(block)
            detected = bool(self._vad.is_speech_detected())
            if detected and not self._in_speech:
                self._in_speech = True
                self._publish_vad_event("speech_start")
            if not detected and self._in_speech:
                self._in_speech = False
                self._publish_vad_event("speech_end")
        while not self._vad.empty():
            self._vad.pop()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceAudioProcessNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
