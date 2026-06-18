from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
import sherpa_onnx
from voice_interfaces.msg import VoiceAudioFrame, VoiceKwsEvent


class VoiceKwsNode(Node):
    """关键词唤醒节点，订阅 raw 音频并发布 KWS 事件。"""

    def __init__(self) -> None:
        super().__init__("voice_kws")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("kws_model_dir", "")
        self.declare_parameter("kws_keywords_file", "")
        self.declare_parameter("kws_keywords_score", 1.0)
        self.declare_parameter("kws_keywords_threshold", 0.25)
        self.declare_parameter("kws_num_trailing_blanks", 1)
        self.declare_parameter("num_threads", 2)
        self.declare_parameter("onnx_provider", "cpu")

        self._kws = self._create_kws()
        self._stream = self._kws.create_stream() if self._kws is not None else None

        self._pub = self.create_publisher(VoiceKwsEvent, "/voice/kws/events", 20)
        self._sub = self.create_subscription(
            VoiceAudioFrame, "/voice/audio/raw", self._on_audio, 30
        )
        self.get_logger().info("voice_kws ready: topic=/voice/kws/events")

    @staticmethod
    def _pick_onnx(model_dir: str, role: str) -> str:
        d = Path(model_dir)
        candidates = sorted(
            p
            for p in d.glob(f"{role}-*.onnx")
            if "int8" not in p.name and "chunk-16" in p.name
        )
        if not candidates:
            candidates = sorted(p for p in d.glob(f"{role}-*.onnx") if "int8" not in p.name)
        if not candidates:
            raise FileNotFoundError(f"缺少 {role} onnx: {model_dir}")
        return str(candidates[0])

    def _create_kws(self):
        model_dir = str(self.get_parameter("kws_model_dir").value or "").strip()
        keywords_file = str(self.get_parameter("kws_keywords_file").value or "").strip()
        if not model_dir or not keywords_file:
            self.get_logger().warning("kws_model_dir / kws_keywords_file 未配置，KWS 停用")
            return None
        try:
            return sherpa_onnx.KeywordSpotter(
                tokens=f"{model_dir}/tokens.txt",
                encoder=self._pick_onnx(model_dir, "encoder"),
                decoder=self._pick_onnx(model_dir, "decoder"),
                joiner=self._pick_onnx(model_dir, "joiner"),
                keywords_file=keywords_file,
                keywords_score=float(self.get_parameter("kws_keywords_score").value),
                keywords_threshold=float(self.get_parameter("kws_keywords_threshold").value),
                num_trailing_blanks=int(self.get_parameter("kws_num_trailing_blanks").value),
                num_threads=int(self.get_parameter("num_threads").value),
                sample_rate=int(self.get_parameter("sample_rate").value),
                provider=str(self.get_parameter("onnx_provider").value),
            )
        except Exception as exc:
            self.get_logger().error(f"KWS 初始化失败: {exc}")
            return None

    def _on_audio(self, msg: VoiceAudioFrame) -> None:
        if self._kws is None or self._stream is None:
            return
        try:
            if msg.encoding == "float32":
                samples = np.frombuffer(bytes(msg.data), dtype=np.float32)
            elif msg.encoding == "pcm16":
                pcm = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32)
                samples = pcm / 32768.0
            else:
                return
            self._stream.accept_waveform(int(msg.sample_rate), samples)
            while self._kws.is_ready(self._stream):
                self._kws.decode_stream(self._stream)
                keyword = self._kws.get_result(self._stream).strip()
                if keyword:
                    self.get_logger().info(f"KWS 命中: {keyword}")
                    event = VoiceKwsEvent()
                    event.stamp = self.get_clock().now().to_msg()
                    event.keyword = keyword
                    event.score = 1.0
                    self._pub.publish(event)
                    self._kws.reset_stream(self._stream)
        except Exception as exc:
            self.get_logger().warning(f"KWS 处理失败: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceKwsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
