from __future__ import annotations

import uuid
from pathlib import Path

import numpy as np
import rclpy
import sherpa_onnx
from rclpy.action import ActionClient
from rclpy.node import Node
from voice_interfaces.action import PlayAudio
from voice_interfaces.msg import VoiceAudioFrame
from voice_interfaces.srv import SynthesizeSpeech


class VoiceTtsNode(Node):
    """TTS 服务节点，合成后转发给 playback action。"""

    def __init__(self) -> None:
        super().__init__("voice_tts")
        self.declare_parameter("tts_model_dir", "")
        self.declare_parameter("tts_speaker_id", 0)
        self.declare_parameter("tts_speed", 1.0)
        self.declare_parameter("tts_volume", 1.0)
        self.declare_parameter("tts_max_chars_per_chunk", 80)
        self.declare_parameter("tts_sentence_pause", 0.40)
        self.declare_parameter("tts_clause_pause", 0.15)
        self.declare_parameter("onnx_provider", "cpu")
        self.declare_parameter("num_threads", 2)

        self._tts = self._init_local_tts()
        self._play_client = ActionClient(self, PlayAudio, "/voice/playback/play")
        self._srv = self.create_service(
            SynthesizeSpeech, "/voice/tts/synthesize", self._handle_synthesize
        )
        ready = self._tts is not None
        self.get_logger().info(
            f"voice_tts ready: srv=/voice/tts/synthesize model_loaded={ready}"
        )

    def _ensure_tts_ready(self) -> bool:
        if self._tts is not None:
            return True
        self._tts = self._init_local_tts()
        return self._tts is not None

    @staticmethod
    def _float32_to_pcm16(samples: np.ndarray) -> bytes:
        clamped = np.clip(samples, -1.0, 1.0)
        return (clamped * 32767.0).astype(np.int16).tobytes()

    def _init_local_tts(self):
        model_dir = str(self.get_parameter("tts_model_dir").value or "").strip()
        if not model_dir:
            self.get_logger().warning("未配置 tts_model_dir，本地 TTS 不可用")
            return None
        self.get_logger().info(f"加载 TTS 模型: {model_dir}")
        try:
            model_path = Path(model_dir)
            onnx_files = sorted(model_path.glob("*.onnx"))
            if not onnx_files:
                raise FileNotFoundError(f"未找到 ONNX 模型: {model_dir}/*.onnx")
            onnx_model = str(onnx_files[0])
            config = sherpa_onnx.OfflineTtsConfig(
                model=sherpa_onnx.OfflineTtsModelConfig(
                    vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                        model=onnx_model,
                        lexicon=f"{model_dir}/lexicon.txt",
                        tokens=f"{model_dir}/tokens.txt",
                    ),
                    num_threads=int(self.get_parameter("num_threads").value),
                    provider=str(self.get_parameter("onnx_provider").value),
                )
            )
            return sherpa_onnx.OfflineTts(config=config)
        except Exception as exc:
            self.get_logger().error(f"本地 TTS 初始化失败: {exc}")
            return None

    def _split_text(self, text: str) -> list[tuple[str, str]]:
        sentence_end = set("。！？.!?\n")
        clause_sep = set(",，;；:：、—")
        cleaned = "".join(text.strip().split())
        if not cleaned:
            return []
        max_len = int(self.get_parameter("tts_max_chars_per_chunk").value)
        segments: list[tuple[str, str]] = []
        buf: list[str] = []
        for ch in cleaned:
            buf.append(ch)
            if ch in sentence_end:
                seg = "".join(buf).strip()
                if seg:
                    segments.append((seg, "sentence"))
                buf = []
            elif ch in clause_sep:
                seg = "".join(buf).strip()
                if seg:
                    segments.append((seg, "clause"))
                buf = []
        if buf:
            seg = "".join(buf).strip()
            if seg:
                segments.append((seg, "none"))
        result: list[tuple[str, str]] = []
        for seg_text, pause_type in segments:
            if len(seg_text) <= max_len:
                result.append((seg_text, pause_type))
                continue
            start = 0
            while start < len(seg_text):
                chunk = seg_text[start : start + max_len]
                start += max_len
                p = pause_type if start >= len(seg_text) else "clause"
                result.append((chunk, p))
        return result

    def _generate_local_tts(self, text: str) -> tuple[np.ndarray, int]:
        if self._tts is None:
            raise RuntimeError("本地 TTS 未初始化")
        segments = self._split_text(text)
        if not segments:
            return np.array([], dtype=np.float32), 16000
        combined: list[np.ndarray] = []
        sample_rate = 16000
        sentence_pause = None
        clause_pause = None
        for seg_text, pause_type in segments:
            audio = self._tts.generate(
                seg_text,
                sid=int(self.get_parameter("tts_speaker_id").value),
                speed=float(self.get_parameter("tts_speed").value),
            )
            sample_rate = int(audio.sample_rate)
            if sentence_pause is None:
                sentence_pause = np.zeros(
                    int(sample_rate * float(self.get_parameter("tts_sentence_pause").value)),
                    dtype=np.float32,
                )
                clause_pause = np.zeros(
                    int(sample_rate * float(self.get_parameter("tts_clause_pause").value)),
                    dtype=np.float32,
                )
            combined.append(np.asarray(audio.samples, dtype=np.float32))
            if pause_type == "sentence" and sentence_pause is not None:
                combined.append(sentence_pause)
            elif pause_type == "clause" and clause_pause is not None:
                combined.append(clause_pause)
        if not combined:
            return np.array([], dtype=np.float32), sample_rate
        out = np.concatenate(combined)
        gain = float(self.get_parameter("tts_volume").value)
        if gain != 1.0:
            out = np.clip(out * gain, -1.0, 1.0)
        return out, sample_rate

    def _build_play_goal(self, samples: np.ndarray, sample_rate: int, priority: int) -> PlayAudio.Goal:
        pcm_bytes = self._float32_to_pcm16(samples)
        chunk = VoiceAudioFrame()
        chunk.encoding = "pcm16"
        chunk.sample_rate = int(sample_rate)
        chunk.channels = 1
        chunk.data = list(pcm_bytes)
        goal = PlayAudio.Goal()
        goal.chunks = [chunk]
        goal.priority = int(priority)
        goal.preempt_lower_priority = True
        return goal

    def _handle_synthesize(self, request: SynthesizeSpeech.Request, response: SynthesizeSpeech.Response):
        try:
            if not self._ensure_tts_ready():
                raise RuntimeError("本地 TTS 未初始化")
            samples, sample_rate = self._generate_local_tts(request.text)
            if len(samples) == 0:
                raise RuntimeError("TTS 未生成有效音频")
            if not self._play_client.wait_for_server(timeout_sec=1.0):
                raise RuntimeError("playback action 不可用")
            goal = self._build_play_goal(samples, sample_rate, int(request.priority))
            self._play_client.send_goal_async(goal)
            response.accepted = True
            response.request_id = str(uuid.uuid4())
            response.estimated_duration_sec = float(len(samples) / float(sample_rate))
            response.error_message = ""
        except Exception as exc:
            response.accepted = False
            response.request_id = ""
            response.estimated_duration_sec = 0.0
            response.error_message = str(exc)
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceTtsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
