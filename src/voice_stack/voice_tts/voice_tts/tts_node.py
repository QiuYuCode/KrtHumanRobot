from __future__ import annotations

import base64
import hashlib
import hmac
import json
import time
import uuid
import wave
from email.utils import formatdate
from io import BytesIO
from pathlib import Path
from urllib.parse import urlencode

import numpy as np
import rclpy
import sherpa_onnx
import websocket  # type: ignore[import-not-found]
from rclpy.action import ActionClient
from rclpy.node import Node
from voice_interfaces.action import PlayAudio
from voice_interfaces.msg import VoiceAudioFrame
from voice_interfaces.srv import SynthesizeSpeech


class VoiceTtsNode(Node):
    """TTS 服务节点，合成后转发给 playback action。"""

    def __init__(self) -> None:
        super().__init__("voice_tts")
        self.declare_parameter("tts_backend", "iflytek_cloud")
        self.declare_parameter("cloud_tts_fallback_to_local", True)
        self.declare_parameter("tts_model_dir", "")
        self.declare_parameter("tts_speaker_id", 0)
        self.declare_parameter("tts_speed", 1.0)
        self.declare_parameter("tts_volume", 1.0)
        self.declare_parameter("tts_max_chars_per_chunk", 80)
        self.declare_parameter("tts_sentence_pause", 0.40)
        self.declare_parameter("tts_clause_pause", 0.15)
        self.declare_parameter("onnx_provider", "cpu")
        self.declare_parameter("num_threads", 2)
        self.declare_parameter("iflytek_tts_app_id", "")
        self.declare_parameter("iflytek_tts_api_key", "")
        self.declare_parameter("iflytek_tts_api_secret", "")
        self.declare_parameter("iflytek_tts_vcn", "x4_yezi")
        self.declare_parameter("iflytek_tts_aue", "raw")
        self.declare_parameter("iflytek_tts_auf", "audio/L16;rate=16000")
        self.declare_parameter("iflytek_tts_speed", 50)
        self.declare_parameter("iflytek_tts_tte", "UTF8")
        self.declare_parameter("iflytek_tts_request_text_encoding", "utf-8")
        self.declare_parameter("iflytek_tts_max_bytes", 8000)

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
                sentence_pause_sec = float(
                    self.get_parameter("tts_sentence_pause").value
                )
                clause_pause_sec = float(
                    self.get_parameter("tts_clause_pause").value
                )
                sentence_pause = np.zeros(
                    int(sample_rate * sentence_pause_sec),
                    dtype=np.float32,
                )
                clause_pause = np.zeros(
                    int(sample_rate * clause_pause_sec),
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

    @staticmethod
    def _build_ws_url(api_key: str, api_secret: str) -> str:
        base_url = "wss://tts-api.xfyun.cn/v2/tts"
        host = "tts-api.xfyun.cn"
        path = "/v2/tts"
        date = formatdate(timeval=None, localtime=False, usegmt=True)
        signature_origin = f"host: {host}\ndate: {date}\nGET {path} HTTP/1.1"
        signature_sha = hmac.new(
            api_secret.encode("utf-8"),
            signature_origin.encode("utf-8"),
            digestmod=hashlib.sha256,
        ).digest()
        signature = base64.b64encode(signature_sha).decode("utf-8")
        authorization_origin = (
            f'api_key="{api_key}", algorithm="hmac-sha256", '
            f'headers="host date request-line", signature="{signature}"'
        )
        authorization = base64.b64encode(
            authorization_origin.encode("utf-8")
        ).decode("utf-8")
        query = urlencode({"host": host, "date": date, "authorization": authorization})
        return f"{base_url}?{query}"

    @staticmethod
    def _sample_rate_from_auf(auf: str) -> int:
        marker = "rate="
        if marker not in auf:
            return 16000
        try:
            return int(auf.split(marker, 1)[1].split(";", 1)[0])
        except ValueError:
            return 16000

    @staticmethod
    def _wav_to_pcm16(audio: bytes) -> tuple[bytes, int]:
        with wave.open(BytesIO(audio), "rb") as wav:
            sample_rate = int(wav.getframerate())
            channels = int(wav.getnchannels())
            sample_width = int(wav.getsampwidth())
            frames = wav.readframes(wav.getnframes())
        if sample_width != 2:
            raise RuntimeError(f"不支持的 WAV 采样宽度: {sample_width}")
        if channels == 1:
            return frames, sample_rate
        pcm = np.frombuffer(frames, dtype=np.int16).reshape(-1, channels)
        mono = np.mean(pcm.astype(np.float32), axis=1).astype(np.int16)
        return mono.tobytes(), sample_rate

    def _split_text_by_cloud_limit(self, text: str) -> list[str]:
        encoding = str(self.get_parameter("iflytek_tts_request_text_encoding").value)
        max_bytes = int(self.get_parameter("iflytek_tts_max_bytes").value)
        max_bytes = max(1, max_bytes)
        chunks: list[str] = []
        for segment, _pause_type in self._split_text(text):
            buf: list[str] = []
            buf_size = 0
            for ch in segment:
                ch_size = len(ch.encode(encoding))
                if buf and buf_size + ch_size > max_bytes:
                    chunks.append("".join(buf))
                    buf = []
                    buf_size = 0
                buf.append(ch)
                buf_size += ch_size
            if buf:
                chunks.append("".join(buf))
        return chunks

    def _generate_iflytek_tts(self, text: str) -> tuple[bytes, int]:
        app_id = str(self.get_parameter("iflytek_tts_app_id").value)
        api_key = str(self.get_parameter("iflytek_tts_api_key").value)
        api_secret = str(self.get_parameter("iflytek_tts_api_secret").value)
        if not (app_id and api_key and api_secret):
            raise RuntimeError("讯飞 TTS 密钥缺失")

        aue = str(self.get_parameter("iflytek_tts_aue").value)
        auf = str(self.get_parameter("iflytek_tts_auf").value)
        encoding = str(self.get_parameter("iflytek_tts_request_text_encoding").value)
        sample_rate = self._sample_rate_from_auf(auf)
        pcm_chunks: list[bytes] = []
        text_chunks = self._split_text_by_cloud_limit(text)
        if not text_chunks:
            return b"", sample_rate

        for text_chunk in text_chunks:
            ws = websocket.create_connection(
                self._build_ws_url(api_key, api_secret),
                timeout=10,
                http_no_proxy=["tts-api.xfyun.cn"],
            )
            try:
                ws.send(
                    json.dumps(
                        {
                            "common": {"app_id": app_id},
                            "business": {
                                "aue": aue,
                                "auf": auf,
                                "vcn": str(self.get_parameter("iflytek_tts_vcn").value),
                                "speed": int(
                                    self.get_parameter("iflytek_tts_speed").value
                                ),
                                "tte": str(self.get_parameter("iflytek_tts_tte").value),
                            },
                            "data": {
                                "status": 2,
                                "text": base64.b64encode(
                                    text_chunk.encode(encoding)
                                ).decode("utf-8"),
                            },
                        },
                        ensure_ascii=False,
                    )
                )
                audio_parts: list[bytes] = []
                while True:
                    resp = json.loads(ws.recv())
                    if int(resp.get("code", -1)) != 0:
                        raise RuntimeError(f"讯飞 TTS 错误: {resp}")
                    data = resp.get("data", {}) or {}
                    audio = data.get("audio", "")
                    if audio:
                        audio_parts.append(base64.b64decode(audio))
                    if int(data.get("status", 1)) == 2:
                        break
                audio_bytes = b"".join(audio_parts)
                if aue.lower() in ("raw", "pcm", "pcm16"):
                    pcm_chunks.append(audio_bytes)
                elif aue.lower() in ("lame", "mp3"):
                    raise RuntimeError("讯飞 TTS 返回 MP3，当前播放链路仅支持 PCM16")
                else:
                    pcm_bytes, wav_rate = self._wav_to_pcm16(audio_bytes)
                    sample_rate = wav_rate
                    pcm_chunks.append(pcm_bytes)
            finally:
                ws.close()
            time.sleep(0.02)

        return b"".join(pcm_chunks), sample_rate

    def _build_play_goal_from_pcm(
        self,
        pcm_bytes: bytes,
        sample_rate: int,
        priority: int,
    ) -> PlayAudio.Goal:
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

    def _build_play_goal(
        self,
        samples: np.ndarray,
        sample_rate: int,
        priority: int,
    ) -> PlayAudio.Goal:
        return self._build_play_goal_from_pcm(
            self._float32_to_pcm16(samples),
            sample_rate,
            priority,
        )

    def _synthesize_pcm16(self, text: str) -> tuple[bytes, int, str]:
        backend = (
            str(self.get_parameter("tts_backend").value).strip()
            or "iflytek_cloud"
        )
        if backend == "iflytek_cloud":
            try:
                pcm_bytes, sample_rate = self._generate_iflytek_tts(text)
                return pcm_bytes, sample_rate, "iflytek_cloud"
            except Exception as exc:
                if not bool(self.get_parameter("cloud_tts_fallback_to_local").value):
                    raise
                self.get_logger().warning(f"讯飞云 TTS 失败，回退本地 TTS: {exc}")
        elif backend != "local":
            if not bool(self.get_parameter("cloud_tts_fallback_to_local").value):
                raise RuntimeError(f"不支持的 TTS 后端: {backend}")
            self.get_logger().warning(f"不支持的 TTS 后端 {backend!r}，回退本地 TTS")

        if not self._ensure_tts_ready():
            raise RuntimeError("本地 TTS 未初始化")
        samples, sample_rate = self._generate_local_tts(text)
        return self._float32_to_pcm16(samples), sample_rate, "local"

    def _handle_synthesize(
        self,
        request: SynthesizeSpeech.Request,
        response: SynthesizeSpeech.Response,
    ):
        try:
            pcm_bytes, sample_rate, backend_used = self._synthesize_pcm16(request.text)
            if len(pcm_bytes) == 0:
                raise RuntimeError("TTS 未生成有效音频")
            if not self._play_client.wait_for_server(timeout_sec=1.0):
                raise RuntimeError("playback action 不可用")
            goal = self._build_play_goal_from_pcm(
                pcm_bytes, sample_rate, int(request.priority)
            )
            self._play_client.send_goal_async(goal)
            response.accepted = True
            response.request_id = str(uuid.uuid4())
            response.estimated_duration_sec = float(
                len(pcm_bytes) / 2.0 / float(sample_rate)
            )
            response.error_message = ""
            self.get_logger().info(
                "TTS 已提交播放: "
                f"backend={backend_used} "
                f"duration={response.estimated_duration_sec:.2f}s"
            )
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
