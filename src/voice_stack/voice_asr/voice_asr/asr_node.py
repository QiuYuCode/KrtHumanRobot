from __future__ import annotations

import base64
import hashlib
import hmac
import json
import threading
import time
from email.utils import formatdate
from urllib.parse import urlencode

import numpy as np
import rclpy
import sherpa_onnx
import websocket  # type: ignore[import-not-found]
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from voice_interfaces.action import RecognizeStream
from voice_interfaces.msg import VoiceAudioFrame, VoiceVadEvent
from voice_interfaces.srv import RecognizeSpeech


class VoiceAsrNode(Node):
    """ASR 节点：提供整段识别服务与本地流式识别 action。"""

    def __init__(self) -> None:
        super().__init__("voice_asr")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("asr_backend", "local")
        self.declare_parameter("cloud_asr_fallback_to_local", True)
        self.declare_parameter("asr_model_dir", "")
        self.declare_parameter("num_threads", 2)
        self.declare_parameter("onnx_provider", "cpu")
        self.declare_parameter("iflytek_iat_app_id", "")
        self.declare_parameter("iflytek_iat_api_key", "")
        self.declare_parameter("iflytek_iat_api_secret", "")
        self.declare_parameter("iflytek_iat_language", "zh_cn")
        self.declare_parameter("iflytek_iat_domain", "iat")
        self.declare_parameter("iflytek_iat_accent", "mandarin")
        self.declare_parameter("iflytek_iat_eos_ms", 1800)
        self.declare_parameter("iflytek_iat_ptt", 1)
        self.declare_parameter("iflytek_iat_audio_format", "audio/L16;rate=16000")
        self.declare_parameter("iflytek_iat_encoding", "raw")
        self.declare_parameter("cloud_asr_strategy", "streaming")
        self.declare_parameter("asr_stream_timeout", 12.0)
        self.declare_parameter("asr_stream_preroll_seconds", 0.5)

        self._cb_group = ReentrantCallbackGroup()
        self._stream_lock = threading.RLock()
        self._stream_end_event = threading.Event()
        self._audio_ring_buffer: list[tuple[float, float, int, np.ndarray]] = []
        self._vad_speech_active = False
        self._stream_active = False
        self._stream_speech_active = False
        self._stream = None
        self._stream_goal_handle = None
        self._stream_text = ""
        self._stream_started_at = 0.0
        self._stream_last_fed_audio_time = 0.0

        self._recognizer = self._create_local_asr()
        self._srv = self.create_service(
            RecognizeSpeech,
            "/voice/asr/recognize",
            self._handle_recognize,
            callback_group=self._cb_group,
        )
        self._audio_sub = self.create_subscription(
            VoiceAudioFrame,
            "/voice/audio/raw",
            self._on_stream_audio,
            30,
            callback_group=self._cb_group,
        )
        self._vad_sub = self.create_subscription(
            VoiceVadEvent,
            "/voice/vad/events",
            self._on_stream_vad,
            20,
            callback_group=self._cb_group,
        )
        self._action_server = ActionServer(
            self,
            RecognizeStream,
            "/voice/asr/stream",
            execute_callback=self._handle_stream,
            goal_callback=self._stream_goal_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            "voice_asr ready: srv=/voice/asr/recognize action=/voice/asr/stream"
        )

    def _create_local_asr(self):
        model_dir = str(self.get_parameter("asr_model_dir").value or "").strip()
        if not model_dir:
            self.get_logger().warning("asr_model_dir 未配置，本地 ASR 停用")
            return None
        try:
            return sherpa_onnx.OnlineRecognizer.from_transducer(
                tokens=f"{model_dir}/tokens.txt",
                encoder=f"{model_dir}/encoder-epoch-99-avg-1.onnx",
                decoder=f"{model_dir}/decoder-epoch-99-avg-1.onnx",
                joiner=f"{model_dir}/joiner-epoch-99-avg-1.onnx",
                num_threads=int(self.get_parameter("num_threads").value),
                sample_rate=int(self.get_parameter("sample_rate").value),
                enable_endpoint_detection=True,
                provider=str(self.get_parameter("onnx_provider").value),
            )
        except Exception as exc:
            self.get_logger().error(f"本地 ASR 初始化失败: {exc}")
            return None

    @staticmethod
    def _build_ws_url(api_key: str, api_secret: str) -> str:
        base_url = "wss://iat-api.xfyun.cn/v2/iat"
        host = "iat-api.xfyun.cn"
        path = "/v2/iat"
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
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode("utf-8")
        query = urlencode({"host": host, "date": date, "authorization": authorization})
        return f"{base_url}?{query}"

    @staticmethod
    def _extract_text(result_obj: dict) -> str:
        words: list[str] = []
        for item in result_obj.get("ws", []):
            for cw in item.get("cw", []):
                text = cw.get("w", "")
                if text:
                    words.append(text)
        return "".join(words).strip()

    def _audio_to_float32(self, audio: VoiceAudioFrame) -> np.ndarray:
        raw = bytes(audio.data)
        if audio.encoding == "float32":
            return np.frombuffer(raw, dtype=np.float32)
        if audio.encoding == "pcm16":
            pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
            return pcm / 32768.0
        raise RuntimeError(f"不支持的音频编码: {audio.encoding}")

    @staticmethod
    def _msg_time_seconds(msg_stamp) -> float:
        sec = int(getattr(msg_stamp, "sec", 0) or 0)
        nanosec = int(getattr(msg_stamp, "nanosec", 0) or 0)
        if sec == 0 and nanosec == 0:
            return 0.0
        return float(sec) + float(nanosec) / 1_000_000_000.0

    def _audio_frame_time(self, msg: VoiceAudioFrame, received_at: float) -> float:
        stamp_time = self._msg_time_seconds(msg.stamp)
        return stamp_time if stamp_time > 0.0 else received_at

    def _append_audio_ring_locked(
        self,
        start_time: float,
        end_time: float,
        sample_rate: int,
        samples: np.ndarray,
    ) -> None:
        self._audio_ring_buffer.append((start_time, end_time, sample_rate, samples))
        if len(self._audio_ring_buffer) > 1:
            self._audio_ring_buffer.sort(key=lambda item: item[0])

        preroll = max(0.0, float(self.get_parameter("asr_stream_preroll_seconds").value))
        keep_seconds = max(1.0, preroll + 0.5)
        cutoff = end_time - keep_seconds
        while self._audio_ring_buffer and self._audio_ring_buffer[0][1] < cutoff:
            self._audio_ring_buffer.pop(0)

    def _recognize_local(self, samples: np.ndarray) -> tuple[str, float]:
        if self._recognizer is None:
            raise RuntimeError("本地 ASR 不可用")
        stream = self._recognizer.create_stream()
        sample_rate = int(self.get_parameter("sample_rate").value)
        stream.accept_waveform(sample_rate, samples)
        while self._recognizer.is_ready(stream):
            self._recognizer.decode_stream(stream)
        text = self._recognizer.get_result(stream).strip()
        return text, 1.0 if text else 0.0

    def _resolve_backend(self, requested: str) -> str:
        return requested.strip() or str(self.get_parameter("asr_backend").value)

    def _stream_goal_callback(self, goal_request: RecognizeStream.Goal) -> GoalResponse:
        backend = self._resolve_backend(goal_request.backend)
        if backend != "local":
            self.get_logger().warning(
                f"流式 ASR 暂只支持 local，拒绝 backend={backend}"
            )
            return GoalResponse.REJECT
        if self._recognizer is None:
            self.get_logger().warning("流式 ASR 拒绝: 本地 ASR 不可用")
            return GoalResponse.REJECT
        with self._stream_lock:
            if self._stream_active:
                self.get_logger().warning("流式 ASR 拒绝: 已有会话进行中")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _publish_stream_feedback(self, text: str) -> None:
        goal_handle = self._stream_goal_handle
        if goal_handle is None:
            return
        feedback = RecognizeStream.Feedback()
        feedback.partial_text = text
        feedback.partial_confidence = 1.0 if text else 0.0
        goal_handle.publish_feedback(feedback)

    def _feed_stream_audio_locked(
        self,
        sample_rate: int,
        samples: np.ndarray,
        start_time: float,
        end_time: float,
    ) -> float:
        if self._stream is None or self._recognizer is None:
            return 0.0
        if end_time <= self._stream_last_fed_audio_time:
            return 0.0

        feed_samples = samples
        feed_start_time = start_time
        if start_time < self._stream_last_fed_audio_time:
            duration = max(0.0, end_time - start_time)
            if duration <= 0.0:
                return 0.0
            offset_seconds = self._stream_last_fed_audio_time - start_time
            offset_samples = int(round(offset_seconds * float(sample_rate)))
            if offset_samples >= len(samples):
                return 0.0
            feed_samples = samples[offset_samples:]
            feed_start_time = start_time + (offset_samples / float(sample_rate))

        if len(feed_samples) == 0:
            return 0.0

        self._stream.accept_waveform(sample_rate, feed_samples)
        while self._recognizer.is_ready(self._stream):
            self._recognizer.decode_stream(self._stream)
        text = self._recognizer.get_result(self._stream).strip()
        if text and text != self._stream_text:
            self._stream_text = text
            self._publish_stream_feedback(text)

        fed_seconds = len(feed_samples) / float(sample_rate)
        self._stream_last_fed_audio_time = max(
            self._stream_last_fed_audio_time,
            feed_start_time + fed_seconds,
        )
        return fed_seconds

    def _feed_stream_preroll_locked(self, reason: str) -> tuple[int, float]:
        if self._stream is None:
            return 0, 0.0
        preroll = max(0.0, float(self.get_parameter("asr_stream_preroll_seconds").value))
        if preroll <= 0.0:
            self.get_logger().info(f"流式 ASR {reason}: preroll 已禁用")
            return 0, 0.0

        now = time.time()
        latest_end = self._audio_ring_buffer[-1][1] if self._audio_ring_buffer else now
        cutoff = latest_end - preroll
        frame_count = 0
        fed_seconds = 0.0
        try:
            for start_time, end_time, sample_rate, samples in self._audio_ring_buffer:
                if end_time <= cutoff:
                    continue
                fed = self._feed_stream_audio_locked(
                    sample_rate, samples, start_time, end_time
                )
                if fed > 0.0:
                    frame_count += 1
                    fed_seconds += fed
        except Exception as exc:
            self.get_logger().warning(f"流式 ASR {reason} preroll 补喂失败: {exc}")
            return frame_count, fed_seconds

        self.get_logger().info(
            f"流式 ASR {reason}: speech_active={self._stream_speech_active} "
            f"preroll_frames={frame_count} preroll_seconds={fed_seconds:.3f}"
        )
        return frame_count, fed_seconds

    def _on_stream_audio(self, msg: VoiceAudioFrame) -> None:
        received_at = time.time()
        try:
            samples = self._audio_to_float32(msg)
            sample_rate = int(msg.sample_rate or self.get_parameter("sample_rate").value)
        except Exception as exc:
            self.get_logger().warning(f"流式 ASR 音频解码失败: {exc}")
            return

        start_time = self._audio_frame_time(msg, received_at)
        duration = len(samples) / float(sample_rate) if sample_rate > 0 else 0.0
        end_time = start_time + duration

        with self._stream_lock:
            self._append_audio_ring_locked(start_time, end_time, sample_rate, samples)
            if (
                not self._stream_active
                or not self._stream_speech_active
                or self._stream is None
            ):
                return
            try:
                self._feed_stream_audio_locked(
                    sample_rate, samples, start_time, end_time
                )
            except Exception as exc:
                self.get_logger().warning(f"流式 ASR 音频处理失败: {exc}")
                return

    def _on_stream_vad(self, msg: VoiceVadEvent) -> None:
        with self._stream_lock:
            if msg.event_type == "speech_start":
                self._vad_speech_active = True
            elif msg.event_type == "speech_end":
                self._vad_speech_active = False

            if not self._stream_active:
                self.get_logger().info(
                    f"流式 ASR VAD {msg.event_type}: stream_active=False, "
                    f"cached_speech_active={self._vad_speech_active}"
                )
                return
            if msg.event_type == "speech_start":
                self._stream_speech_active = True
                self._feed_stream_preroll_locked("speech_start")
                self.get_logger().info(
                    "流式 ASR VAD speech_start: stream_active=True"
                )
            elif msg.event_type == "speech_end" and self._stream_speech_active:
                self._stream_speech_active = False
                self._stream_end_event.set()
                self.get_logger().info(
                    "流式 ASR VAD speech_end: stream_active=True end_event=True"
                )

    def _reset_stream_state(self) -> None:
        with self._stream_lock:
            self._stream_active = False
            self._stream_speech_active = False
            self._stream = None
            self._stream_goal_handle = None
            self._stream_text = ""
            self._stream_started_at = 0.0
            self._stream_last_fed_audio_time = 0.0
            self._stream_end_event.clear()

    def _recognize_cloud(self, samples: np.ndarray) -> tuple[str, float]:
        app_id = str(self.get_parameter("iflytek_iat_app_id").value)
        api_key = str(self.get_parameter("iflytek_iat_api_key").value)
        api_secret = str(self.get_parameter("iflytek_iat_api_secret").value)
        if not (app_id and api_key and api_secret):
            raise RuntimeError("讯飞 ASR 密钥缺失")
        ws = websocket.create_connection(
            self._build_ws_url(api_key, api_secret),
            timeout=10,
            http_no_proxy=["iat-api.xfyun.cn"],
        )
        try:
            pcm = (np.clip(samples, -1.0, 1.0) * 32767.0).astype(np.int16).tobytes()
            frame_bytes = 1280
            chunks = [pcm[i : i + frame_bytes] for i in range(0, len(pcm), frame_bytes)]
            if not chunks:
                return "", 0.0
            ws.send(
                json.dumps(
                    {
                        "common": {"app_id": app_id},
                        "business": {
                            "language": str(self.get_parameter("iflytek_iat_language").value),
                            "domain": str(self.get_parameter("iflytek_iat_domain").value),
                            "accent": str(self.get_parameter("iflytek_iat_accent").value),
                            "vad_eos": int(self.get_parameter("iflytek_iat_eos_ms").value),
                            "ptt": int(self.get_parameter("iflytek_iat_ptt").value),
                        },
                        "data": {
                            "status": 0,
                            "format": str(self.get_parameter("iflytek_iat_audio_format").value),
                            "audio": base64.b64encode(chunks[0]).decode("utf-8"),
                            "encoding": str(self.get_parameter("iflytek_iat_encoding").value),
                        },
                    }
                )
            )
            strategy = str(self.get_parameter("cloud_asr_strategy").value)
            if strategy == "streaming":
                for i, chunk in enumerate(chunks[1:], start=1):
                    status = 1 if i < len(chunks) - 1 else 2
                    ws.send(
                        json.dumps(
                            {
                                "data": {
                                    "status": status,
                                    "format": str(self.get_parameter("iflytek_iat_audio_format").value),
                                    "audio": base64.b64encode(chunk).decode("utf-8"),
                                    "encoding": str(self.get_parameter("iflytek_iat_encoding").value),
                                }
                            }
                        )
                    )
                    if status != 2:
                        time.sleep(0.04)
            else:
                ws.send(
                    json.dumps(
                        {
                            "data": {
                                "status": 2,
                                "format": str(self.get_parameter("iflytek_iat_audio_format").value),
                                "audio": base64.b64encode(b"".join(chunks[1:])).decode("utf-8"),
                                "encoding": str(self.get_parameter("iflytek_iat_encoding").value),
                            }
                        }
                    )
                )

            final_text = ""
            while True:
                resp = json.loads(ws.recv())
                if int(resp.get("code", -1)) != 0:
                    raise RuntimeError(f"讯飞 ASR 错误: {resp}")
                data = resp.get("data", {}) or {}
                result = data.get("result", {}) or {}
                piece = self._extract_text(result)
                if piece:
                    final_text = piece
                if int(data.get("status", 1)) == 2 or bool(result.get("ls")):
                    break
            return final_text, 1.0 if final_text else 0.0
        finally:
            ws.close()

    def _handle_recognize(
        self, request: RecognizeSpeech.Request, response: RecognizeSpeech.Response
    ) -> RecognizeSpeech.Response:
        backend = request.backend.strip() or str(self.get_parameter("asr_backend").value)
        try:
            samples = self._audio_to_float32(request.audio)
            text = ""
            conf = 0.0
            backend_used = backend
            if backend == "iflytek_cloud":
                try:
                    text, conf = self._recognize_cloud(samples)
                except Exception:
                    if bool(self.get_parameter("cloud_asr_fallback_to_local").value):
                        text, conf = self._recognize_local(samples)
                        backend_used = "local"
                    else:
                        raise
            else:
                text, conf = self._recognize_local(samples)

            response.success = True
            response.text = text
            response.confidence = float(conf)
            response.backend_used = backend_used
            response.error_message = ""
            return response
        except Exception as exc:
            response.success = False
            response.text = ""
            response.confidence = 0.0
            response.backend_used = backend
            response.error_message = str(exc)
            return response

    def _handle_stream(self, goal_handle):
        result = RecognizeStream.Result()
        result.backend_used = self._resolve_backend(goal_handle.request.backend)

        try:
            with self._stream_lock:
                self._stream_active = True
                self._stream_speech_active = self._vad_speech_active
                self._stream = self._recognizer.create_stream()
                self._stream_goal_handle = goal_handle
                self._stream_text = ""
                self._stream_started_at = time.time()
                self._stream_last_fed_audio_time = 0.0
                self._stream_end_event.clear()
                speech_active_at_start = self._stream_speech_active
                if speech_active_at_start:
                    preroll_frames, preroll_seconds = self._feed_stream_preroll_locked(
                        "会话开始"
                    )
                else:
                    preroll_frames, preroll_seconds = 0, 0.0

            timeout = max(0.1, float(self.get_parameter("asr_stream_timeout").value))
            self.get_logger().info(
                f"流式 ASR 会话开始: vad_speech_active={speech_active_at_start} "
                f"preroll_frames={preroll_frames} "
                f"preroll_seconds={preroll_seconds:.3f}"
            )
            while not self._stream_end_event.wait(timeout=0.02):
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.text = ""
                    result.confidence = 0.0
                    result.error_message = "流式 ASR 被取消"
                    goal_handle.canceled()
                    return result
                if time.time() - self._stream_started_at > timeout:
                    result.success = False
                    result.text = ""
                    result.confidence = 0.0
                    result.error_message = "流式 ASR 超时"
                    goal_handle.abort()
                    return result

            with self._stream_lock:
                text = self._stream_text.strip()

            result.success = bool(text)
            result.text = text
            result.confidence = 1.0 if text else 0.0
            result.error_message = "" if text else "未识别到有效文本"
            if text:
                goal_handle.succeed()
                self.get_logger().info(f"流式 ASR 结果: {text}")
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            result.success = False
            result.text = ""
            result.confidence = 0.0
            result.error_message = str(exc)
            goal_handle.abort()
            return result
        finally:
            self._reset_stream_state()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceAsrNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
