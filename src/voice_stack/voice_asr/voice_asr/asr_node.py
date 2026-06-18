from __future__ import annotations

import base64
import hashlib
import hmac
import json
import time
from email.utils import formatdate
from urllib.parse import urlencode

import numpy as np
import rclpy
import sherpa_onnx
import websocket  # type: ignore[import-not-found]
from rclpy.action import ActionServer
from rclpy.node import Node
from voice_interfaces.action import RecognizeStream
from voice_interfaces.msg import VoiceAudioFrame
from voice_interfaces.srv import RecognizeSpeech


class VoiceAsrNode(Node):
    """ASR 节点：提供识别服务与流式 action 占位。"""

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

        self._recognizer = self._create_local_asr()
        self._srv = self.create_service(
            RecognizeSpeech, "/voice/asr/recognize", self._handle_recognize
        )
        self._action_server = ActionServer(
            self,
            RecognizeStream,
            "/voice/asr/stream",
            execute_callback=self._handle_stream,
        )
        self.get_logger().info("voice_asr ready: srv=/voice/asr/recognize")

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

    def _recognize_cloud(self, samples: np.ndarray) -> tuple[str, float]:
        app_id = str(self.get_parameter("iflytek_iat_app_id").value)
        api_key = str(self.get_parameter("iflytek_iat_api_key").value)
        api_secret = str(self.get_parameter("iflytek_iat_api_secret").value)
        if not (app_id and api_key and api_secret):
            raise RuntimeError("讯飞 ASR 密钥缺失")
        ws = websocket.create_connection(self._build_ws_url(api_key, api_secret), timeout=10)
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
        result.success = False
        result.text = ""
        result.confidence = 0.0
        result.backend_used = goal_handle.request.backend
        result.error_message = "RecognizeStream 未在本轮实现，使用 /voice/asr/recognize"
        goal_handle.abort()
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceAsrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
