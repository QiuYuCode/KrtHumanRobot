"""讯飞云 ASR 监听节点"""

import base64
import hashlib
import hmac
import json
import time
from collections import deque
from email.utils import formatdate
from urllib.parse import urlencode

import numpy as np
import py_trees
import websocket  # type: ignore[import-not-found]
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from loguru import logger

from voice_assistant.config import SAMPLE_RATE


class ListenCloudCommand(Behaviour):
    """
    复用本地 VAD 进行端点检测，ASR 使用讯飞 WebSocket /v2/iat。
    支持:
      - streaming: 按 40ms 帧上传
      - endpoint_once: 端点后一次性上传
    """

    def __init__(self, name: str, engine):
        super().__init__(name)
        self.engine = engine
        self.vad_buffer = np.array([], dtype=np.float32)
        self.speech_buffer: list[np.ndarray] = []
        self.pre_roll_buffer: deque[np.ndarray] = deque()
        self.pre_roll_samples = 0
        self.in_speech = False
        self.last_voice_time = 0.0

        self.blackboard = self.attach_blackboard_client(
            name="ListenCloudCommand", namespace="dialog"
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_activity_time", access=py_trees.common.Access.WRITE
        )

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
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
            "utf-8"
        )
        query = urlencode({"host": host, "date": date, "authorization": authorization})
        return f"{base_url}?{query}"

    @staticmethod
    def _float32_to_pcm16_bytes(samples: np.ndarray) -> bytes:
        clamped = np.clip(samples, -1.0, 1.0)
        pcm = (clamped * 32767.0).astype(np.int16)
        return pcm.tobytes()

    @staticmethod
    def _extract_text(result_obj: dict) -> str:
        ws_list = result_obj.get("ws", [])
        words: list[str] = []
        for item in ws_list:
            for cw in item.get("cw", []):
                w = cw.get("w", "")
                if w:
                    words.append(w)
        return "".join(words).strip()

    def _recognize_with_local_fallback(self, samples: np.ndarray) -> str:
        if self.engine.asr is None:
            return ""
        stream = self.engine.asr.create_stream()
        stream.accept_waveform(SAMPLE_RATE, samples)
        while self.engine.asr.is_ready(stream):
            self.engine.asr.decode_stream(stream)
        return self.engine.asr.get_result(stream).strip()

    def _recognize_cloud(self, samples: np.ndarray) -> str:
        cfg = self.engine.config
        if not (cfg.iflytek_iat_app_id and cfg.iflytek_iat_api_key and cfg.iflytek_iat_api_secret):
            raise RuntimeError("讯飞 ASR 密钥缺失，请设置 XFYUN_IAT_APPID/API_KEY/API_SECRET")
        ws_url = self._build_ws_url(cfg.iflytek_iat_api_key, cfg.iflytek_iat_api_secret)
        logger.debug("[ASR][Cloud] 握手: wss://iat-api.xfyun.cn/v2/iat")
        pcm_bytes = self._float32_to_pcm16_bytes(samples)
        frame_bytes = 1280  # 40ms @ 16k, mono, pcm16
        chunks = [pcm_bytes[i : i + frame_bytes] for i in range(0, len(pcm_bytes), frame_bytes)]
        if not chunks:
            return ""

        ws = websocket.create_connection(ws_url, timeout=10)
        final_text = ""
        frame_count = 0
        try:
            first = {
                "common": {"app_id": cfg.iflytek_iat_app_id},
                "business": {
                    "language": cfg.iflytek_iat_language,
                    "domain": cfg.iflytek_iat_domain,
                    "accent": cfg.iflytek_iat_accent,
                    "vad_eos": int(cfg.iflytek_iat_eos_ms),
                    "ptt": int(cfg.iflytek_iat_ptt),
                },
                "data": {
                    "status": 0,
                    "format": cfg.iflytek_iat_audio_format,
                    "audio": base64.b64encode(chunks[0]).decode("utf-8"),
                    "encoding": cfg.iflytek_iat_encoding,
                },
            }
            ws.send(json.dumps(first))
            frame_count += 1

            if cfg.cloud_asr_strategy == "streaming":
                if len(chunks) == 1:
                    ws.send(
                        json.dumps(
                            {
                                "data": {
                                    "status": 2,
                                    "format": cfg.iflytek_iat_audio_format,
                                    "audio": "",
                                    "encoding": cfg.iflytek_iat_encoding,
                                }
                            }
                        )
                    )
                    frame_count += 1
                else:
                    for i, chunk in enumerate(chunks[1:], start=1):
                        status = 1 if i < len(chunks) - 1 else 2
                        payload = {
                            "data": {
                                "status": status,
                                "format": cfg.iflytek_iat_audio_format,
                                "audio": base64.b64encode(chunk).decode("utf-8"),
                                "encoding": cfg.iflytek_iat_encoding,
                            }
                        }
                        ws.send(json.dumps(payload))
                        frame_count += 1
                        if status != 2:
                            time.sleep(0.04)
            else:
                middle = b"".join(chunks[1:])
                last_payload = {
                    "data": {
                        "status": 2,
                        "format": cfg.iflytek_iat_audio_format,
                        "audio": base64.b64encode(middle).decode("utf-8"),
                        "encoding": cfg.iflytek_iat_encoding,
                    }
                }
                ws.send(json.dumps(last_payload))
                frame_count += 1

            while True:
                raw = ws.recv()
                if not raw:
                    continue
                resp = json.loads(raw)
                code = int(resp.get("code", -1))
                if code != 0:
                    sid = resp.get("sid", "")
                    msg = resp.get("message", "")
                    raise RuntimeError(f"讯飞ASR失败 code={code} sid={sid} msg={msg}")
                data = resp.get("data") or {}
                result = data.get("result") or {}
                text_piece = self._extract_text(result)
                if text_piece:
                    final_text = text_piece
                if int(data.get("status", 1)) == 2 or bool(result.get("ls")):
                    break
        finally:
            ws.close()
            logger.debug(f"[ASR][Cloud] 发送帧数: {frame_count}, 最终文本长度: {len(final_text)}")

        return final_text.strip()

    def initialise(self):
        self.logger.info("云端识别聆听中...")
        self.engine.clear_dialog_queue()
        self.engine.vad.reset()
        self.vad_buffer = np.array([], dtype=np.float32)
        self.speech_buffer = []
        self.pre_roll_buffer = deque()
        self.pre_roll_samples = 0
        self.in_speech = False
        self.last_voice_time = time.time()
        self.blackboard.last_activity_time = time.time()

    def update(self):
        has_audio = False
        window_size = self.engine.vad_window_size
        utterance_ready = False

        while not self.engine.dialog_audio_queue.empty():
            data = self.engine.dialog_audio_queue.get()
            samples = np.frombuffer(data, dtype=np.float32)
            self.vad_buffer = np.concatenate([self.vad_buffer, samples])
            if self.in_speech:
                self.speech_buffer.append(samples)
            has_audio = True

        while len(self.vad_buffer) >= window_size:
            block = self.vad_buffer[:window_size]
            self.vad_buffer = self.vad_buffer[window_size:]

            # 维持一段短时预卷缓存，语音起点时拼接，减少句首被截断
            self.pre_roll_buffer.append(block.copy())
            self.pre_roll_samples += len(block)
            max_pre_roll_samples = int(
                SAMPLE_RATE * max(0.0, float(self.engine.config.cloud_asr_preroll_seconds))
            )
            while self.pre_roll_samples > max_pre_roll_samples and self.pre_roll_buffer:
                popped = self.pre_roll_buffer.popleft()
                self.pre_roll_samples -= len(popped)

            self.engine.vad.accept_waveform(block)
            if self.engine.vad.is_speech_detected():
                self.last_voice_time = time.time()
                if not self.in_speech:
                    self.in_speech = True
                    self.speech_buffer = list(self.pre_roll_buffer)

        while not self.engine.vad.empty():
            self.engine.vad.pop()
            self.last_voice_time = time.time()
            if self.in_speech and self.speech_buffer:
                utterance_ready = True
                self.in_speech = False
                break

        if utterance_ready:
            utterance = np.concatenate(self.speech_buffer).astype(np.float32)
            self.speech_buffer = []
            text = ""
            try:
                text = self._recognize_cloud(utterance)
            except Exception as e:
                self.logger.warning(f"云端ASR异常: {e}")
                if self.engine.config.cloud_asr_fallback_to_local:
                    text = self._recognize_with_local_fallback(utterance)
            if text:
                self.logger.info(f"识别结果: {text}")
                self.blackboard.user_command = text
                self.blackboard.last_activity_time = time.time()
                return Status.SUCCESS

        silence_duration = time.time() - self.last_voice_time
        if silence_duration > self.engine.config.dialog_timeout:
            self.logger.warning(
                f"VAD 检测到 {silence_duration:.1f}s 无语音活动，超时回到唤醒"
            )
            timeout_text = self.engine.config.tts_responses.get(
                "timeout", "没有听到您的命令，有需要可以再叫我。"
            )
            self.engine.speak_blocking(timeout_text)
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        self.vad_buffer = np.array([], dtype=np.float32)
        self.speech_buffer = []
        self.pre_roll_buffer = deque()
        self.pre_roll_samples = 0
        self.in_speech = False
