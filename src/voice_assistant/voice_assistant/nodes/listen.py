"""语音指令监听节点"""

from __future__ import annotations

import time

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from voice_interfaces.msg import VoiceAudioFrame, VoiceVadEvent
from voice_interfaces.srv import RecognizeSpeech, SynthesizeSpeech

from voice_assistant.config import RobotConfig
from voice_assistant.ros_voice import speak_blocking


class ListenCommand(Behaviour):
    """
    监听用户语音指令 (VAD 端点触发 ASR + 静默超时)。

    返回值:
      - SUCCESS: ASR 检测到端点且有文本 → 正常对话流程
      - FAILURE: VAD 检测到持续静默超过 dialog_timeout → 超时回到唤醒
      - RUNNING: 仍在监听中
    """

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self.config = config
        self._node = None
        self._asr_client = None
        self._tts_client = None
        self._audio_sub = None
        self._vad_sub = None
        self._listening = False
        self._pending_asr_future = None
        self._speech_active = False
        self._speech_end_detected = False
        self._utterance_bytes = bytearray()
        self._utterance_encoding = "float32"
        self._sample_rate = 16000
        self.last_voice_time = 0.0

        self.blackboard = self.attach_blackboard_client(
            name="ListenCommand", namespace="dialog"
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_activity_time", access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        self.logger.info("正在聆听...")
        self._listening = True
        self._pending_asr_future = None
        self._speech_active = False
        self._speech_end_detected = False
        self._utterance_bytes = bytearray()
        self._utterance_encoding = "float32"
        self._sample_rate = 16000
        self.last_voice_time = time.time()
        self.blackboard.last_activity_time = time.time()

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is None:
            return
        if self._asr_client is None:
            self._asr_client = self._node.create_client(
                RecognizeSpeech, "/voice/asr/recognize"
            )
        if self._tts_client is None:
            self._tts_client = self._node.create_client(
                SynthesizeSpeech, "/voice/tts/synthesize"
            )
        if self._audio_sub is None:
            self._audio_sub = self._node.create_subscription(
                VoiceAudioFrame, "/voice/audio/raw", self._on_audio, 30
            )
        if self._vad_sub is None:
            self._vad_sub = self._node.create_subscription(
                VoiceVadEvent, "/voice/vad/events", self._on_vad_event, 20
            )

    def _on_audio(self, msg: VoiceAudioFrame) -> None:
        if not self._listening:
            return
        if self._speech_active:
            self._utterance_bytes.extend(bytes(msg.data))
            self._utterance_encoding = msg.encoding
            self._sample_rate = int(msg.sample_rate)
            self.last_voice_time = time.time()

    def _on_vad_event(self, msg: VoiceVadEvent) -> None:
        if not self._listening:
            return
        if msg.event_type == "speech_start":
            self._speech_active = True
            self.last_voice_time = time.time()
        elif msg.event_type == "speech_end":
            self._speech_active = False
            self._speech_end_detected = True
            self.last_voice_time = time.time()

    def _start_asr_request(self) -> bool:
        if self._asr_client is None or not self._asr_client.wait_for_service(timeout_sec=0.2):
            return False
        req = RecognizeSpeech.Request()
        req.audio = VoiceAudioFrame()
        req.audio.stamp = self._node.get_clock().now().to_msg()
        req.audio.frame_id = "listen"
        req.audio.encoding = self._utterance_encoding
        req.audio.sample_rate = int(self._sample_rate)
        req.audio.channels = 1
        req.audio.data = list(self._utterance_bytes)
        req.backend = ""
        self._pending_asr_future = self._asr_client.call_async(req)
        self._speech_end_detected = False
        return True

    def _speak_timeout(self, text: str) -> None:
        speak_blocking(self._node, self._tts_client, text)

    def update(self):
        if self._speech_end_detected and self._utterance_bytes and self._pending_asr_future is None:
            if not self._start_asr_request():
                self.logger.warning("ASR 服务不可用")

        if self._pending_asr_future is not None:
            if not self._pending_asr_future.done():
                return Status.RUNNING
            response = self._pending_asr_future.result()
            self._pending_asr_future = None
            self._utterance_bytes = bytearray()
            if response is not None and response.success and response.text.strip():
                text = response.text.strip()
                self.logger.info(f"识别结果: {text}")
                self.blackboard.user_command = text
                self.blackboard.last_activity_time = time.time()
                return Status.SUCCESS

        silence_duration = time.time() - self.last_voice_time
        timeout = self.config.dialog_timeout
        if silence_duration > timeout:
            self.logger.warning(
                f"VAD 检测到 {silence_duration:.1f}s 无语音活动，超时回到唤醒"
            )
            timeout_text = self.config.tts_responses.get(
                "timeout", "没有听到您的命令，有需要可以再叫我。"
            )
            self._speak_timeout(timeout_text)
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        del new_status
        self._listening = False
        self._pending_asr_future = None
        self._speech_active = False
        self._speech_end_detected = False
        self._utterance_bytes = bytearray()
