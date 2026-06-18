"""语音指令监听节点"""

from __future__ import annotations

import time

import py_trees
from rclpy.action import ActionClient
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from voice_interfaces.action import RecognizeStream
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
        self._asr_stream_client = None
        self._tts_client = None
        self._audio_sub = None
        self._vad_sub = None
        self._listening = False
        self._pending_asr_future = None
        self._stream_goal_future = None
        self._stream_result_future = None
        self._stream_goal_handle = None
        self._stream_requested = False
        self._stream_failed = False
        self._latest_partial_text = ""
        self._speech_active = False
        self._speech_end_detected = False
        self._utterance_bytes = bytearray()
        self._utterance_encoding = "float32"
        self._sample_rate = 16000
        self._listen_start_time = 0.0
        self._listen_ignore_until = 0.0
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
        self._stream_goal_future = None
        self._stream_result_future = None
        self._stream_goal_handle = None
        self._stream_requested = False
        self._stream_failed = False
        self._latest_partial_text = ""
        self._speech_active = False
        self._speech_end_detected = False
        self._utterance_bytes = bytearray()
        self._utterance_encoding = "float32"
        self._sample_rate = 16000
        now = time.time()
        cooldown = max(0.0, float(self.config.post_tts_listen_delay))
        self._listen_start_time = now
        self._listen_ignore_until = now + cooldown
        self.last_voice_time = self._listen_ignore_until
        self.blackboard.last_activity_time = now

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is None:
            return
        if self._asr_client is None:
            self._asr_client = self._node.create_client(
                RecognizeSpeech, "/voice/asr/recognize"
            )
        if self._asr_stream_client is None:
            self._asr_stream_client = ActionClient(
                self._node, RecognizeStream, "/voice/asr/stream"
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
        if time.time() < self._listen_ignore_until:
            return
        if self._speech_active:
            self._utterance_bytes.extend(bytes(msg.data))
            self._utterance_encoding = msg.encoding
            self._sample_rate = int(msg.sample_rate)
            self.last_voice_time = time.time()

    def _on_vad_event(self, msg: VoiceVadEvent) -> None:
        if not self._listening:
            return
        if time.time() < self._listen_ignore_until:
            self._speech_active = False
            self._speech_end_detected = False
            self._utterance_bytes = bytearray()
            return
        if msg.event_type == "speech_start":
            self._speech_active = True
            self.last_voice_time = time.time()
            if (
                self._streaming_enabled()
                and not self._stream_failed
                and not self._stream_requested
                and not self._stream_in_progress()
            ):
                if not self._start_stream_request():
                    self.logger.warning("流式 ASR action 不可用，回退整段识别")
                    self._stream_failed = True
        elif msg.event_type == "speech_end" and self._speech_active:
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

    def _streaming_enabled(self) -> bool:
        return (
            bool(self.config.asr_streaming_enabled)
            and str(self.config.asr_backend).strip() == "local"
        )

    def _stream_in_progress(self) -> bool:
        return (
            self._stream_goal_future is not None
            or self._stream_result_future is not None
            or self._stream_goal_handle is not None
        )

    def _on_stream_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        text = str(getattr(feedback, "partial_text", "")).strip()
        if text and text != self._latest_partial_text:
            self._latest_partial_text = text
            self.logger.debug(f"流式识别 partial: {text}")

    def _start_stream_request(self) -> bool:
        if self._asr_stream_client is None:
            return False
        if not self._asr_stream_client.wait_for_server(timeout_sec=0.05):
            return False
        goal = RecognizeStream.Goal()
        goal.backend = "local"
        self._stream_goal_future = self._asr_stream_client.send_goal_async(
            goal, feedback_callback=self._on_stream_feedback,
        )
        self._stream_requested = True
        self.logger.info("流式 ASR 已启动")
        return True

    def _poll_stream_result(self) -> Status | None:
        if not self._streaming_enabled() or self._stream_failed:
            return None

        now = time.time()
        if (
            not self._stream_requested
            and not self._stream_in_progress()
            and now >= self._listen_ignore_until
        ):
            if not self._start_stream_request():
                self.logger.warning("流式 ASR action 不可用，回退整段识别")
                self._stream_failed = True
                return None

        if self._stream_goal_future is not None:
            if not self._stream_goal_future.done():
                return None
            goal_handle = self._stream_goal_future.result()
            self._stream_goal_future = None
            if goal_handle is None or not goal_handle.accepted:
                self.logger.warning("流式 ASR goal 被拒绝，回退整段识别")
                self._stream_failed = True
                self._stream_goal_handle = None
                return None
            self._stream_goal_handle = goal_handle
            self._stream_result_future = goal_handle.get_result_async()

        if self._stream_result_future is None:
            return None
        if not self._stream_result_future.done():
            return None

        wrapped = self._stream_result_future.result()
        result = wrapped.result if wrapped is not None else None
        self._stream_result_future = None
        self._stream_goal_handle = None
        if result is not None and result.success and result.text.strip():
            text = result.text.strip()
            self.logger.info(f"流式识别结果: {text}")
            self.blackboard.user_command = text
            self.blackboard.last_activity_time = time.time()
            self._utterance_bytes = bytearray()
            self._speech_end_detected = False
            return Status.SUCCESS

        error = getattr(result, "error_message", "unknown") if result else "unknown"
        self.logger.warning(f"流式 ASR 失败，回退整段识别: {error}")
        self._stream_failed = True
        return None

    def _speak_timeout(self, text: str) -> None:
        speak_blocking(self._node, self._tts_client, text)

    def update(self):
        stream_status = self._poll_stream_result()
        if stream_status is not None:
            return stream_status

        if (
            self._speech_end_detected
            and self._utterance_bytes
            and self._pending_asr_future is None
            and (not self._streaming_enabled() or self._stream_failed)
        ):
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
        if self._stream_goal_handle is not None:
            try:
                self._stream_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._pending_asr_future = None
        self._stream_goal_future = None
        self._stream_result_future = None
        self._stream_goal_handle = None
        self._stream_requested = False
        self._stream_failed = False
        self._latest_partial_text = ""
        self._speech_active = False
        self._speech_end_detected = False
        self._listen_start_time = 0.0
        self._listen_ignore_until = 0.0
        self._utterance_bytes = bytearray()
