"""语音指令监听节点"""

import time

import numpy as np
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from voice_assistant.config import SAMPLE_RATE


class ListenCommand(Behaviour):
    """
    监听用户语音指令 (流式 ASR + VAD 静默超时)。

    使用 sherpa_onnx OnlineRecognizer 流式识别，同时通过
    VoiceActivityDetector (Silero VAD) 追踪语音活动。

    返回值:
      - SUCCESS: ASR 检测到端点且有文本 → 正常对话流程
      - FAILURE: VAD 检测到持续静默超过 dialog_timeout → 超时回到唤醒
      - RUNNING: 仍在监听中
    """

    def __init__(self, name: str, engine):
        super().__init__(name)
        self.engine = engine
        self.asr_stream = None
        self.vad_buffer = np.array([], dtype=np.float32)
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
        self.asr_stream = self.engine.asr.create_stream()
        # 清空对话队列，避免处理旧数据
        self.engine.clear_dialog_queue()
        # 重置 VAD 状态
        self.vad_buffer = np.array([], dtype=np.float32)
        self.engine.vad.reset()
        self.last_voice_time = time.time()
        self.blackboard.last_activity_time = time.time()

    def update(self):
        has_audio = False
        window_size = self.engine.vad_window_size

        # --- 从队列读取音频，同时送入 ASR 和 VAD ---
        while not self.engine.dialog_audio_queue.empty():
            data = self.engine.dialog_audio_queue.get()
            samples = np.frombuffer(data, dtype=np.float32)
            # 送入 ASR
            self.asr_stream.accept_waveform(SAMPLE_RATE, samples)
            # 累积到 VAD 缓冲区
            self.vad_buffer = np.concatenate([self.vad_buffer, samples])
            has_audio = True

        # --- VAD 处理：按 window_size 分块送入 ---
        while len(self.vad_buffer) >= window_size:
            self.engine.vad.accept_waveform(self.vad_buffer[:window_size])
            self.vad_buffer = self.vad_buffer[window_size:]

        # 检测正在进行的语音活动 (实时)
        if self.engine.vad.is_speech_detected():
            self.last_voice_time = time.time()

        # 消费已完成的语音段 (语音→静默转换时产生)
        while not self.engine.vad.empty():
            self.engine.vad.pop()
            self.last_voice_time = time.time()

        # --- ASR 解码 ---
        if has_audio:
            while self.engine.asr.is_ready(self.asr_stream):
                self.engine.asr.decode_stream(self.asr_stream)

            text = self.engine.asr.get_result(self.asr_stream).strip()
            is_endpoint = self.engine.asr.is_endpoint(self.asr_stream)

            if is_endpoint and len(text) > 0:
                self.logger.info(f"识别结果: {text}")
                self.blackboard.user_command = text
                self.blackboard.last_activity_time = time.time()
                return Status.SUCCESS

        # --- VAD 静默超时检测 ---
        silence_duration = time.time() - self.last_voice_time
        timeout = self.engine.config.dialog_timeout
        if silence_duration > timeout:
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
        self.asr_stream = None
        self.vad_buffer = np.array([], dtype=np.float32)
