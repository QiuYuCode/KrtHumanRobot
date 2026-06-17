"""播报期间的唤醒词打断节点"""

import random
import time

import numpy as np
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from voice_assistant.config import SAMPLE_RATE, RobotConfig


def _is_wakeword_interrupted(blackboard) -> bool:
    """黑板字段未初始化时按 False 处理。"""
    try:
        return bool(blackboard.wakeword_interrupted)
    except KeyError:
        return False


class WakeWordInterruptMonitor(Behaviour):
    """TTS 播报期间持续监听唤醒词，命中后触发停播。"""

    def __init__(self, name: str, engine, config: RobotConfig):
        super().__init__(name)
        self.engine = engine
        self.config = config
        self.kws_stream = None

        self.blackboard = self.attach_blackboard_client(
            name="WakeWordInterruptMonitor", namespace="dialog"
        )
        self.blackboard.register_key(
            key="is_speaking", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="speak_start_time", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="wakeword_interrupted", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        if self.engine.kws is not None:
            self.kws_stream = self.engine.kws.create_stream()

    def initialise(self):
        self.blackboard.wakeword_interrupted = False
        if self.kws_stream is not None and self.engine.kws is not None:
            self.engine.kws.reset_stream(self.kws_stream)
        self.engine.clear_kws_queue()

    def update(self):
        if self.engine.kws is None or self.kws_stream is None:
            return Status.RUNNING

        try:
            is_speaking = bool(self.blackboard.is_speaking)
        except KeyError:
            is_speaking = False
        if not is_speaking:
            self.engine.clear_kws_queue()
            return Status.RUNNING

        try:
            speak_start_time = float(self.blackboard.speak_start_time)
        except KeyError:
            speak_start_time = 0.0
        if time.time() - speak_start_time < self.config.interrupt_min_speech_seconds:
            self.engine.clear_kws_queue()
            return Status.RUNNING

        while not self.engine.kws_audio_queue.empty():
            data = self.engine.kws_audio_queue.get()
            samples = np.frombuffer(data, dtype=np.float32)
            self.kws_stream.accept_waveform(SAMPLE_RATE, samples)

            while self.engine.kws.is_ready(self.kws_stream):
                self.engine.kws.decode_stream(self.kws_stream)
                keyword = self.engine.kws.get_result(self.kws_stream)
                if keyword:
                    self.logger.info(f"TTS 播报中检测到唤醒词: {keyword.strip()}")
                    self.blackboard.wakeword_interrupted = True
                    self.engine.kws.reset_stream(self.kws_stream)
                    return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
        if self.kws_stream is not None and self.engine.kws is not None:
            self.engine.kws.reset_stream(self.kws_stream)


class ResetWakeWordInterruptState(Behaviour):
    """在播报被唤醒词打断后清理本轮对话状态，下一轮直接进入 Listen。"""

    def __init__(self, name: str, engine, config: RobotConfig):
        super().__init__(name)
        self.engine = engine
        self.config = config

        self.blackboard = self.attach_blackboard_client(
            name="ResetWakeWordInterruptState", namespace="dialog"
        )
        self.blackboard.register_key(
            key="wakeword_interrupted", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="action_plan", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_activity_time", access=py_trees.common.Access.WRITE
        )

    def update(self):
        if not _is_wakeword_interrupted(self.blackboard):
            return Status.SUCCESS

        self.logger.info("唤醒词打断成功，停止当前回复并回到聆听")
        responses = [
            text.strip()
            for text in self.config.interrupt_wakeup_responses
            if text and text.strip()
        ]
        if responses:
            self.engine.speak_blocking(random.choice(responses))
        self.blackboard.wakeword_interrupted = False
        self.blackboard.intent = ""
        self.blackboard.response_text = ""
        self.blackboard.user_command = ""
        self.blackboard.action_plan = []
        self.blackboard.last_activity_time = time.time()
        self.engine.clear_dialog_queue()
        self.engine.clear_kws_queue()
        return Status.SUCCESS
