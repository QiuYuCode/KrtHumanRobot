"""播报期间的唤醒词打断节点"""

from __future__ import annotations

import random
import time

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from voice_interfaces.msg import VoiceKwsEvent
from voice_interfaces.srv import SynthesizeSpeech

from voice_assistant.config import RobotConfig
from voice_assistant.ros_voice import speak_blocking


def _is_wakeword_interrupted(blackboard) -> bool:
    """黑板字段未初始化时按 False 处理。"""
    try:
        return bool(blackboard.wakeword_interrupted)
    except KeyError:
        return False


class WakeWordInterruptMonitor(Behaviour):
    """TTS 播报期间持续监听唤醒词，命中后触发停播。"""

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self.config = config
        self._node = None
        self._subscription = None
        self._kws_triggered = False
        self._last_keyword = ""

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
        self._node = kwargs.get("node")
        if self._node is not None and self._subscription is None:
            self._subscription = self._node.create_subscription(
                VoiceKwsEvent, "/voice/kws/events", self._on_kws_event, 20
            )

    def _on_kws_event(self, msg: VoiceKwsEvent) -> None:
        self._kws_triggered = True
        self._last_keyword = msg.keyword

    def initialise(self):
        self.blackboard.wakeword_interrupted = False
        self._kws_triggered = False
        self._last_keyword = ""

    def update(self):
        try:
            is_speaking = bool(self.blackboard.is_speaking)
        except KeyError:
            is_speaking = False
        if not is_speaking:
            return Status.RUNNING

        try:
            speak_start_time = float(self.blackboard.speak_start_time)
        except KeyError:
            speak_start_time = 0.0
        if time.time() - speak_start_time < self.config.interrupt_min_speech_seconds:
            return Status.RUNNING

        if self._kws_triggered:
            self.logger.info(f"TTS 播报中检测到唤醒词: {self._last_keyword or 'unknown'}")
            self.blackboard.wakeword_interrupted = True
            self._kws_triggered = False
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status):
        del new_status


class ResetWakeWordInterruptState(Behaviour):
    """在播报被唤醒词打断后清理本轮对话状态，下一轮直接进入 Listen。"""

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self.config = config
        self._node = None
        self._tts_client = None

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

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is None:
            return
        self._tts_client = self._node.create_client(
            SynthesizeSpeech, "/voice/tts/synthesize"
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
            speak_blocking(self._node, self._tts_client, random.choice(responses))
        self.blackboard.wakeword_interrupted = False
        self.blackboard.intent = ""
        self.blackboard.response_text = ""
        self.blackboard.user_command = ""
        self.blackboard.action_plan = []
        self.blackboard.last_activity_time = time.time()
        return Status.SUCCESS
