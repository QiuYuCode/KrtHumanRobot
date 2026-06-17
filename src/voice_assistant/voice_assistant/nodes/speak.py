"""语音合成播放节点"""

import time

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

class SpeakResponse(Behaviour):
    """
    非阻塞 TTS 播放。

    initialise(): 生成音频并启动播放 (不阻塞)
    update():     轮询播放状态，播放中返回 RUNNING，播放完返回 SUCCESS
    terminate():  被打断时立即停播

    配合 Parallel(SuccessOnOne) 实现: TTS 播放期间 WakeWordInterruptMonitor
    可以并行 tick，检测到唤醒词则 Parallel 终止，触发 terminate() 停播。
    """

    def __init__(self, name: str, engine):
        super().__init__(name)
        self.engine = engine
        self._response_text = ""
        self._play_started = False
        self._finished = False

        self.blackboard = self.attach_blackboard_client(
            name="SpeakResponse", namespace="dialog"
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="is_speaking", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="speak_start_time", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="last_activity_time", access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        self._response_text = getattr(self.blackboard, "response_text", "")
        self._play_started = False
        self._finished = False

        if not self._response_text:
            return

        self.logger.info(f"机器人说: {self._response_text}")
        self.blackboard.is_speaking = True
        self.blackboard.speak_start_time = time.time()
        self._play_started = self.engine.start_speaking(self._response_text)
        if not self._play_started:
            self._finish()

    def update(self):
        if self._finished:
            return Status.SUCCESS

        if not self._response_text:
            self._finish()
            return Status.SUCCESS

        if self.engine.is_speaking_active():
            return Status.RUNNING

        self._finish()
        return Status.SUCCESS

    def terminate(self, new_status):
        """被打断或正常结束时，确保停止播放并重置标志"""
        if self._play_started and not self._finished:
            self.engine.stop_speaking()
        try:
            self.blackboard.is_speaking = False
            self.blackboard.speak_start_time = 0.0
        except Exception:
            pass

    def _finish(self):
        """播放结束的清理"""
        if self._finished:
            return
        self.engine.stop_speaking()
        self.blackboard.is_speaking = False
        self.blackboard.speak_start_time = 0.0
        self.blackboard.last_activity_time = time.time()
        self._finished = True

        # 写入监控对话历史（仅当监控已启用时）
        monitor = getattr(self.engine, "monitor", None)
        if monitor is not None and self._response_text:
            user_cmd = getattr(self.blackboard, "user_command", "")
            monitor.log_conversation(user_cmd, self._response_text)


class WakeupResponse(Behaviour):
    """
    唤醒成功后的简短提示音 (阻塞式)。

    播放一句简短的 "我在，请说" 然后 SUCCESS。
    因为是极短的提示音 (~1 秒)，阻塞不影响体验。
    """

    def __init__(self, name: str, engine, config):
        super().__init__(name)
        self.engine = engine
        self.config = config

    def update(self):
        text = self.config.tts_responses.get("wakeup", "我在，请说。")
        self.engine.speak_blocking(text)
        return Status.SUCCESS
