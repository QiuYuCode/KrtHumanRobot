"""语音合成播放节点"""

from __future__ import annotations

import time

import py_trees
import rclpy
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from voice_interfaces.srv import StopPlayback, SynthesizeSpeech

from krt_human_robot.config import RobotConfig


class SpeakResponse(Behaviour):
    """
    非阻塞 TTS 播放。

    initialise(): 生成音频并启动播放 (不阻塞)
    update():     轮询播放状态，播放中返回 RUNNING，播放完返回 SUCCESS
    terminate():  被打断时立即停播

    配合 Parallel(SuccessOnOne) 实现: TTS 播放期间 WakeWordInterruptMonitor
    可以并行 tick，检测到唤醒词则 Parallel 终止，触发 terminate() 停播。
    """

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self._config = config
        self._node = None
        self._tts_client = None
        self._stop_client = None
        self._synthesize_future = None
        self._play_deadline = 0.0
        self._cooldown_deadline = 0.0
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
        self.blackboard.register_key(
            key="wakeword_interrupted", access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is None:
            return
        self._tts_client = self._node.create_client(
            SynthesizeSpeech, "/voice/tts/synthesize"
        )
        self._stop_client = self._node.create_client(
            StopPlayback, "/voice/playback/stop"
        )

    def initialise(self):
        self._response_text = getattr(self.blackboard, "response_text", "")
        self._play_started = False
        self._finished = False
        self._synthesize_future = None
        self._play_deadline = 0.0
        self._cooldown_deadline = 0.0

        if not self._response_text:
            return

        self.logger.info(f"机器人说: {self._response_text}")
        self.blackboard.is_speaking = True
        self.blackboard.speak_start_time = time.time()
        self._play_started = self._start_speaking()
        if not self._play_started:
            self._finish()

    def update(self):
        if self._finished:
            return Status.SUCCESS

        if not self._response_text:
            self._finish()
            return Status.SUCCESS

        if self._is_wakeword_interrupted():
            self.logger.info("SpeakResponse 检测到唤醒词打断，停止当前播报")
            self._finish()
            return Status.SUCCESS

        if self._synthesize_future is not None:
            if not self._synthesize_future.done():
                return Status.RUNNING
            resp = self._synthesize_future.result()
            self._synthesize_future = None
            if self._is_wakeword_interrupted():
                self.logger.info("TTS 合成完成时已被唤醒词打断，立即停播")
                self._stop_speaking("synthesize_interrupted")
                self._finish()
                return Status.SUCCESS
            if resp is None or not resp.accepted:
                self.logger.warning(
                    f"TTS 服务调用失败: {getattr(resp, 'error_message', 'unknown')}"
                )
                self._finish()
                return Status.SUCCESS
            self._play_deadline = time.time() + max(0.05, float(resp.estimated_duration_sec))
            return Status.RUNNING

        if self._play_deadline > 0.0 and time.time() < self._play_deadline:
            return Status.RUNNING

        if self._cooldown_deadline <= 0.0:
            delay = max(0.0, float(self._config.post_tts_listen_delay))
            self._cooldown_deadline = time.time() + delay
            return Status.RUNNING

        if time.time() < self._cooldown_deadline:
            return Status.RUNNING

        self._finish()
        return Status.SUCCESS

    def terminate(self, new_status):
        """被打断或正常结束时，确保停止播放并重置标志"""
        del new_status
        if self._play_started and not self._finished:
            self._stop_speaking("behaviour_terminate")
        try:
            self.blackboard.is_speaking = False
            self.blackboard.speak_start_time = 0.0
        except Exception:
            pass

    def _finish(self):
        """播放结束的清理"""
        if self._finished:
            return
        self._stop_speaking("finish")
        self.blackboard.is_speaking = False
        self.blackboard.speak_start_time = 0.0
        self.blackboard.last_activity_time = time.time()
        self._finished = True

    def _start_speaking(self) -> bool:
        if self._tts_client is not None and self._tts_client.wait_for_service(timeout_sec=0.2):
            req = SynthesizeSpeech.Request()
            req.text = self._response_text
            req.language = "zh-CN"
            req.style = "default"
            req.priority = 1
            self._synthesize_future = self._tts_client.call_async(req)
            return True
        return False

    def _stop_speaking(self, reason: str = "behaviour_terminate") -> None:
        self._play_deadline = 0.0
        self._cooldown_deadline = 0.0
        if self._stop_client is not None and self._stop_client.wait_for_service(timeout_sec=0.1):
            req = StopPlayback.Request()
            req.reason = reason
            self._stop_client.call_async(req)
            self.logger.info(f"SpeakResponse 已发送停播请求: reason={reason}")
        else:
            self.logger.warning("SpeakResponse 停播失败：/voice/playback/stop 服务不可用")

    def _is_wakeword_interrupted(self) -> bool:
        try:
            return bool(self.blackboard.wakeword_interrupted)
        except KeyError:
            return False


class WakeupResponse(Behaviour):
    """
    唤醒成功后的简短提示音 (阻塞式)。

    播放一句简短的 "我在，请说" 然后 SUCCESS。
    因为是极短的提示音 (~1 秒)，阻塞不影响体验。
    """

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self.config = config
        self._node = None
        self._tts_client = None

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is None:
            return
        self._tts_client = self._node.create_client(
            SynthesizeSpeech, "/voice/tts/synthesize"
        )

    def update(self):
        text = self.config.tts_responses.get("wakeup", "我在，请说。")
        self._speak_and_wait(text)
        return Status.SUCCESS

    def _speak_and_wait(self, text: str) -> bool:
        """提交唤醒提示音，并等待估算播放时长和回声冷却窗口。"""
        if not text.strip() or self._node is None or self._tts_client is None:
            return False
        if not self._tts_client.wait_for_service(timeout_sec=0.2):
            return False
        req = SynthesizeSpeech.Request()
        req.text = text
        req.language = "zh-CN"
        req.style = "default"
        req.priority = 2
        future = self._tts_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=3.0)
            resp = future.result()
            if resp is None or not resp.accepted:
                return False
            wait_s = max(0.0, float(resp.estimated_duration_sec))
            wait_s += max(0.0, float(self.config.post_tts_listen_delay))
            if wait_s > 0.0:
                time.sleep(wait_s)
            return True
        except Exception:
            return False
