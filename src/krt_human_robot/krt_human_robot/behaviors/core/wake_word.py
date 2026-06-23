"""唤醒词检测节点"""

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from voice_interfaces.msg import VoiceKwsEvent


class WaitForWakeWord(Behaviour):
    """
    等待唤醒词。

    持续监听麦克风，检测到唤醒词后返回 SUCCESS。
    通过 config.kws_keywords_file 替换唤醒词，无需修改代码。
    """

    def __init__(self, name: str):
        super().__init__(name)
        self._node = None
        self._subscription = None
        self._kws_triggered = False
        self._last_keyword = ""

    def setup(self, **kwargs):
        self._node = kwargs.get("node")

    def _on_kws_event(self, msg: VoiceKwsEvent) -> None:
        self._kws_triggered = True
        self._last_keyword = msg.keyword

    def initialise(self):
        self.logger.info("待机中，等待唤醒词...")
        self._kws_triggered = False
        self._last_keyword = ""
        if self._node is not None and self._subscription is None:
            self._subscription = self._node.create_subscription(
                VoiceKwsEvent, "/voice/kws/events", self._on_kws_event, 20
            )

    def update(self):
        if self._kws_triggered:
            self.logger.info(f"唤醒成功! 关键词: {self._last_keyword or 'unknown'}")
            self._kws_triggered = False
            return Status.SUCCESS
        return Status.RUNNING

    def terminate(self, new_status):
        del new_status
