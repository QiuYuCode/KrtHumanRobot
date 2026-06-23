"""固定回复动作节点"""

from __future__ import annotations

from krt_human_robot.config import RobotConfig

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status


class FixedResponseAction(Behaviour):
    """命中配置中的固定意图时，直接返回预设回复。"""

    _RESERVED_RESPONSE_KEYS = {"wakeup", "timeout"}

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self._config = config
        self.blackboard = self.attach_blackboard_client(
            name="FixedResponseAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def update(self) -> Status:
        intent = getattr(self.blackboard, "intent", "")
        if not intent:
            return Status.FAILURE

        if intent in self._RESERVED_RESPONSE_KEYS:
            return Status.FAILURE

        response = self._config.tts_responses.get(intent, "").strip()
        if not response:
            return Status.FAILURE

        # self.logger.info(f"固定回复命中: {intent}")
        self.blackboard.response_text = response
        return Status.SUCCESS
