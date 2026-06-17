"""对话黑板初始化节点。"""

from __future__ import annotations

import time

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status


class InitializeDialogBlackboard(Behaviour):
    """在每轮对话开始前补齐黑板默认字段，避免读取未定义键。"""

    def __init__(self, name: str = "InitDialogBlackboard"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(
            name="InitializeDialogBlackboard",
            namespace="dialog",
        )
        self.blackboard.register_key(
            key="intent",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="response_text",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="user_command",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="action_plan",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="is_speaking",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="speak_start_time",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="wakeword_interrupted",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="arm_pending_tts",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="arm_busy",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="last_activity_time",
            access=py_trees.common.Access.WRITE,
        )

    def update(self) -> Status:
        now = time.time()
        defaults = {
            "intent": "",
            "response_text": "",
            "user_command": "",
            "action_plan": [],
            "is_speaking": False,
            "speak_start_time": 0.0,
            "wakeword_interrupted": False,
            "arm_pending_tts": "",
            "arm_busy": False,
            "last_activity_time": now,
        }
        for key, default_value in defaults.items():
            try:
                getattr(self.blackboard, key)
            except KeyError:
                setattr(self.blackboard, key, default_value)
        return Status.SUCCESS
