"""后台动作的待播报消息转发节点。

用于“机械臂动作异步执行失败后”的二次播报：
- 后台线程写 blackboard.arm_pending_tts
- 本节点将其搬运到 blackboard.response_text 并清空 pending
"""

from __future__ import annotations

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status


class ArmPendingTtsAction(Behaviour):
    """若存在 pending TTS，则触发一次播报。"""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(
            name="ArmPendingTtsAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="arm_pending_tts", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="arm_busy", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="is_speaking", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )
        # 初始化默认值，避免首轮 tick 读取不存在键导致 KeyError
        try:
            self.blackboard.arm_pending_tts = ""
        except Exception:
            pass

    def update(self) -> Status:
        try:
            pending = self.blackboard.arm_pending_tts or ""
        except KeyError:
            self.blackboard.arm_pending_tts = ""
            pending = ""
        if not pending.strip():
            return Status.FAILURE

        # 说话期间不抢占，等待下一次 tick
        try:
            is_speaking = bool(self.blackboard.is_speaking)
        except KeyError:
            is_speaking = False
        if is_speaking:
            return Status.RUNNING

        self.blackboard.response_text = pending.strip()
        self.blackboard.arm_pending_tts = ""
        return Status.SUCCESS

