"""默认响应动作节点"""

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status


class DefaultResponse(Behaviour):
    """
    兜底响应。

    当所有其他 action (包括 LLM) 都无法处理时的最后兜底。
    始终返回 SUCCESS，确保 ActionSelector 不会整体 FAILURE。
    """

    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(
            name="DefaultResponse", namespace="dialog"
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def update(self):
        command = getattr(self.blackboard, "user_command", "")
        self.logger.info(f"默认响应: {command}")
        self.blackboard.response_text = f"我听到了: {command}，但我还不能理解这个指令。"
        return Status.SUCCESS
