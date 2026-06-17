"""对话流程守卫节点"""

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status


class DialogContinueGuard(Behaviour):
    """
    对话回合结束守卫。

    放在 DialogLoop (Sequence) 的最后一个子节点。
    检测当前轮对话的 intent：
      - intent != "exit" → SUCCESS (本轮正常结束；是否连续由 main.py 决定)
      - intent == "exit" → FAILURE (打断 DialogLoop → 传播至 Root → 回到唤醒)
    """

    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(
            name="DialogContinueGuard", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )

    def update(self):
        try:
            intent = self.blackboard.intent
        except KeyError:
            intent = ""
        if intent == "exit":
            self.logger.info("用户请求退出，终止对话循环")
            return Status.FAILURE
        return Status.SUCCESS
