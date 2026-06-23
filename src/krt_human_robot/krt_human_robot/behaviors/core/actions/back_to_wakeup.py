"""回到待机状态动作节点"""

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status


class BackToWakeUp(Behaviour):
    """
    回到待机状态。

    当 intent == "exit" (用户说了"退出"等关键词) 时：
      - 设置 response_text 回复
      - 返回 SUCCESS，由下游 DialogContinueGuard 检测 exit 意图后
        返回 FAILURE 终止对话循环

    当 intent 不匹配时返回 FAILURE，让 Selector 继续尝试下一个 action。
    """

    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(
            name="BackToWakeUp", namespace="dialog"
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def update(self):
        if self.blackboard.intent != "exit":
            return Status.FAILURE

        command = self.blackboard.user_command
        self.logger.info(f"用户命令: {command}, 回到待机状态")
        self.blackboard.response_text = "好的，我先休息了。"
        return Status.SUCCESS