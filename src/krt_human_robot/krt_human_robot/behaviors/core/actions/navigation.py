"""ROS 导航动作节点"""

from __future__ import annotations

from loguru import logger

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status


# ============================================================================
# 独立执行函数 (供 Behaviour 节点和 planner tool 共同调用)
# ============================================================================

def execute_navigate(destination: str) -> str:
    """导航核心逻辑。返回执行结果描述。

    TODO: 通过 ROS topic / service / action 发布导航目标
    """
    logger.info("执行导航: {}", destination)
    # TODO: 实际 ROS 导航实现
    return f"好的，正在前往{destination}。"


# ============================================================================
# 导航动作 (行为树节点)
# ============================================================================

class NavigationAction(Behaviour):
    """
    通过 ROS 发布导航目标。

    当 intent == "navigation" 时执行，否则返回 FAILURE。
    预留 ROS 接口，实际使用时通过 rclpy 或 rosbridge 通信。
    """

    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(
            name="NavigationAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def update(self):
        if self.blackboard.intent != "navigation":
            return Status.FAILURE

        command = self.blackboard.user_command
        self.logger.info(f"执行: 导航 ({command})")
        self.blackboard.response_text = execute_navigate(command)
        return Status.SUCCESS
