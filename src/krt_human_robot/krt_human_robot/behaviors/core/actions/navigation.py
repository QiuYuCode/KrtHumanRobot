"""ROS 导航动作节点"""

from __future__ import annotations

from loguru import logger

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.adapters.navigation import RangerNavAdapter
from krt_human_robot.config import RobotConfig


# ============================================================================
# 独立执行函数 (供 Behaviour 节点和 planner tool 共同调用)
# ============================================================================

def execute_navigate(config: RobotConfig, destination: str) -> str:
    """导航核心逻辑。返回执行结果描述。

    具体地点导航尚未接 Nav2 goal action；当前只启动导航模式。
    """
    logger.info("执行导航: {}", destination)
    return "导航地点功能待接入，请先说开始导航。"


# ============================================================================
# 导航动作 (行为树节点)
# ============================================================================

class NavigationAction(Behaviour):
    """
    控制 ranger_nav 建图/保存/导航模式。

    地点导航保留为旧 navigation intent，后续再接 Nav2 goal action。
    """

    _INTENTS = {
        "start_mapping",
        "save_mapping",
        "start_navigation",
        "stop_navigation",
        "navigation",
    }

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self._config = config
        self._adapter = RangerNavAdapter(config)

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
        intent = self.blackboard.intent
        if intent not in self._INTENTS:
            return Status.FAILURE

        command = self.blackboard.user_command
        self.logger.info(f"执行: 导航控制 {intent} ({command})")
        if intent == "start_mapping":
            result = self._adapter.start_mapping()
        elif intent == "save_mapping":
            result = self._adapter.save_mapping()
        elif intent == "start_navigation":
            result = self._adapter.start_navigation()
        elif intent == "stop_navigation":
            result = self._adapter.stop_navigation()
        else:
            self.blackboard.response_text = execute_navigate(self._config, command)
            return Status.SUCCESS

        self.blackboard.response_text = result.message
        return Status.SUCCESS
