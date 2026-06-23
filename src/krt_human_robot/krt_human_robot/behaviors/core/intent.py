"""意图识别节点"""

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.config import RobotConfig


class RecognizeIntent(Behaviour):
    """
    基于关键词匹配的意图识别。

    读取 blackboard.user_command，匹配 config.intent_patterns 中的关键词。
    匹配到则写入对应的 intent 名称；
    匹配不到任何关键词时 intent 设为 "chat" (交给 LLM 处理)。
    """

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self.config = config

        self.blackboard = self.attach_blackboard_client(
            name="RecognizeIntent", namespace="dialog"
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.WRITE
        )

    def update(self):
        command = self.blackboard.user_command

        # 按优先级匹配意图关键词
        for intent_name, keywords in self.config.intent_patterns.items():
            for keyword in keywords:
                if keyword in command:
                    self.logger.info(
                        f"意图: {intent_name} (匹配关键词: '{keyword}')"
                    )
                    self.blackboard.intent = intent_name
                    return Status.SUCCESS

        # 没有匹配到，交给 LLM 自由对话
        self.logger.info("意图: chat (无关键词匹配)")
        self.blackboard.intent = "chat"
        return Status.SUCCESS
