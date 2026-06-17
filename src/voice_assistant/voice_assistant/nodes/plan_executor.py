"""计划执行器 - 依次执行 LLM 规划的多步动作"""

from __future__ import annotations

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from voice_assistant.config import RobotConfig
from voice_assistant.nodes.actions.camera import execute_take_photo, execute_record_video
from voice_assistant.nodes.actions.gripper import execute_gripper_action
from voice_assistant.nodes.actions.navigation import execute_navigate
from voice_assistant.nodes.actions.robot_arm import execute_robot_arm
from voice_assistant.nodes.actions.vision import execute_describe_scene


class PlanExecutor(Behaviour):
    """
    执行 LLMTaskPlanner 输出的 action_plan。

    读取 blackboard.action_plan (list[dict])，依次调用对应的执行函数，
    汇总所有结果写入 response_text。

    当 intent == "chat" 时（无 action_plan），跳过执行，保留 planner 写入的
    response_text 作为纯对话回复。
    """

    def __init__(self, name: str, config: RobotConfig, engine=None):
        super().__init__(name)
        self._config = config
        self._engine = engine

        self.blackboard = self.attach_blackboard_client(
            name="PlanExecutor", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="action_plan", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def _execute_action(self, name: str, args: dict) -> str:
        """根据动作名称分发到对应的执行函数。"""
        if name == "take_photo":
            return execute_take_photo(self._config)

        if name == "record_video":
            duration = args.get("duration")
            return execute_record_video(
                self._config,
                duration=float(duration) if duration is not None else None,
            )

        if name == "navigate":
            destination = args.get("destination", "未知位置")
            return execute_navigate(destination)

        if name == "control_robot_arm":
            return execute_robot_arm(
                config=self._config,
                action=args.get("action", ""),
                arm_side=args.get("arm_side"),
                operation=args.get("operation"),
                group_name=args.get("group_name"),
            )

        if name == "control_gripper":
            hand = args.get("hand", "")
            action = args.get("action", "")
            return execute_gripper_action(self._config, {"hand": hand, "action": action})

        if name == "describe_scene":
            question = args.get("question", "请描述你看到的场景")
            return execute_describe_scene(self._config, question)

        if name == "exit_conversation":
            return "好的，我先休息了。"

        return f"未知动作: {name}"

    def update(self):
        intent = self.blackboard.intent
        plan = getattr(self.blackboard, "action_plan", None) or []

        if intent == "chat" or not plan:
            return Status.SUCCESS

        self.logger.info(f"开始执行计划: {len(plan)} 步")

        results = []
        for i, step in enumerate(plan, 1):
            name = step["name"]
            args = step.get("args", {})
            self.logger.info(f"  步骤 {i}/{len(plan)}: {name}({args})")

            try:
                result = self._execute_action(name, args)
                results.append(result)
                self.logger.info(f"  步骤 {i} 完成: {result}")
                if self._engine is not None:
                    self._engine.speak_blocking(f"第{i}步完成。{result}")
            except Exception as e:
                error_msg = f"{name} 执行失败: {e}"
                results.append(error_msg)
                self.logger.error(f"  步骤 {i} 失败: {e}")
                if self._engine is not None:
                    self._engine.speak_blocking(f"第{i}步失败。{error_msg}")

        if self._engine is not None:
            self.blackboard.response_text = "全部步骤执行完成。还有什么需要做的吗？"
        else:
            self.blackboard.response_text = "；".join(results)
        return Status.SUCCESS
