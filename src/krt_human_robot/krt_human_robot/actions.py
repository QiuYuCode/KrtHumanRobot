"""Core-only behavior actions."""

from __future__ import annotations

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.behaviors.core.actions.back_to_wakeup import BackToWakeUp
from krt_human_robot.behaviors.core.actions.default_response import DefaultResponse
from krt_human_robot.behaviors.core.actions.fixed_response import FixedResponseAction
from krt_human_robot.behaviors.core.actions.gripper import GripperAction, execute_gripper_action
from krt_human_robot.behaviors.core.actions.llm_dialog import LLMDialogAction
from krt_human_robot.behaviors.core.actions.navigation import NavigationAction, execute_navigate
from krt_human_robot.behaviors.core.actions.robot_arm import RobotArmAction, execute_robot_arm
from krt_human_robot.behaviors.voice import speak_blocking
from voice_interfaces.srv import SynthesizeSpeech


class VisionDisabledAction(Behaviour):
    """Explicitly blocks old camera intents until the camera subsystem is redesigned."""

    _INTENTS = {
        "take_photo",
        "record_video",
        "describe_scene",
        "describe_left_palm",
        "describe_right_palm",
    }

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(
            name="VisionDisabledAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="intent", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def update(self):
        if self.blackboard.intent not in self._INTENTS:
            return Status.FAILURE
        self.blackboard.response_text = "视觉功能正在重构，暂时还不能使用。"
        return Status.SUCCESS


class CorePlanExecutor(Behaviour):
    """Execute planner steps owned by the core tree, with camera steps disabled."""

    _VISION_STEPS = {"take_photo", "record_video", "describe_scene"}

    def __init__(self, name: str, config):
        super().__init__(name)
        self._config = config
        self._node = None
        self._tts_client = None
        self.blackboard = self.attach_blackboard_client(
            name="CorePlanExecutor", namespace="dialog"
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

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is not None:
            self._tts_client = self._node.create_client(
                SynthesizeSpeech, "/voice/tts/synthesize"
            )

    def update(self):
        intent = self.blackboard.intent
        plan = getattr(self.blackboard, "action_plan", None) or []
        if intent == "chat" or not plan:
            return Status.SUCCESS

        results = []
        for i, step in enumerate(plan, 1):
            name = step["name"]
            args = step.get("args", {})
            try:
                result = self._execute_action(name, args)
                results.append(result)
                self._speak(f"第{i}步完成。{result}")
            except Exception as exc:
                error_msg = f"{name} 执行失败: {exc}"
                results.append(error_msg)
                self._speak(f"第{i}步失败。{error_msg}")

        self.blackboard.response_text = "；".join(results) if results else "计划执行完成。"
        return Status.SUCCESS

    def _execute_action(self, name: str, args: dict) -> str:
        if name in self._VISION_STEPS:
            return "视觉功能正在重构，暂时还不能使用。"
        if name == "navigate":
            return execute_navigate(self._config, args.get("destination", "未知位置"))
        if name == "control_robot_arm":
            return execute_robot_arm(
                config=self._config,
                action=args.get("action", ""),
                arm_side=args.get("arm_side"),
                operation=args.get("operation"),
                group_name=args.get("group_name"),
            )
        if name == "control_gripper":
            return execute_gripper_action(
                self._config,
                {"hand": args.get("hand", ""), "action": args.get("action", "")},
            )
        if name == "exit_conversation":
            return "好的，我先休息了。"
        return f"未知动作: {name}"

    def _speak(self, text: str) -> None:
        speak_blocking(self._node, self._tts_client, text)


__all__ = [
    "BackToWakeUp",
    "CorePlanExecutor",
    "DefaultResponse",
    "FixedResponseAction",
    "GripperAction",
    "LLMDialogAction",
    "NavigationAction",
    "RobotArmAction",
    "VisionDisabledAction",
]
