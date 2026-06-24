"""ROS 导航动作节点"""

from __future__ import annotations

import re

from loguru import logger

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.adapters.navigation import NavigationResult, RangerNavAdapter
from krt_human_robot.config import RobotConfig


# ============================================================================
# 独立执行函数 (供 Behaviour 节点和 planner tool 共同调用)
# ============================================================================

def execute_navigate(config: RobotConfig, destination: str) -> str:
    """导航到保存点位。返回执行结果描述。"""
    logger.info("执行导航: {}", destination)
    name = _extract_waypoint_name(destination, ["带我去", "前往", "导航到", "导航", "去"])
    if not name:
        return "请说明要去的点位名称。"
    result = RangerNavAdapter(config).start_cruise([name])
    return result.message


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
        "mark_waypoint",
        "start_cruise",
        "cruise_waypoints",
        "loop_cruise",
        "repeat_cruise",
        "stop_cruise",
        "continue_waypoint",
        "remove_waypoint",
        "list_waypoints",
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
        elif intent == "mark_waypoint":
            name = _extract_waypoint_name(command, ["打点", "保存点位", "记录点位", "标记点位"])
            result = self._adapter.mark_waypoint(name or None)
        elif intent == "start_cruise":
            result = self._adapter.start_cruise()
        elif intent == "cruise_waypoints":
            names = _extract_waypoint_names(command, ["巡航指定点", "巡航点位", "巡航"])
            result = self._adapter.start_cruise(names)
        elif intent == "loop_cruise":
            names = _extract_waypoint_names(command, ["循环巡航", "一直巡航"])
            result = self._adapter.start_cruise(names, loop=True)
        elif intent == "repeat_cruise":
            repeat = _extract_repeat_count(command)
            names = _extract_waypoint_names(command, ["巡航"])
            result = self._adapter.start_cruise(names, repeat=repeat)
        elif intent == "stop_cruise":
            result = self._adapter.stop_cruise()
        elif intent == "continue_waypoint":
            result = self._adapter.continue_waypoint_input()
        elif intent == "remove_waypoint":
            name = _extract_waypoint_name(command, ["删除点位", "删除", "移除点位", "移除"])
            result = self._adapter.remove_waypoint(name)
        elif intent == "list_waypoints":
            result = self._adapter.list_waypoints()
        else:
            name = _extract_waypoint_name(command, ["带我去", "前往", "导航到", "导航", "去"])
            if not name:
                result = NavigationResult(False, "请说明要去的点位名称。")
            else:
                result = self._adapter.start_cruise([name])

        self.blackboard.response_text = result.message
        return Status.SUCCESS


def _extract_waypoint_name(command: str, keywords: list[str]) -> str:
    text = (command or "").strip()
    for keyword in keywords:
        if keyword in text:
            text = text.split(keyword, 1)[1].strip()
            break
    return _clean_name_text(text)


def _extract_waypoint_names(command: str, keywords: list[str]) -> list[str]:
    name_text = _extract_waypoint_name(command, keywords)
    if not name_text:
        return []
    return [
        item.strip()
        for item in re.split(r"[、,，\s]+", name_text)
        if item.strip() and item.strip() not in {"一遍", "两遍", "三遍", "四遍", "五遍"}
    ]


def _clean_name_text(text: str) -> str:
    text = re.sub(r"(一遍|两遍|二遍|三遍|四遍|五遍|六遍|七遍|八遍|九遍|十遍)", "", text)
    text = re.sub(r"(一次|两次|二次|三次|四次|五次|六次|七次|八次|九次|十次)", "", text)
    return text.strip(" ，,。.!！?？")


def _extract_repeat_count(command: str) -> int:
    text = command or ""
    match = re.search(r"(\d+)\s*[遍次]", text)
    if match:
        return max(1, int(match.group(1)))
    numbers = {
        "一": 1,
        "二": 2,
        "两": 2,
        "三": 3,
        "四": 4,
        "五": 5,
        "六": 6,
        "七": 7,
        "八": 8,
        "九": 9,
        "十": 10,
    }
    match = re.search(r"([一二两三四五六七八九十])\s*[遍次]", text)
    if match:
        return numbers[match.group(1)]
    return 1
