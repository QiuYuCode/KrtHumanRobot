"""Named routine action node."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import py_trees
import rclpy
from krt_task_interfaces.action import RunRoutine
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from rclpy.action import ActionClient

from krt_human_robot.config import RobotConfig


@dataclass
class MatchedRoutine:
    routine_name: str
    response_text: str
    priority: int
    matched_keyword: str


def resolve_routine(config: RobotConfig, command: str) -> MatchedRoutine | None:
    matched: list[MatchedRoutine] = []
    for item in getattr(config, "routine_keyword_actions", []) or []:
        if not isinstance(item, dict):
            continue
        keywords = item.get("keywords", [])
        if isinstance(keywords, str):
            keywords = [keywords]
        routine_name = str(item.get("routine_name", "")).strip()
        if not routine_name:
            continue
        try:
            priority = int(item.get("priority", 0))
        except Exception:
            priority = 0
        for keyword in keywords:
            keyword = str(keyword).strip()
            if keyword and keyword in command:
                matched.append(MatchedRoutine(
                    routine_name=routine_name,
                    response_text=str(item.get("response_text", "")).strip(),
                    priority=priority,
                    matched_keyword=keyword,
                ))
    if not matched:
        return None
    matched.sort(key=lambda x: (x.priority, len(x.matched_keyword)), reverse=True)
    return matched[0]


def execute_routine(
    config: RobotConfig,
    command: str = "",
    routine_name: str | None = None,
    node=None,
) -> str:
    resolved = routine_name or ""
    response_text = ""
    if not resolved:
        matched = resolve_routine(config, command)
        if matched is not None:
            resolved = matched.routine_name
            response_text = matched.response_text
    if not resolved:
        return "请说明要执行的流程名称。"
    if node is None:
        raise RuntimeError("routine 执行需要 rclpy node。")

    client = ActionClient(
        node,
        RunRoutine,
        getattr(config, "routine_action", "/krt_task/run_routine"),
    )
    timeout = float(getattr(config, "routine_action_timeout_s", 120.0))
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("routine action server 不可用")
    goal = RunRoutine.Goal()
    goal.routine_name = resolved
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError(f"routine 被拒绝: {resolved}")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
    if not result_future.done():
        raise RuntimeError(f"routine 执行超时: {resolved}")
    result = result_future.result().result
    if not result.success:
        raise RuntimeError(result.message)
    return response_text or f"流程“{resolved}”已完成。"


class RoutineAction(Behaviour):
    """Run a named routine via /krt_task/run_routine."""

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self._config = config
        self._node = None
        self.blackboard = self.attach_blackboard_client(
            name="RoutineAction", namespace="dialog"
        )
        self.blackboard.register_key(key="intent", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self._node = kwargs.get("node")

    def update(self):
        if self.blackboard.intent != "routine":
            return Status.FAILURE
        try:
            self.blackboard.response_text = execute_routine(
                self._config,
                command=self.blackboard.user_command,
                node=self._node,
            )
        except Exception as exc:
            self.blackboard.response_text = f"流程执行失败：{exc}"
        return Status.SUCCESS
