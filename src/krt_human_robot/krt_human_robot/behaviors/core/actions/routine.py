"""Named routine action node."""

from __future__ import annotations

from dataclasses import dataclass
import time

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


class RoutineVoiceTriggerAmbiguous(ValueError):
    """Raised when one utterance has equally specific routine matches."""


def resolve_routine_voice_trigger(
    database, command: str
) -> MatchedRoutine | None:
    matched: list[MatchedRoutine] = []
    for item in database.list_routine_voice_triggers():
        routine_name = str(item.get("routine_name", "")).strip()
        if not routine_name:
            continue
        for raw_keyword in item.get("keywords", []):
            keyword = str(raw_keyword).strip()
            if keyword and keyword in command:
                matched.append(MatchedRoutine(
                    routine_name=routine_name,
                    response_text=str(item.get("response_text", "")).strip(),
                    priority=0,
                    matched_keyword=keyword,
                ))
    if not matched:
        return None
    longest = max(len(item.matched_keyword) for item in matched)
    most_specific = [
        item for item in matched if len(item.matched_keyword) == longest
    ]
    routine_names = sorted({item.routine_name for item in most_specific})
    if len(routine_names) > 1:
        names = "、".join(routine_names)
        raise RoutineVoiceTriggerAmbiguous(f"语音指令同时命中多个动作编排：{names}")
    return most_specific[0]


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


class RoutineVoiceAction(Behaviour):
    """Resolve a Web-managed voice trigger before normal command dispatch."""

    def __init__(self, name: str, config: RobotConfig, database) -> None:
        super().__init__(name)
        self._config = config
        self._database = database
        self._node = None
        self._client = None
        self._matched: MatchedRoutine | None = None
        self._goal_future = None
        self._result_future = None
        self._deadline = 0.0
        self._terminal_response: str | None = None
        self.blackboard = self.attach_blackboard_client(
            name="RoutineVoiceAction", namespace="dialog"
        )
        self.blackboard.register_key(
            key="user_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="response_text", access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is not None:
            self._client = ActionClient(
                self._node,
                RunRoutine,
                getattr(
                    self._config,
                    "routine_action",
                    "/krt_task/run_routine",
                ),
            )

    def initialise(self) -> None:
        """Resolve one voice trigger; ROS futures advance in later tree ticks."""
        self._matched = None
        self._goal_future = None
        self._result_future = None
        self._deadline = 0.0
        self._terminal_response = None
        if self._database is None:
            return

        try:
            self._matched = resolve_routine_voice_trigger(
                self._database, self.blackboard.user_command
            )
        except RoutineVoiceTriggerAmbiguous as exc:
            self.logger.warning(str(exc))
            self._terminal_response = f"{exc}，请说得更明确。"
        except Exception as exc:
            self.logger.error(f"读取语音动作配置失败: {exc}")
            self._terminal_response = "语音动作配置暂不可用，请稍后再试。"

        if self._matched is not None:
            self.logger.info(
                f"语音触发动作编排: {self._matched.routine_name} "
                f"(匹配关键词: '{self._matched.matched_keyword}')"
            )

    def _failure_response(self, error: Exception | str) -> Status:
        message = str(error)
        if message == "routine 正在执行":
            self.blackboard.response_text = (
                f"流程“{self._matched.routine_name}”正在执行，请稍候。"
            )
        else:
            self.blackboard.response_text = f"流程执行失败：{message}"
        return Status.SUCCESS

    def update(self):
        if self._database is None:
            return Status.FAILURE
        if self._terminal_response is not None:
            self.blackboard.response_text = self._terminal_response
            return Status.SUCCESS
        if self._matched is None:
            return Status.FAILURE
        if self._client is None:
            return self._failure_response("routine ROS 客户端尚未初始化")

        try:
            now = time.monotonic()
            if self._result_future is not None:
                if not self._result_future.done():
                    if now > self._deadline:
                        return self._failure_response(
                            f"routine 执行超时: {self._matched.routine_name}"
                        )
                    return Status.RUNNING
                result = self._result_future.result().result
                self._result_future = None
                if not result.success:
                    return self._failure_response(result.message)
                self.blackboard.response_text = (
                    self._matched.response_text
                    or f"流程“{self._matched.routine_name}”已完成。"
                )
                return Status.SUCCESS

            if self._goal_future is not None:
                if not self._goal_future.done():
                    if now > self._deadline:
                        return self._failure_response(
                            f"routine 目标发送超时: {self._matched.routine_name}"
                        )
                    return Status.RUNNING
                goal_handle = self._goal_future.result()
                self._goal_future = None
                if goal_handle is None or not goal_handle.accepted:
                    return self._failure_response(
                        f"routine 被拒绝: {self._matched.routine_name}"
                    )
                self._result_future = goal_handle.get_result_async()
                timeout = float(
                    getattr(self._config, "routine_action_timeout_s", 120.0)
                )
                self._deadline = now + timeout
                return Status.RUNNING

            if not self._client.server_is_ready():
                if self._deadline == 0.0:
                    self._deadline = now + 5.0
                if now > self._deadline:
                    return self._failure_response("routine action server 不可用")
                return Status.RUNNING

            goal = RunRoutine.Goal()
            goal.routine_name = self._matched.routine_name
            self._goal_future = self._client.send_goal_async(goal)
            self._deadline = now + 5.0
            return Status.RUNNING
        except Exception as exc:
            self.logger.exception("语音动作编排执行失败")
            return self._failure_response(exc)
