"""机械臂控制动作节点（双臂示教/保存/回放）。"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from agx_action_group_interfaces.action import RunActionGroup
from agx_action_group_interfaces.srv import StartTeach, StopTeach
from loguru import logger
from rclpy.action import ActionClient

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.config import RobotConfig


def _normalize_side(text: str | None) -> str | None:
    if not text:
        return None
    t = text.lower().strip()
    if t in {"left", "左", "左边", "左臂", "左手"}:
        return "left"
    if t in {"right", "右", "右边", "右臂", "右手"}:
        return "right"
    return None


@dataclass
class ParsedRobotArmCommand:
    arm_side: str | None
    operation: str | None
    group_name: str | None
    raw_command: str


@dataclass
class MatchedKeywordAction:
    arm_side: str
    group_name: str
    response_text: str
    priority: int
    matched_keyword: str


class RobotArmTeachManager:
    """ROS service/action backed arm controller."""

    def __init__(self, config: RobotConfig, node):
        self._config = config
        self._node = node
        self._start_client = node.create_client(
            StartTeach,
            getattr(config, "robot_arm_teach_start_service", "/agx_action_group/start_teach"),
        )
        self._stop_client = node.create_client(
            StopTeach,
            getattr(config, "robot_arm_teach_stop_service", "/agx_action_group/stop_teach"),
        )
        self._action_client = ActionClient(
            node,
            RunActionGroup,
            getattr(config, "robot_arm_action_group_action", "/agx_action_group/run_action_group"),
        )

    def _call_service(self, client, request, timeout: float):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"ROS 服务不可用: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"ROS 服务超时: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"ROS 服务调用失败: {client.srv_name}")
        return result

    def enter_teach(self, side: str, group_name: str | None) -> str:
        req = StartTeach.Request()
        req.arm_target = side
        req.group_name = group_name or ""
        res = self._call_service(
            self._start_client,
            req,
            float(getattr(self._config, "robot_arm_teach_service_timeout_s", 8.0)),
        )
        if not res.success:
            raise RuntimeError(res.message)
        return f"{'左臂' if side == 'left' else '右臂'}已进入示教模式，请拖动机械臂完成示教。"

    def start_teach_async(self, side: str, group_name: str | None):
        if not self._start_client.service_is_ready():
            return None
        req = StartTeach.Request()
        req.arm_target = side
        req.group_name = group_name or ""
        return self._start_client.call_async(req)

    def exit_teach(self, side: str | None, group_name: str | None) -> str:
        resolved_side = _normalize_side(side) or ""
        req = StopTeach.Request()
        req.arm_target = resolved_side
        req.group_name = group_name or ""
        res = self._call_service(
            self._stop_client,
            req,
            float(getattr(self._config, "robot_arm_teach_service_timeout_s", 8.0)),
        )
        if not res.success:
            raise RuntimeError(res.message)
        side_text = "机械臂"
        if resolved_side in {"left", "right"}:
            side_text = "左臂" if resolved_side == "left" else "右臂"
        return f"{side_text}已退出示教，动作组“{res.group_name}”已保存。"

    def stop_teach_async(self, side: str | None, group_name: str | None):
        if not self._stop_client.service_is_ready():
            return None
        req = StopTeach.Request()
        req.arm_target = _normalize_side(side) or ""
        req.group_name = group_name or ""
        return self._stop_client.call_async(req)

    def run_group(self, side: str, group_name: str) -> str:
        timeout = float(getattr(self._config, "robot_arm_action_timeout_s", 60.0))
        if not self._action_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("动作组 action server 不可用")
        goal = RunActionGroup.Goal()
        goal.arm_target = side
        goal.group_name = group_name
        goal.repeat_count = int(getattr(self._config, "robot_arm_action_repeat_count", 1))
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=timeout)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"动作组被拒绝: {group_name}")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)
        if not result_future.done():
            raise RuntimeError(f"动作组执行超时: {group_name}")
        result = result_future.result().result
        if not result.success:
            raise RuntimeError(result.message)
        side_text = "左臂" if side == "left" else "右臂"
        return f"{side_text}已执行动作组“{group_name}”"


def _extract_group_name(command: str) -> str | None:
    patterns = [
        r"(?:保存为|保存成|动作组为|动作组叫|命名为|叫做)\s*([^\s，。,.；;]+)",
        r"动作组\s*([^\s，。,.；;]+)",
        r"执行(?:示教)?动作(?:组)?\s*([^\s，。,.；;]+)",
        r"回放(?:示教)?动作(?:组)?\s*([^\s，。,.；;]+)",
    ]
    for pat in patterns:
        matched = re.search(pat, command)
        if matched:
            name = matched.group(1).strip().strip("，。,.；;：:")
            if name:
                return name
    return None


def _iter_normalized_keyword_actions(config: RobotConfig) -> list[dict[str, Any]]:
    raw = getattr(config, "robot_arm_keyword_actions", []) or []
    if not isinstance(raw, list):
        return []
    actions: list[dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        keywords_raw = item.get("keywords", [])
        if isinstance(keywords_raw, str):
            keywords_raw = [keywords_raw]
        if not isinstance(keywords_raw, list):
            continue
        keywords = [str(k).strip() for k in keywords_raw if str(k).strip()]
        arm_side = _normalize_side(str(item.get("arm_side", "")).strip().lower())
        group_name = str(item.get("group_name", "")).strip()
        response_text = str(item.get("response_text", "")).strip()
        if not keywords or not arm_side or not group_name:
            continue
        try:
            priority = int(item.get("priority", 0))
        except Exception:
            priority = 0
        actions.append({
            "keywords": keywords,
            "arm_side": arm_side,
            "group_name": group_name,
            "response_text": response_text,
            "priority": priority,
        })
    return actions


def resolve_keyword_robot_arm_action(
    config: RobotConfig,
    command: str,
) -> MatchedKeywordAction | None:
    """按 priority + 关键词长度 匹配机械臂动作映射。"""
    if not command:
        return None
    matched: list[MatchedKeywordAction] = []
    for item in _iter_normalized_keyword_actions(config):
        for keyword in item["keywords"]:
            if keyword in command:
                matched.append(MatchedKeywordAction(
                    arm_side=item["arm_side"],
                    group_name=item["group_name"],
                    response_text=item["response_text"],
                    priority=item["priority"],
                    matched_keyword=keyword,
                ))

    if not matched:
        return None

    matched.sort(
        key=lambda x: (x.priority, len(x.matched_keyword)),
        reverse=True,
    )
    return matched[0]


def parse_robot_arm_command(command: str) -> ParsedRobotArmCommand:
    command = command.replace("试教", "示教")
    side = None
    if any(k in command for k in ["左臂", "左手", "左边", "左机械臂"]):
        side = "left"
    elif any(k in command for k in ["右臂", "右手", "右边", "右机械臂"]):
        side = "right"

    operation = None
    if any(k in command for k in ["进入拖动", "开始拖动", "开启拖动", "进入示教", "开始示教", "开启示教"]):
        operation = "enter_teach"
    elif any(k in command for k in [
        "退出拖动", "结束拖动", "停止拖动",
        "退出示教", "结束示教", "停止示教",
        "保存动作组", "保存示教",
    ]):
        operation = "exit_teach"
    elif (
        ("执行" in command or "回放" in command)
        and any(k in command for k in ["拖动", "示教", "动作组", "轨迹"])
    ):
        operation = "run_group"

    return ParsedRobotArmCommand(
        arm_side=side,
        operation=operation,
        group_name=_extract_group_name(command),
        raw_command=command,
    )


_MANAGER_CACHE: dict[tuple[int, int], RobotArmTeachManager] = {}


def _get_manager(config: RobotConfig, node) -> RobotArmTeachManager:
    if node is None:
        raise RuntimeError("机械臂 ROS 控制需要传入 rclpy node。")
    key = (id(config), id(node))
    manager = _MANAGER_CACHE.get(key)
    if manager is None:
        manager = RobotArmTeachManager(config, node)
        _MANAGER_CACHE[key] = manager
    return manager


def execute_robot_arm(
    config: RobotConfig,
    action: str | None = None,
    arm_side: str | None = None,
    operation: str | None = None,
    group_name: str | None = None,
    node=None,
) -> str:
    """机械臂示教控制入口。"""
    if not config.robot_arm_enabled:
        return "机械臂功能未启用。"

    mapped = resolve_keyword_robot_arm_action(config=config, command=action or "")
    if mapped is not None:
        manager = _get_manager(config, node)
        logger.info(
            "命中机械臂关键词映射: keyword={} side={} group={} priority={}",
            mapped.matched_keyword,
            mapped.arm_side,
            mapped.group_name,
            mapped.priority,
        )
        manager.run_group(mapped.arm_side, mapped.group_name)
        if mapped.response_text:
            return mapped.response_text
        side_text = "左臂" if mapped.arm_side == "left" else "右臂"
        return f"{side_text}已执行动作组“{mapped.group_name}”"

    parsed = parse_robot_arm_command(action or "")
    resolved_side = _normalize_side(arm_side) or parsed.arm_side
    resolved_operation = operation or parsed.operation
    resolved_group_name = group_name or parsed.group_name
    manager = _get_manager(config, node)

    logger.info(
        "执行机械臂指令: side={} op={} group={} action={}",
        resolved_side,
        resolved_operation,
        resolved_group_name,
        action,
    )

    if resolved_operation == "enter_teach":
        if resolved_side is None:
            return "请说明进入示教的是左臂还是右臂。"
        return manager.enter_teach(resolved_side, resolved_group_name)

    if resolved_operation == "exit_teach":
        return manager.exit_teach(resolved_side, resolved_group_name)

    if resolved_operation == "run_group":
        if resolved_side is None:
            return "请说明要执行动作组的是左臂还是右臂。"
        if not resolved_group_name:
            return "请说明要执行的动作组名称。"
        return manager.run_group(resolved_side, resolved_group_name)

    return (
        "未识别机械臂指令。请使用“左/右臂进入示教”、“退出示教并保存为动作名”"
        "或“左/右臂执行拖动动作: 动作名”。"
    )


class RobotArmAction(Behaviour):
    """机械臂动作节点。"""

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self._config = config
        self.blackboard = self.attach_blackboard_client(
            name="RobotArmAction", namespace="dialog"
        )
        self.blackboard.register_key(key="intent", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="user_command", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="response_text", access=py_trees.common.Access.WRITE)
        self._node = None
        self._manager = None
        self._future = None
        self._operation = None
        self._side = None
        self._group_name = None
        self._deadline = 0.0

    def setup(self, **kwargs):
        self._node = kwargs.get("node")

    def initialise(self):
        self._future = None
        self._operation = None
        self._side = None
        self._group_name = None
        self._deadline = time.time() + float(
            getattr(self._config, "robot_arm_teach_service_timeout_s", 8.0)
        )

        if self.blackboard.intent != "robot_arm":
            return
        command = self.blackboard.user_command
        self.logger.info(f"执行: 机械臂控制 ({command})")
        if not self._config.robot_arm_enabled:
            self.blackboard.response_text = "机械臂功能未启用。"
            return
        parsed = parse_robot_arm_command(command)
        self._side = parsed.arm_side
        self._operation = parsed.operation
        self._group_name = parsed.group_name
        if self._operation not in {"enter_teach", "exit_teach"}:
            return
        if self._operation == "enter_teach" and self._side is None:
            self.blackboard.response_text = "请说明进入示教的是左臂还是右臂。"
            return
        self._manager = _get_manager(self._config, self._node)

    def _finish_teach(self, result) -> Status:
        if result is None:
            self.blackboard.response_text = "机械臂操作失败：服务调用失败。"
        elif not result.success:
            self.blackboard.response_text = f"机械臂操作失败：{result.message}"
        elif self._operation == "enter_teach":
            side_text = "左臂" if self._side == "left" else "右臂"
            self.blackboard.response_text = f"{side_text}已进入示教模式，请拖动机械臂完成示教。"
        else:
            side_text = {"left": "左臂", "right": "右臂"}.get(self._side, "机械臂")
            if result.sample_count and result.group_name:
                self.blackboard.response_text = (
                    f"{side_text}已退出示教，动作组“{result.group_name}”已保存。"
                )
            elif result.group_name:
                self.blackboard.response_text = f"{side_text}已退出示教，但未录到动作，未保存。"
            else:
                self.blackboard.response_text = f"{side_text}已退出示教，未提供动作名，未保存。"
        return Status.SUCCESS

    def update(self):
        if self.blackboard.intent != "robot_arm":
            return Status.FAILURE
        if self._operation in {"enter_teach", "exit_teach"}:
            if self._manager is None:
                return Status.SUCCESS
            if self._future is None:
                self._future = (
                    self._manager.start_teach_async(self._side, self._group_name)
                    if self._operation == "enter_teach"
                    else self._manager.stop_teach_async(self._side, self._group_name)
                )
                if self._future is None:
                    if time.time() > self._deadline:
                        self.blackboard.response_text = "机械臂操作失败：示教服务不可用。"
                        return Status.SUCCESS
                    return Status.RUNNING
            if not self._future.done():
                if time.time() > self._deadline:
                    self.blackboard.response_text = "机械臂操作失败：示教服务超时。"
                    return Status.SUCCESS
                return Status.RUNNING
            try:
                return self._finish_teach(self._future.result())
            except Exception as exc:
                self.blackboard.response_text = f"机械臂操作失败：{exc}"
                return Status.SUCCESS
        try:
            self.blackboard.response_text = execute_robot_arm(
                config=self._config,
                action=self.blackboard.user_command,
                node=self._node,
            )
        except Exception as exc:
            self.blackboard.response_text = f"机械臂操作失败：{exc}"
        return Status.SUCCESS
