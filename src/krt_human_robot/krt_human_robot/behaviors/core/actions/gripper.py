"""双手夹爪控制动作节点。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from threading import Lock
from typing import Any

import py_trees
import rclpy
from loguru import logger
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from rclpy.action import ActionClient

from hands_control_interfaces.action import HandControl
from hands_control_interfaces.srv import (
    GetApproachingValue,
    GetNormalPressure,
    GetTangentPressure,
)

from krt_human_robot.config import RobotConfig


_HAND_ALIASES = {
    "left": "left",
    "left_hand": "left",
    "左": "left",
    "左手": "left",
    "右": "right",
    "右手": "right",
    "right": "right",
    "right_hand": "right",
}

# 子串匹配时按 key 长度从长到短尝试，避免「握」抢先匹配「握手」等
_ACTION_ALIASES: dict[str, str] = {
    "shake": "shake",
    "move": "shake",
    "handshake": "handshake",
    "close": "handshake",
    "open": "open",
    "动动": "shake",
    "动一下": "shake",
    "动一动": "shake",
    "活动一下": "shake",
    "活动活动": "shake",
    "晃一下": "shake",
    "张开": "open",
    "放开": "open",
    "松一下": "open",
    "放一下": "open",
    "松开": "open",
    "放松": "open",
    "松开手": "open",
    "握手": "handshake",
    "握一下": "handshake",
    "捏一下": "handshake",
    "握": "handshake",
}


@dataclass
class _HandSpec:
    side: str
    adapter_index: int
    device_id: int
    has_pressure_sensor: bool


class RosGripperManager:
    """DexHand 的 ROS 接口管理器。"""

    def __init__(self, config: RobotConfig, node):
        self._config = config
        self._node = node
        self._lock = Lock()
        self._hand_clients: dict[str, ActionClient] = {}
        self._pressure_clients: dict[str, dict[str, Any]] = {}

    def _hand_spec(self, side: str) -> _HandSpec:
        raw = self._config.left_gripper if side == "left" else self._config.right_gripper
        return _HandSpec(
            side=side,
            adapter_index=int(raw.get("adapter_index", 0)),
            device_id=int(raw.get("device_id", 0x01)),
            has_pressure_sensor=bool(raw.get("has_pressure_sensor", False)),
        )

    def _hand_client(self, side: str) -> ActionClient:
        with self._lock:
            client = self._hand_clients.get(side)
            if client is None:
                client = ActionClient(
                    self._node,
                    HandControl,
                    f"/{side}/hand_control",
                )
                self._hand_clients[side] = client
            return client

    def _pressure_client(self, side: str, metric: str):
        spec = self._hand_spec(side)
        if not spec.has_pressure_sensor:
            return None

        service_map = {
            "normal": (GetNormalPressure, "get_normal_pressure"),
            "tangent": (GetTangentPressure, "get_tangent_pressure"),
            "approach": (GetApproachingValue, "get_approaching_value"),
        }
        if metric not in service_map:
            raise ValueError(f"不支持的压力类型: {metric}")

        with self._lock:
            side_clients = self._pressure_clients.setdefault(side, {})
            client = side_clients.get(metric)
            if client is None:
                service_type, service_name = service_map[metric]
                client = self._node.create_client(
                    service_type,
                    f"/{side}/{service_name}",
                )
                side_clients[metric] = client
            return client

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

    def _call_action(self, side: str, goal: HandControl.Goal, timeout: float):
        client = self._hand_client(side)
        if not client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"动作服务器不可用: /{side}/hand_control")

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=timeout)
        if not send_future.done():
            raise RuntimeError(f"动作目标发送超时: /{side}/hand_control")

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"动作目标被拒绝: /{side}/hand_control")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout)
        if not result_future.done():
            raise RuntimeError(f"动作执行超时: /{side}/hand_control")

        result = result_future.result().result
        if not result.success:
            raise RuntimeError(result.message)
        return result

    def move_fingers(
        self,
        side: str,
        target: int,
        speed: int,
        control_mode: int,
        delay_ms: int,
        finger_ids: list[int],
        inter_finger_delay: float = 0.0,
    ) -> None:
        spec = self._hand_spec(side)
        finger_ids = [int(fid) for fid in finger_ids]
        timeout = max(5.0, float(delay_ms) * 0.1 + 5.0)

        if not finger_ids:
            return

        if finger_ids == [1, 2, 3]:
            goal = HandControl.Goal()
            goal.adapter_index = spec.adapter_index
            goal.finger_id = 0
            goal.position = int(target)
            goal.speed = int(speed)
            goal.force = int(control_mode)
            goal.wait_time = int(delay_ms)
            self._call_action(side, goal, timeout)
            return

        for index, finger_id in enumerate(finger_ids):
            goal = HandControl.Goal()
            goal.adapter_index = spec.adapter_index
            goal.finger_id = int(finger_id)
            goal.position = int(target)
            goal.speed = int(speed)
            goal.force = int(control_mode)
            goal.wait_time = int(delay_ms)
            self._call_action(side, goal, timeout)
            if inter_finger_delay > 0 and index + 1 < len(finger_ids):
                time.sleep(inter_finger_delay)

    def read_pressure_text(self, side: str, finger_ids: list[int]) -> str:
        spec = self._hand_spec(side)
        if not spec.has_pressure_sensor:
            return ""

        values: list[float] = []
        for finger_id in finger_ids:
            value = self.read_normal_pressure(side, finger_id)
            if value is None:
                continue
            values.append(float(value))

        if not values:
            return ""

        peak = max(abs(v) for v in values)
        if peak < 1e-6:
            return "，压力读数接近零。"
        if peak < 0.5:
            return "，压力读数较低。"
        return "，压力读数已更新。"

    def read_normal_pressure(self, side: str, finger_id: int) -> float | None:
        return self._read_pressure_metric(side, "normal", finger_id)

    def read_tangent_pressure(self, side: str, finger_id: int) -> float | None:
        return self._read_pressure_metric(side, "tangent", finger_id)

    def read_approaching_value(self, side: str, finger_id: int) -> float | None:
        return self._read_pressure_metric(side, "approach", finger_id)

    def _read_pressure_metric(
        self,
        side: str,
        metric: str,
        finger_id: int,
    ) -> float | None:
        spec = self._hand_spec(side)
        if not spec.has_pressure_sensor:
            return None

        client = self._pressure_client(side, metric)
        if client is None:
            return None

        if finger_id not in [0x01, 0x02, 0x03]:
            raise ValueError(f"无效的 finger_id: {finger_id} (支持 1,2,3)")

        request = client.srv_type.Request()
        request.finger_id = int(finger_id)
        try:
            response = self._call_service(client, request, timeout=2.0)
        except Exception as exc:
            logger.warning(
                "压力读取失败: side={}, metric={}, finger_id={}, error={}",
                side,
                metric,
                finger_id,
                exc,
            )
            return None

        if not response.success or not response.available:
            return None
        return float(response.value)


_MANAGER: RosGripperManager | None = None


def _get_manager(config: RobotConfig, node=None) -> RosGripperManager:
    global _MANAGER
    if _MANAGER is None:
        if node is None:
            raise RuntimeError("夹爪 ROS 管理器尚未初始化")
        _MANAGER = RosGripperManager(config, node)
    return _MANAGER


def _parse_side_action(command_or_args: str | dict[str, Any]) -> tuple[str | None, str | None]:
    if isinstance(command_or_args, dict):
        side_raw = str(command_or_args.get("hand", "")).strip().lower()
        action_raw = str(command_or_args.get("action", "")).strip().lower()
        side = _HAND_ALIASES.get(side_raw)
        action = _ACTION_ALIASES.get(action_raw, action_raw if action_raw else None)
        return side, action

    command = str(command_or_args)
    side = None
    if "左手" in command:
        side = "left"
    elif "右手" in command:
        side = "right"
    elif "左" in command:
        side = "left"
    elif "右" in command:
        side = "right"

    action = None
    for key, normalized in sorted(_ACTION_ALIASES.items(), key=lambda kv: -len(kv[0])):
        if key in command:
            action = normalized
            break
    return side, action


def execute_gripper_action(
    config: RobotConfig,
    command_or_args: str | dict[str, Any],
    node=None,
) -> str:
    """夹爪控制核心逻辑。"""
    if not config.gripper_enabled:
        return "夹爪功能未启用。"

    side, action = _parse_side_action(command_or_args)
    if side is None:
        return config.tts_responses.get("gripper_missing_side", "请说明左手还是右手。")
    if action not in {"shake", "open", "handshake"}:
        return "夹爪动作暂不支持，可以说动动、动一下、张开、放一下、握手、握一下。"

    manager = _get_manager(config, node)
    finger_ids = [int(fid) for fid in config.gripper_finger_ids]
    speed_close = int(getattr(config, "gripper_speed_close", config.gripper_default_speed))
    speed_open = int(getattr(config, "gripper_speed_open", config.gripper_default_speed))
    control_mode = int(config.gripper_control_mode)
    delay_ms = int(config.gripper_exec_delay_ms)
    pause_close = float(getattr(config, "gripper_shake_pause_close", 0.5))
    pause_open = float(getattr(config, "gripper_shake_pause_open", 0.5))
    inter = float(getattr(config, "gripper_inter_finger_delay", 0.0))

    spec = manager._hand_spec(side)
    logger.info(
        "执行夹爪动作: side={}, action={}, device_id={}",
        side,
        action,
        spec.device_id,
    )

    if action == "open":
        manager.move_fingers(
            side,
            int(config.gripper_open_value),
            speed_open,
            control_mode,
            delay_ms,
            finger_ids,
            inter,
        )
        time.sleep(max(pause_open, 0.35))
        return f"{'左' if side == 'left' else '右'}手已张开。"

    if action == "handshake":
        manager.move_fingers(
            side,
            int(config.gripper_close_value),
            speed_close,
            control_mode,
            delay_ms,
            finger_ids,
            inter,
        )
        time.sleep(max(0.35, pause_close * 0.5))
        pressure_text = manager.read_pressure_text(side, finger_ids)
        return f"{'左' if side == 'left' else '右'}手已握紧。{pressure_text}".rstrip("。") + "。"

    cycles = max(1, int(config.gripper_shake_cycles))
    for _ in range(cycles):
        manager.move_fingers(
            side,
            int(config.gripper_close_value),
            speed_close,
            control_mode,
            delay_ms,
            finger_ids,
            inter,
        )
        time.sleep(pause_close)
        manager.move_fingers(
            side,
            int(config.gripper_open_value),
            speed_open,
            control_mode,
            delay_ms,
            finger_ids,
            inter,
        )
        time.sleep(pause_open)

    pressure_text = manager.read_pressure_text(side, finger_ids)
    return f"{'左' if side == 'left' else '右'}手动了一下。{pressure_text}".rstrip("。") + "。"


class GripperAction(Behaviour):
    """夹爪行为树节点。"""

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self._config = config
        self._node = None
        self.blackboard = self.attach_blackboard_client(
            name="GripperAction", namespace="dialog"
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

    def setup(self, **kwargs):
        self._node = kwargs.get("node")
        if self._node is not None:
            _get_manager(self._config, self._node)

    def update(self) -> Status:
        if self.blackboard.intent != "gripper_control":
            return Status.FAILURE

        try:
            command = self.blackboard.user_command
            self.blackboard.response_text = execute_gripper_action(
                self._config,
                command,
                node=self._node,
            )
            return Status.SUCCESS
        except Exception as e:
            logger.exception("夹爪控制失败")
            self.blackboard.response_text = f"夹爪控制失败: {e}"
            return Status.SUCCESS
