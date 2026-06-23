"""双手夹爪控制动作节点。"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Any

from loguru import logger

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.config import RobotConfig

_DEXHAND_IMPORT_ERROR: Exception | None = None
try:
    from dexhand.dexhand import AdapterType, DexHand021S
except (ImportError, OSError) as e:
    _DEXHAND_IMPORT_ERROR = e
    # 回退：在未走 uv 环境时，尝试从仓库同级目录加载 dexhand_sdk_python。
    repo_root = Path(__file__).resolve().parents[2]
    local_dexhand_repo = repo_root.parent / "dexhand_sdk_python"
    if local_dexhand_repo.exists():
        sys.path.insert(0, str(local_dexhand_repo))
        try:
            from dexhand.dexhand import AdapterType, DexHand021S
            _DEXHAND_IMPORT_ERROR = None
        except (ImportError, OSError) as inner_e:
            _DEXHAND_IMPORT_ERROR = inner_e
            AdapterType = None
            DexHand021S = None
    else:
        AdapterType = None
        DexHand021S = None


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


class DexHandManager:
    """DexHand 双手实例管理器。"""

    def __init__(self, config: RobotConfig):
        self._config = config
        self._lock = Lock()
        self._hands: dict[str, Any] = {}

    def _adapter_type(self) -> Any:
        if AdapterType is None:
            detail = f" 原始错误: {_DEXHAND_IMPORT_ERROR}" if _DEXHAND_IMPORT_ERROR else ""
            raise RuntimeError(f"未安装 dexhand SDK，请先安装并配置 dexhand_sdk_python。{detail}")

        mapping = {
            "zlg_mini": AdapterType.ZLG_MINI,
            "zlg_200u": AdapterType.ZLG_200U,
            "lys_mini": AdapterType.LYS_MINI,
        }
        key = str(self._config.gripper_adapter_type).strip().lower()
        if key not in mapping:
            raise RuntimeError(f"不支持的 gripper_adapter_type: {self._config.gripper_adapter_type}")
        return mapping[key]

    def _hand_spec(self, side: str) -> _HandSpec:
        raw = self._config.left_gripper if side == "left" else self._config.right_gripper
        return _HandSpec(
            side=side,
            adapter_index=int(raw.get("adapter_index", 0)),
            device_id=int(raw.get("device_id", 0x01)),
            has_pressure_sensor=bool(raw.get("has_pressure_sensor", False)),
        )

    def get_hand(self, side: str) -> tuple[Any, _HandSpec]:
        with self._lock:
            if side in self._hands:
                return self._hands[side], self._hand_spec(side)

            if DexHand021S is None:
                detail = f" 原始错误: {_DEXHAND_IMPORT_ERROR}" if _DEXHAND_IMPORT_ERROR else ""
                raise RuntimeError(f"未安装 dexhand SDK，请先安装并配置 dexhand_sdk_python。{detail}")

            spec = self._hand_spec(side)
            if self._hands:
                time.sleep(float(self._config.gripper_second_hand_init_delay))
            hand = DexHand021S(
                adapter_type=self._adapter_type(),
                adapter_index=spec.adapter_index,
            )
            hand.listen(enable=True)
            hand.enable_realtime_response(device_id=spec.device_id, enable=True)

            hand.clear_error(spec.device_id)
            if self._config.gripper_set_safe_current:
                max_current = int(self._config.gripper_safe_current)
                for fid in self._config.gripper_finger_ids:
                    hand.set_safe_current(spec.device_id, int(fid), max_current)

            hand.reset_joints(spec.device_id)
            time.sleep(float(self._config.gripper_post_reset_sleep))
            self._hands[side] = hand
            return hand, spec


_MANAGER: DexHandManager | None = None


def _get_manager(config: RobotConfig) -> DexHandManager:
    global _MANAGER
    if _MANAGER is None:
        _MANAGER = DexHandManager(config)
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


def _move_all_fingers(
    hand: Any,
    device_id: int,
    target: int,
    speed: int,
    mode: int,
    delay_ms: int,
    finger_ids: list[int],
    inter_finger_delay: float = 0.0,
) -> None:
    for i, finger_id in enumerate(finger_ids):
        hand.move_finger(device_id, finger_id, target, speed, mode, delay_ms)
        if inter_finger_delay > 0 and i + 1 < len(finger_ids):
            time.sleep(inter_finger_delay)


def _clear_finger_errors(hand: Any, device_id: int, _finger_ids: list[int]) -> None:
    hand.clear_error(device_id)


def _read_pressure_text(hand: Any, spec: _HandSpec, finger_ids: list[int]) -> str:
    """返回仅含中文的简短说明，避免本地 VITS 对英文/冒号/数字发音崩溃。"""
    if not spec.has_pressure_sensor:
        return ""
    values: list[float] = []
    for finger_id in finger_ids:
        v = float(hand.get_normal_pressure(spec.device_id, finger_id))
        values.append(v)
        logger.debug("压力 device_id={} finger={} value={}", spec.device_id, finger_id, v)
    if not values:
        return ""
    peak = max(abs(v) for v in values)
    if peak < 1e-6:
        return "，压力读数接近零。"
    if peak < 0.5:
        return "，压力读数较低。"
    return "，压力读数已更新。"


def execute_gripper_action(config: RobotConfig, command_or_args: str | dict[str, Any]) -> str:
    """夹爪控制核心逻辑。"""
    if not config.gripper_enabled:
        return "夹爪功能未启用。"

    side, action = _parse_side_action(command_or_args)
    if side is None:
        return config.tts_responses.get("gripper_missing_side", "请说明左手还是右手。")
    if action not in {"shake", "open", "handshake"}:
        return "夹爪动作暂不支持，可以说动动、动一下、张开、放一下、握手、握一下。"

    hand, spec = _get_manager(config).get_hand(side)
    finger_ids = [int(fid) for fid in config.gripper_finger_ids]
    speed_close = int(getattr(config, "gripper_speed_close", config.gripper_default_speed))
    speed_open = int(getattr(config, "gripper_speed_open", config.gripper_default_speed))
    mode = int(config.gripper_control_mode)
    delay_ms = int(config.gripper_exec_delay_ms)
    pause_close = float(getattr(config, "gripper_shake_pause_close", 0.5))
    pause_open = float(getattr(config, "gripper_shake_pause_open", 0.5))
    inter = float(getattr(config, "gripper_inter_finger_delay", 0.0))

    logger.info("执行夹爪动作: side={}, action={}, device_id={}", side, action, spec.device_id)

    if action == "open":
        _move_all_fingers(
            hand,
            spec.device_id,
            int(config.gripper_open_value),
            speed_open,
            mode,
            delay_ms,
            finger_ids,
            inter,
        )
        _clear_finger_errors(hand, spec.device_id, finger_ids)
        time.sleep(max(pause_open, 0.35))
        return f"{'左' if side == 'left' else '右'}手已张开。"

    if action == "handshake":
        _move_all_fingers(
            hand,
            spec.device_id,
            int(config.gripper_close_value),
            speed_close,
            mode,
            delay_ms,
            finger_ids,
            inter,
        )
        _clear_finger_errors(hand, spec.device_id, finger_ids)
        time.sleep(max(0.35, pause_close * 0.5))
        pressure_text = _read_pressure_text(hand, spec, finger_ids)
        return f"{'左' if side == 'left' else '右'}手已握紧。{pressure_text}".rstrip("。") + "。"

    cycles = max(1, int(config.gripper_shake_cycles))
    for _ in range(cycles):
        _move_all_fingers(
            hand,
            spec.device_id,
            int(config.gripper_close_value),
            speed_close,
            mode,
            delay_ms,
            finger_ids,
            inter,
        )
        _clear_finger_errors(hand, spec.device_id, finger_ids)
        time.sleep(pause_close)
        _move_all_fingers(
            hand,
            spec.device_id,
            int(config.gripper_open_value),
            speed_open,
            mode,
            delay_ms,
            finger_ids,
            inter,
        )
        _clear_finger_errors(hand, spec.device_id, finger_ids)
        time.sleep(pause_open)
    pressure_text = _read_pressure_text(hand, spec, finger_ids)
    return f"{'左' if side == 'left' else '右'}手动了一下。{pressure_text}".rstrip("。") + "。"


class GripperAction(Behaviour):
    """夹爪行为树节点。"""

    def __init__(self, name: str, config: RobotConfig):
        super().__init__(name)
        self._config = config
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

    def update(self) -> Status:
        if self.blackboard.intent != "gripper_control":
            return Status.FAILURE

        try:
            command = self.blackboard.user_command
            self.blackboard.response_text = execute_gripper_action(self._config, command)
            return Status.SUCCESS
        except Exception as e:
            logger.exception("夹爪控制失败")
            self.blackboard.response_text = f"夹爪控制失败: {e}"
            return Status.SUCCESS
