"""机械臂控制动作节点（双臂示教/保存/回放）。"""

from __future__ import annotations

import json
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from loguru import logger

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from krt_human_robot.config import RobotConfig

try:
    from pyAgxArm import AgxArmFactory, NeroFW, create_agx_arm_config
except ImportError:
    AgxArmFactory = None
    NeroFW = None
    create_agx_arm_config = None


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


class ArmTeachRuntime:
    """单机械臂示教运行时。"""

    def __init__(self, config: RobotConfig, arm_side: str):
        self._config = config
        self._arm_side = arm_side
        self._channel = str(config.robot_arm_channels.get(arm_side, "")).strip()
        if not self._channel:
            raise RuntimeError(f"{arm_side} 臂未配置通信通道。")

        self._arm = None
        self._connected = False
        self._enabled = False

        self._record_lock = threading.Lock()
        self._record_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._records: list[dict[str, Any]] = []
        self._is_recording = False
        self._current_group_name: str | None = None
        self._record_started_at: float | None = None

    def _ensure_sdk(self) -> None:
        if AgxArmFactory is None or create_agx_arm_config is None:
            raise RuntimeError("缺少 pyAgxArm 依赖，无法控制机械臂。")

    def _firmware(self):
        firmware_ver = str(self._config.robot_arm_firmware).strip().lower()
        if NeroFW is None:
            return None
        if firmware_ver == "v111" and hasattr(NeroFW, "V111"):
            return NeroFW.V111
        if hasattr(NeroFW, "DEFAULT"):
            return NeroFW.DEFAULT
        return None

    def _build_arm(self):
        self._ensure_sdk()
        arm_cfg = create_agx_arm_config(
            robot=self._config.robot_arm_robot,
            comm=self._config.robot_arm_comm,
            channel=self._channel,
            firmeware_version=self._firmware(),
        )
        self._arm = AgxArmFactory.create_arm(arm_cfg)

    def ensure_connected_and_enabled(self):
        if self._arm is None:
            self._build_arm()
        if not self._connected:
            self._arm.connect()
            if not self._arm.is_connected():
                raise RuntimeError(f"{self._arm_side} 臂连接失败。")
            self._connected = True

        if self._enabled:
            return

        timeout = float(self._config.robot_arm_enable_timeout)
        start_t = time.monotonic()
        while True:
            if self._arm.enable():
                self._enabled = True
                return
            if time.monotonic() - start_t > timeout:
                raise RuntimeError(f"{self._arm_side} 臂使能超时（{timeout:.1f}s）。")
            time.sleep(0.2)

    def _record_loop(self):
        while not self._stop_event.is_set():
            joint_angles = self._arm.get_leader_joint_angles()
            if joint_angles is not None and getattr(joint_angles, "msg", None) is not None:
                joints = [float(a) for a in list(joint_angles.msg)]
                if len(joints) == 7:
                    timestamp = float(getattr(joint_angles, "timestamp", time.time()))
                    with self._record_lock:
                        self._records.append({"timestamp": timestamp, "joints": joints})
            time.sleep(float(self._config.robot_arm_sample_interval_s))

    def enter_teach(self, group_name: str | None):
        if self._is_recording:
            raise RuntimeError(f"{self._arm_side} 臂已处于示教录制中。")
        self.ensure_connected_and_enabled()
        self._arm.set_leader_mode()
        self._stop_event.clear()
        with self._record_lock:
            self._records = []
        self._record_started_at = time.time()
        self._current_group_name = group_name.strip() if group_name else None
        self._record_thread = threading.Thread(
            target=self._record_loop,
            name=f"{self._arm_side}-arm-teach-recorder",
            daemon=True,
        )
        self._is_recording = True
        self._record_thread.start()

    def _resolve_group_name(self, group_name: str | None) -> str:
        if group_name and group_name.strip():
            return group_name.strip()
        if self._current_group_name:
            return self._current_group_name
        return time.strftime("%Y%m%d_%H%M%S")

    def _records_file_path(self) -> Path:
        save_dir = Path(self._config.robot_arm_teach_save_dir)
        if not save_dir.is_absolute():
            save_dir = Path(__file__).resolve().parents[2] / save_dir
        filename = self._config.robot_arm_teach_file_template.format(arm=self._arm_side)
        return save_dir / filename

    def _load_file_data(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {
                "meta": {
                    "robot": self._config.robot_arm_robot,
                    "comm": self._config.robot_arm_comm,
                    "channel": self._channel,
                    "created_at": time.time(),
                },
                "groups": {},
            }
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise RuntimeError(f"{self._arm_side} 臂示教文件格式错误。")
        data.setdefault("meta", {})
        data.setdefault("groups", {})
        if not isinstance(data["groups"], dict):
            raise RuntimeError(f"{self._arm_side} 臂示教文件 groups 字段非法。")
        return data

    def exit_teach_and_save(self, group_name: str | None) -> tuple[str, int, Path]:
        if not self._is_recording:
            raise RuntimeError(f"{self._arm_side} 臂当前不在示教状态。")
        self._stop_event.set()
        if self._record_thread is not None:
            self._record_thread.join(timeout=2.0)
        if self._record_thread is not None and self._record_thread.is_alive():
            raise RuntimeError(f"{self._arm_side} 臂录制线程未正常退出。")
        self._is_recording = False

        with self._record_lock:
            records = list(self._records)
        if not records:
            raise RuntimeError(f"{self._arm_side} 臂示教未采集到有效轨迹。")

        self._arm.set_normal_mode()
        time.sleep(0.2)
        self._enabled = False
        self.ensure_connected_and_enabled()

        resolved_group_name = self._resolve_group_name(group_name)
        path = self._records_file_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = self._load_file_data(path)
        data["meta"].update({
            "updated_at": time.time(),
            "record_count": len(records),
            "channel": self._channel,
        })
        data["groups"][resolved_group_name] = {
            "saved_at": time.time(),
            "record_started_at": self._record_started_at,
            "record_count": len(records),
            "records": records,
        }
        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        self._current_group_name = resolved_group_name
        return resolved_group_name, len(records), path

    def run_group(self, group_name: str) -> tuple[int, Path]:
        self.ensure_connected_and_enabled()
        path = self._records_file_path()
        data = self._load_file_data(path)
        groups = data.get("groups", {})
        group = groups.get(group_name)
        if not isinstance(group, dict):
            raise RuntimeError(f"{self._arm_side} 臂动作组不存在: {group_name}")
        records = group.get("records")
        if not isinstance(records, list) or not records:
            raise RuntimeError(f"{self._arm_side} 臂动作组无有效轨迹: {group_name}")

        speed_percent = int(self._config.robot_arm_replay_speed_percent)
        use_timing = bool(self._config.robot_arm_replay_use_timing)
        self._arm.set_speed_percent(speed_percent)
        self._arm.set_normal_mode()
        time.sleep(0.2)
        self._enabled = False
        self.ensure_connected_and_enabled()

        first_joints = self._clamp_joints_to_limits(records[0]["joints"])
        self._send_joint_command_with_retry(first_joints, use_js=False)
        last_timestamp = None
        min_interval = max(0.0, float(self._config.robot_arm_replay_min_interval_s))
        min_delta = max(0.0, float(self._config.robot_arm_replay_min_delta_rad))
        max_seconds = max(0.0, float(self._config.robot_arm_replay_max_seconds))
        max_frames = max(1, int(self._config.robot_arm_replay_max_frames))
        last_send_monotonic: float | None = None
        playback_start = time.monotonic()
        sent_count = 1
        last_sent_joints = list(first_joints)
        for item in records[1:]:
            if sent_count >= max_frames:
                logger.warning(
                    "{} 臂回放达到最大帧数限制 {}，提前结束。",
                    self._arm_side,
                    max_frames,
                )
                break
            if max_seconds > 0 and (time.monotonic() - playback_start) >= max_seconds:
                logger.warning(
                    "{} 臂回放达到最大时长限制 {:.1f}s，提前结束。",
                    self._arm_side,
                    max_seconds,
                )
                break
            current_timestamp = float(item["timestamp"])
            delay = 0.0
            if use_timing and last_timestamp is not None:
                delay = max(0.0, current_timestamp - last_timestamp)
            if last_send_monotonic is not None:
                elapsed = time.monotonic() - last_send_monotonic
                delay = max(delay, max(0.0, min_interval - elapsed))
            if delay > 0:
                time.sleep(delay)
            joints = self._clamp_joints_to_limits(item["joints"])
            if self._is_small_delta(last_sent_joints, joints, min_delta):
                last_timestamp = current_timestamp
                continue
            self._send_joint_command_with_retry(joints, use_js=True)
            last_send_monotonic = time.monotonic()
            last_sent_joints = joints
            last_timestamp = current_timestamp
            sent_count += 1
        return sent_count, path

    def _is_can_tx_buffer_error(self, exc: Exception) -> bool:
        text = str(exc)
        return "No buffer space available" in text or "Error Code 105" in text

    def _send_joint_command_with_retry(self, joints: list[float], use_js: bool) -> None:
        retries = max(0, int(self._config.robot_arm_replay_send_retries))
        backoff = max(0.0, float(self._config.robot_arm_replay_retry_backoff_s))
        for attempt in range(retries + 1):
            try:
                if use_js:
                    self._arm.move_js(joints)
                else:
                    self._arm.move_j(joints)
                return
            except RuntimeError as exc:
                if not self._is_can_tx_buffer_error(exc) or attempt >= retries:
                    raise
                sleep_s = backoff * (attempt + 1)
                logger.warning(
                    "{} 臂 CAN 发送缓存不足，{} 指令第 {}/{} 次重试，退避 {:.3f}s",
                    self._arm_side,
                    "move_js" if use_js else "move_j",
                    attempt + 1,
                    retries,
                    sleep_s,
                )
                if sleep_s > 0:
                    time.sleep(sleep_s)

    def _clamp_joints_to_limits(self, joints: list[float]) -> list[float]:
        limits = getattr(self._config, "robot_arm_joint_limits", None) or []
        if (
            not isinstance(limits, list)
            or len(limits) != len(joints)
            or any(not isinstance(bound, list) or len(bound) != 2 for bound in limits)
        ):
            return [float(v) for v in joints]
        clamped: list[float] = []
        for idx, value in enumerate(joints):
            lower = float(limits[idx][0])
            upper = float(limits[idx][1])
            if lower > upper:
                lower, upper = upper, lower
            if value < lower or value > upper:
                logger.debug(
                    "{} 臂关节 {} 越限({:.6f})，夹紧到 [{:.6f}, {:.6f}]",
                    self._arm_side,
                    idx,
                    float(value),
                    lower,
                    upper,
                )
            clamped.append(min(max(float(value), lower), upper))
        return clamped

    @staticmethod
    def _is_small_delta(prev_joints: list[float], curr_joints: list[float], min_delta: float) -> bool:
        if min_delta <= 0:
            return False
        if len(prev_joints) != len(curr_joints):
            return False
        return max(abs(a - b) for a, b in zip(prev_joints, curr_joints)) < min_delta

    @property
    def is_recording(self) -> bool:
        return self._is_recording


class RobotArmTeachManager:
    """双臂示教会话管理器。"""

    def __init__(self, config: RobotConfig):
        self._config = config
        self._runtimes: dict[str, ArmTeachRuntime] = {}
        self._lock = threading.Lock()
        self._active_teach_arm: str | None = None

    def _runtime(self, side: str) -> ArmTeachRuntime:
        side = _normalize_side(side)
        if side not in {"left", "right"}:
            raise RuntimeError("请指定左臂或右臂。")
        runtime = self._runtimes.get(side)
        if runtime is None:
            runtime = ArmTeachRuntime(self._config, side)
            self._runtimes[side] = runtime
        return runtime

    def enter_teach(self, side: str, group_name: str | None) -> str:
        with self._lock:
            runtime = self._runtime(side)
            runtime.enter_teach(group_name=group_name)
            self._active_teach_arm = side
        return f"{'左臂' if side == 'left' else '右臂'}已进入示教模式，请拖动机械臂完成示教。"

    def exit_teach(self, side: str | None, group_name: str | None) -> str:
        with self._lock:
            resolved_side = _normalize_side(side) or self._active_teach_arm
            if resolved_side is None:
                raise RuntimeError("未识别到要退出示教的机械臂，请说明左臂或右臂。")
            runtime = self._runtime(resolved_side)
            resolved_group, count, path = runtime.exit_teach_and_save(group_name=group_name)
            self._active_teach_arm = None
        side_text = "左臂" if resolved_side == "left" else "右臂"
        return (
            f"{side_text}已退出示教，动作组“{resolved_group}”已保存，"
        )

    def run_group(self, side: str, group_name: str) -> str:
        with self._lock:
            runtime = self._runtime(side)
            count, path = runtime.run_group(group_name)
        side_text = "左臂" if side == "left" else "右臂"
        return (
            f"{side_text}已执行动作组“{group_name}”"
        )

    @property
    def active_teach_arm(self) -> str | None:
        return self._active_teach_arm


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
    side = None
    if any(k in command for k in ["左臂", "左手", "左边", "左机械臂"]):
        side = "left"
    elif any(k in command for k in ["右臂", "右手", "右边", "右机械臂"]):
        side = "right"

    operation = None
    if any(k in command for k in ["进入拖动", "开始拖动", "开启拖动"]):
        operation = "enter_teach"
    elif any(k in command for k in ["退出拖动", "结束拖动", "停止拖动"]):
        operation = "exit_teach"
    elif (
        ("执行" in command or "回放" in command)
        and any(k in command for k in ["拖动", "动作组", "轨迹"])
    ):
        operation = "run_group"

    return ParsedRobotArmCommand(
        arm_side=side,
        operation=operation,
        group_name=_extract_group_name(command),
        raw_command=command,
    )


_MANAGER_CACHE: dict[int, RobotArmTeachManager] = {}


def _get_manager(config: RobotConfig) -> RobotArmTeachManager:
    manager = _MANAGER_CACHE.get(id(config))
    if manager is None:
        manager = RobotArmTeachManager(config)
        _MANAGER_CACHE[id(config)] = manager
    return manager


def execute_robot_arm(
    config: RobotConfig,
    action: str | None = None,
    arm_side: str | None = None,
    operation: str | None = None,
    group_name: str | None = None,
) -> str:
    """机械臂示教控制入口。"""
    if not config.robot_arm_enabled:
        return "机械臂功能未启用。"

    mapped = resolve_keyword_robot_arm_action(config=config, command=action or "")
    if mapped is not None:
        manager = _get_manager(config)
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
    manager = _get_manager(config)

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
        "未识别机械臂指令。请使用“左/右臂进入拖动”、“退出拖动并保存为: 动作名”"
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

    def update(self):
        if self.blackboard.intent != "robot_arm":
            return Status.FAILURE
        command = self.blackboard.user_command
        self.logger.info(f"执行: 机械臂控制 ({command})")
        try:
            self.blackboard.response_text = execute_robot_arm(
                config=self._config,
                action=command,
            )
        except Exception as exc:
            self.blackboard.response_text = f"机械臂操作失败：{exc}"
        return Status.SUCCESS
