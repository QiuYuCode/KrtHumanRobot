"""Lifecycle and process control for Web-managed gripper nodes."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from typing import Any, Callable

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


SIDES = ("left", "right")
STATIC_KEYS = ("adapter_type", "adapter_index", "device_id")
RUNTIME_KEYS = ("listen_enabled", "realtime_response_enabled")


class GripperSystemController:
    """Discover, launch, and transition left/right lifecycle nodes."""

    def __init__(
        self,
        node,
        database,
        *,
        hand_clients: dict[str, Any],
        future_result: Callable[[Any, float], Any],
        popen: Callable[..., subprocess.Popen] = subprocess.Popen,
        sleep: Callable[[float], None] = time.sleep,
        startup_timeout: float = 8.0,
    ) -> None:
        self.database = database
        self.hand_clients = hand_clients
        self._future_result = future_result
        self._popen = popen
        self._sleep = sleep
        self.startup_timeout = startup_timeout
        self.lock = threading.Lock()
        self.processes = []
        self.last_errors = {side: "" for side in SIDES}
        self.state_clients = {}
        self.change_clients = {}
        self.get_parameter_clients = {}
        self.set_parameter_clients = {}
        for side in SIDES:
            base = f"/{side}/hand_control_server"
            self.state_clients[side] = node.create_client(GetState, f"{base}/get_state")
            self.change_clients[side] = node.create_client(
                ChangeState, f"{base}/change_state"
            )
            self.get_parameter_clients[side] = node.create_client(
                GetParameters, f"{base}/get_parameters"
            )
            self.set_parameter_clients[side] = node.create_client(
                SetParameters, f"{base}/set_parameters"
            )

    @staticmethod
    def _selected(target: str) -> list[str]:
        if target == "both":
            return list(SIDES)
        if target not in SIDES:
            raise ValueError("target 必须是 left、right 或 both")
        return [target]

    def _state(self, side: str) -> str:
        client = self.state_clients[side]
        if not client.wait_for_service(timeout_sec=0.05):
            return "missing"
        response = self._future_result(client.call_async(GetState.Request()), 0.8)
        return str(response.current_state.label).lower()

    @staticmethod
    def _parameter_value(value: Any) -> ParameterValue:
        if isinstance(value, bool):
            return ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=value
            )
        if isinstance(value, int):
            return ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=value
            )
        return ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=str(value)
        )

    def _set_parameters(self, side: str, values: dict[str, Any]) -> None:
        client = self.set_parameter_clients[side]
        if not client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(f"{side} 参数服务不可用")
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=self._parameter_value(value))
            for name, value in values.items()
        ]
        response = self._future_result(client.call_async(request), 1.0)
        failures = [item.reason for item in response.results if not item.successful]
        if failures:
            raise RuntimeError("; ".join(failures))

    def _get_parameters(self, side: str) -> dict[str, Any]:
        client = self.get_parameter_clients[side]
        if not client.wait_for_service(timeout_sec=0.05):
            return {}
        names = [*STATIC_KEYS, *RUNTIME_KEYS]
        request = GetParameters.Request()
        request.names = names
        response = self._future_result(client.call_async(request), 0.8)
        values = {}
        for name, value in zip(names, response.values):
            if name == "adapter_type":
                values[name] = value.string_value
            elif name in STATIC_KEYS:
                values[name] = int(value.integer_value)
            else:
                values[name] = bool(value.bool_value)
        return values

    def _transition(self, side: str, transition_id: int) -> None:
        client = self.change_clients[side]
        if not client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(f"{side} 生命周期服务不可用")
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self._future_result(client.call_async(request), 3.5)
        if not response.success:
            raise RuntimeError(f"{side} 生命周期转换失败: {transition_id}")

    def _launch_missing(self, sides: list[str]) -> None:
        command = [
            "ros2", "launch", "hands_control", "hand_control_launch.py",
            f"enable_left:={'true' if 'left' in sides else 'false'}",
            f"enable_right:={'true' if 'right' in sides else 'false'}",
            "autostart:=false",
        ]
        process = self._popen(command, start_new_session=True)
        self.processes.append(process)
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if all(self._state(side) != "missing" for side in sides):
                return
            if process.poll() is not None:
                break
            self._sleep(0.1)
        self._terminate(process)
        raise RuntimeError(f"夹爪节点启动超时: {', '.join(sides)}")

    @staticmethod
    def _terminate(process) -> None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except (AttributeError, OSError, ProcessLookupError):
            try:
                process.terminate()
            except (AttributeError, OSError):
                pass

    def _start_side(self, side: str) -> dict[str, Any]:
        state = self._state(side)
        if state == "active":
            return {"success": True, "state": state, "message": "已经运行"}
        if state == "unconfigured":
            settings = self.database.get_gripper_settings(side)
            self._set_parameters(side, {
                key: settings[key] for key in (*STATIC_KEYS, *RUNTIME_KEYS)
            })
            self._transition(side, Transition.TRANSITION_CONFIGURE)
            state = self._state(side)
        if state == "inactive":
            self._transition(side, Transition.TRANSITION_ACTIVATE)
            state = self._state(side)
        if state != "active":
            raise RuntimeError(f"{side} 无法从 {state} 进入 active")
        return {"success": True, "state": state, "message": "已开启"}

    def _stop_side(self, side: str) -> dict[str, Any]:
        state = self._state(side)
        if state in {"missing", "unconfigured"}:
            return {"success": True, "state": state, "message": "已经关闭"}
        if state == "active":
            self._transition(side, Transition.TRANSITION_DEACTIVATE)
            state = self._state(side)
        if state == "inactive":
            self._transition(side, Transition.TRANSITION_CLEANUP)
            state = self._state(side)
        if state != "unconfigured":
            raise RuntimeError(f"{side} 无法从 {state} 进入 unconfigured")
        return {"success": True, "state": state, "message": "已关闭"}

    def control(self, target: str, enabled: bool) -> dict[str, Any]:
        sides = self._selected(target)
        with self.lock:
            if enabled:
                missing = [side for side in sides if self._state(side) == "missing"]
                if missing:
                    self._launch_missing(missing)
            results = {}
            for side in sides:
                try:
                    result = self._start_side(side) if enabled else self._stop_side(side)
                    self.last_errors[side] = ""
                except Exception as exc:
                    self.last_errors[side] = str(exc)
                    result = {
                        "success": False, "state": "unknown",
                        "message": str(exc),
                    }
                results[side] = result
            return {
                "success": all(item["success"] for item in results.values()),
                "hands": results,
            }

    def status(self) -> dict[str, Any]:
        settings = {
            item["side"]: item for item in self.database.list_gripper_settings()
        }
        hands = {}
        for side in SIDES:
            try:
                state = self._state(side)
                current_parameters = self._get_parameters(side) \
                    if state != "missing" else {}
                error = self.last_errors[side]
            except Exception as exc:
                state = "unknown"
                current_parameters = {}
                error = str(exc)
            hands[side] = {
                "present": state != "missing",
                "lifecycle_state": state,
                "action_ready": state == "active" and self.hand_clients[
                    side
                ].wait_for_server(timeout_sec=0.0),
                "settings": settings.get(side, {}),
                "current_parameters": current_parameters,
                "error": error,
            }
        return {"hands": hands}

    def update_settings(self, side: str, changes: dict[str, Any]) -> dict[str, Any]:
        state = self._state(side)
        if state not in {"missing", "unconfigured"}:
            raise RuntimeError("请先关闭夹爪再修改硬件参数")
        if set(changes) - set(STATIC_KEYS):
            raise ValueError("这里只允许修改硬件参数")
        self.database.update_gripper_settings(side, changes)
        return self.database.get_gripper_settings(side)

    def update_runtime(self, side: str, changes: dict[str, Any]) -> dict[str, Any]:
        if side not in SIDES or set(changes) - set(RUNTIME_KEYS):
            raise ValueError("运行开关参数无效")
        self.database.update_gripper_settings(side, changes)
        if self._state(side) == "active":
            self._set_parameters(side, changes)
        return self.database.get_gripper_settings(side)
