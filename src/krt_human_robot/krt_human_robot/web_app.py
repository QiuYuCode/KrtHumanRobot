"""Flask application for the KRT robot operations console."""

from __future__ import annotations

import os
import secrets
import threading
import time
import uuid
import wave
from functools import wraps
from pathlib import Path
from typing import Any, Callable

from flask import Flask, jsonify, render_template, request, session
from hands_control_interfaces.action import HandControl
from hands_control_interfaces.srv import (
    GetApproachingValue,
    GetFingerValue,
    GetNormalPressure,
    GetTangentPressure,
)
from krt_task.robot_db import RobotDatabase, validate_gripper_targets
from krt_task_interfaces.action import RunRoutine
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from werkzeug.exceptions import RequestEntityTooLarge
from werkzeug.utils import secure_filename

from krt_human_robot.adapters.navigation import RangerNavAdapter
from krt_human_robot.config import load_config
from krt_human_robot.web_auth import AuthDatabase


class RosBridge:
    """One rclpy node owned by the single Gunicorn worker."""

    def __init__(self, action_name: str = "/krt_task/run_routine", robot_config=None) -> None:
        import rclpy

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("krt_web_bridge")
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.client = ActionClient(self.node, RunRoutine, action_name)
        self.hand_clients = {
            side: ActionClient(self.node, HandControl, f"/{side}/hand_control")
            for side in ("left", "right")
        }
        self.hand_adapter_indices = {
            "left": int(getattr(robot_config, "left_gripper", {}).get("adapter_index", 0)),
            "right": int(getattr(robot_config, "right_gripper", {}).get("adapter_index", 1)),
        }
        self.telemetry_clients = {
            "left": {
                "position": self.node.create_client(GetFingerValue, "/left/get_joint_degree"),
            },
            "right": {
                "position": self.node.create_client(GetFingerValue, "/right/get_joint_degree"),
                "normal_pressure": self.node.create_client(
                    GetNormalPressure, "/right/get_normal_pressure"
                ),
                "tangent_pressure": self.node.create_client(
                    GetTangentPressure, "/right/get_tangent_pressure"
                ),
                "approaching": self.node.create_client(
                    GetApproachingValue, "/right/get_approaching_value"
                ),
            },
        }
        self.monitor_get_client = self.node.create_client(
            GetParameters, "/right/hand_control_server/get_parameters"
        )
        self.monitor_set_client = self.node.create_client(
            SetParameters, "/right/hand_control_server/set_parameters"
        )
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.monitor_lock = threading.Lock()
        self.monitor_active = False
        self.monitor_previous = None
        self.monitor_deadline = 0.0
        self.monitor_timer = self.node.create_timer(1.0, self._monitor_watchdog)
        self.goal_handle = None
        self.gripper_goal_handles = {}
        self._gripper_remaining: set[str] = set()
        self.cancel_requested = False
        self.state = {"mode": "idle", "status": "idle", "current_step": "", "message": ""}

    def run(self, routine_name: str) -> None:
        with self.lock:
            if self.state["status"] == "running":
                raise RuntimeError("已有任务正在执行")
            if not self.client.wait_for_server(timeout_sec=2.0):
                raise RuntimeError("routine action 不可用")
            self.state = {"mode": "routine", "status": "running", "current_step": "", "message": ""}
            self.cancel_requested = False
        goal = RunRoutine.Goal()
        goal.routine_name = routine_name
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def run_gripper(self, targets: list[dict[str, Any]]) -> None:
        for target in targets:
            side = str(target["side"])
            if not self.hand_clients[side].wait_for_server(timeout_sec=2.0):
                raise RuntimeError(f"夹爪 action 不可用: /{side}/hand_control")
        hands = {
            str(target["side"]): {
                "status": "pending", "progress": 0.0,
                "current_positions": [], "message": "",
            }
            for target in targets
        }
        with self.lock:
            if self.state["status"] == "running":
                raise RuntimeError("已有任务正在执行")
            self.state = {
                "mode": "gripper", "status": "running", "current_step": "",
                "message": "", "hands": hands,
            }
            self.cancel_requested = False
            self.gripper_goal_handles = {}
            self._gripper_remaining = set(hands)
        for target in targets:
            side = str(target["side"])
            goal = HandControl.Goal()
            goal.adapter_index = self.hand_adapter_indices[side]
            goal.finger_id = int(target["finger_id"])
            goal.position = int(target["position"])
            goal.speed = int(target["speed"])
            goal.force = int(target["force"])
            goal.wait_time = int(target["wait_time"])
            future = self.hand_clients[side].send_goal_async(
                goal,
                feedback_callback=lambda message, hand=side: self._gripper_feedback(
                    hand, message
                ),
            )
            future.add_done_callback(
                lambda response, hand=side: self._gripper_goal_response(hand, response)
            )

    def _gripper_feedback(self, side: str, message) -> None:
        feedback = message.feedback
        with self.lock:
            hand = self.state.get("hands", {}).get(side)
            if hand is None:
                return
            hand["status"] = "running"
            hand["progress"] = float(feedback.progress)
            hand["current_positions"] = list(feedback.current_positions)
            self.state["current_step"] = f"gripper:{side}"

    def _gripper_goal_response(self, side: str, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self._finish_gripper_hand(side, False, "目标被拒绝", [])
                return
            with self.lock:
                self.gripper_goal_handles[side] = goal_handle
                cancel_requested = self.cancel_requested
            if cancel_requested:
                goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(
                lambda result, hand=side: self._gripper_result(hand, result)
            )
        except Exception as exc:
            self._finish_gripper_hand(side, False, str(exc), [])

    def _gripper_result(self, side: str, future) -> None:
        try:
            result = future.result().result
            self._finish_gripper_hand(
                side, bool(result.success), str(result.message), list(result.final_positions)
            )
        except Exception as exc:
            self._finish_gripper_hand(side, False, str(exc), [])

    def _finish_gripper_hand(
        self, side: str, success: bool, message: str, positions: list[int]
    ) -> None:
        with self.lock:
            hand = self.state["hands"][side]
            hand.update(
                status="succeeded" if success else "failed",
                progress=1.0 if success else hand["progress"],
                current_positions=positions or hand["current_positions"],
                message=message,
            )
            self._gripper_remaining.discard(side)
            if not success:
                self.cancel_requested = True
            handles = list(self.gripper_goal_handles.values()) if not success else []
            if not self._gripper_remaining:
                all_succeeded = all(
                    item["status"] == "succeeded" for item in self.state["hands"].values()
                )
                self.state["status"] = "succeeded" if all_succeeded else "failed"
                self.state["message"] = "夹爪动作完成" if all_succeeded else message
                self.cancel_requested = False
        for goal_handle in handles:
            goal_handle.cancel_goal_async()

    def _feedback(self, message) -> None:
        with self.lock:
            self.state["current_step"] = message.feedback.current_step

    def _goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                raise RuntimeError("routine 目标被拒绝")
            with self.lock:
                self.goal_handle = goal_handle
                cancel_requested = self.cancel_requested
            if cancel_requested:
                goal_handle.cancel_goal_async()
            goal_handle.get_result_async().add_done_callback(self._result)
        except Exception as exc:
            self._finish(False, str(exc))

    def _result(self, future) -> None:
        try:
            result = future.result().result
            self._finish(bool(result.success), result.message)
        except Exception as exc:
            self._finish(False, str(exc))

    def _finish(self, success: bool, message: str) -> None:
        with self.lock:
            self.state["status"] = "succeeded" if success else "failed"
            self.state["message"] = message
            self.goal_handle = None
            self.cancel_requested = False

    def cancel(self) -> None:
        with self.lock:
            goal_handle = self.goal_handle
            if self.state["status"] != "running":
                raise RuntimeError("没有可取消的 routine")
            self.cancel_requested = True
            gripper_handles = list(self.gripper_goal_handles.values())
            mode = self.state["mode"]
        if mode == "gripper":
            for handle in gripper_handles:
                handle.cancel_goal_async()
        elif goal_handle is not None:
            goal_handle.cancel_goal_async()

    def status(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.state)

    @staticmethod
    def _future_result(future, timeout: float):
        if not hasattr(future, "done"):
            return future.result()
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            raise TimeoutError("ROS 服务调用超时")
        return future.result()

    def telemetry(self) -> dict[str, Any]:
        if not self.telemetry_lock.acquire(blocking=False):
            raise RuntimeError("上一轮夹爪遥测尚未完成")
        try:
            with self.monitor_lock:
                if not self.monitor_active:
                    raise RuntimeError("夹爪监测尚未开启")
                self.monitor_deadline = time.monotonic() + 3.0
            snapshot = {
                "timestamp": time.time(),
                "hands": {
                    "left": {"pressure_supported": False, "fingers": []},
                    "right": {"pressure_supported": True, "fingers": []},
                },
            }
            pending = []
            for side, clients in self.telemetry_clients.items():
                fingers = snapshot["hands"][side]["fingers"]
                for finger_id in (1, 2, 3):
                    finger = {"id": finger_id}
                    fingers.append(finger)
                    for metric, client in clients.items():
                        if not client.wait_for_service(timeout_sec=0.05):
                            finger[metric] = {
                                "available": False, "value": None,
                                "error": f"服务不可用: {client.srv_name}",
                            }
                            continue
                        request_message = client.srv_type.Request()
                        request_message.finger_id = finger_id
                        pending.append((finger, metric, client.call_async(request_message)))
            deadline = time.monotonic() + 0.45
            for finger, metric, future in pending:
                try:
                    response = self._future_result(
                        future, max(0.0, deadline - time.monotonic())
                    )
                    available = bool(
                        response.success and getattr(response, "available", True)
                    )
                    finger[metric] = {
                        "available": available,
                        "value": float(response.value) if available else None,
                        "error": "" if available else str(response.error_message),
                    }
                except Exception as exc:
                    finger[metric] = {
                        "available": False, "value": None, "error": str(exc),
                    }
            return snapshot
        finally:
            self.telemetry_lock.release()

    def set_monitor(self, enabled: bool) -> dict[str, bool]:
        if enabled:
            with self.monitor_lock:
                if self.monitor_active:
                    self.monitor_deadline = time.monotonic() + 3.0
                    return {"enabled": True}
            previous = self._get_monitor_parameters()
            self._set_monitor_parameters({
                "listen_enabled": True, "realtime_response_enabled": True,
            })
            with self.monitor_lock:
                self.monitor_previous = previous
                self.monitor_active = True
                self.monitor_deadline = time.monotonic() + 3.0
            return {"enabled": True}

        with self.monitor_lock:
            previous = self.monitor_previous
            was_active = self.monitor_active
            self.monitor_active = False
            self.monitor_previous = None
            self.monitor_deadline = 0.0
        if was_active and previous is not None:
            self._set_monitor_parameters(previous)
        return {"enabled": False}

    def _monitor_watchdog(self) -> None:
        with self.monitor_lock:
            expired = self.monitor_active and time.monotonic() > self.monitor_deadline
        if expired:
            try:
                self.set_monitor(False)
            except Exception as exc:
                self.node.get_logger().warning(f"恢复夹爪监测参数失败: {exc}")

    def _get_monitor_parameters(self) -> dict[str, bool]:
        if not self.monitor_get_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("右手参数读取服务不可用")
        request_message = GetParameters.Request()
        request_message.names = ["listen_enabled", "realtime_response_enabled"]
        response = self._future_result(self.monitor_get_client.call_async(request_message), 1.0)
        return {
            name: bool(value.bool_value)
            for name, value in zip(request_message.names, response.values)
        }

    def _set_monitor_parameters(self, values: dict[str, bool]) -> None:
        if not self.monitor_set_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("右手参数设置服务不可用")
        request_message = SetParameters.Request()
        request_message.parameters = [
            Parameter(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)
                ),
            )
            for name, value in values.items()
        ]
        response = self._future_result(self.monitor_set_client.call_async(request_message), 1.0)
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            raise RuntimeError("; ".join(failures))


class WebRuntime:
    def __init__(self, adapter: RangerNavAdapter, bridge: RosBridge | None) -> None:
        self.adapter = adapter
        self.bridge = bridge
        self.lock = threading.Lock()
        self.mode = "idle"

    def run_routine(self, name: str) -> None:
        if self.bridge is None:
            raise RuntimeError("ROS bridge 未启用")
        with self.lock:
            if self.active():
                raise RuntimeError("已有任务正在执行")
            self.bridge.run(name)
            self.mode = "routine"

    def run_waypoint(self, name: str) -> None:
        with self.lock:
            if self.active():
                raise RuntimeError("已有任务正在执行")
            result = self.adapter.start_cruise([name])
            if not result.success:
                raise RuntimeError(result.message)
            self.mode = "waypoint"

    def run_gripper(self, targets: list[dict[str, Any]]) -> None:
        if self.bridge is None:
            raise RuntimeError("ROS bridge 未启用")
        with self.lock:
            if self.active():
                raise RuntimeError("已有任务正在执行")
            self.bridge.run_gripper(targets)
            self.mode = "gripper"

    def active(self) -> bool:
        if self.mode == "routine" and self.bridge:
            return self.bridge.status()["status"] == "running"
        if self.mode == "gripper" and self.bridge:
            return self.bridge.status()["status"] == "running"
        if self.mode == "waypoint":
            return self.adapter._running(self.adapter._cruise_process)
        return False

    def status(self) -> dict[str, Any]:
        if self.mode == "routine" and self.bridge:
            return self.bridge.status()
        if self.mode == "gripper" and self.bridge:
            return self.bridge.status()
        if self.mode == "waypoint":
            process = self.adapter._cruise_process
            running = self.adapter._running(process)
            code = None if running or process is None else process.poll()
            return {
                "mode": "waypoint",
                "status": "running" if running else ("succeeded" if code == 0 else "failed"),
                "current_step": "",
                "message": "" if code in {None, 0} else f"巡航退出码: {code}",
            }
        return {"mode": "idle", "status": "idle", "current_step": "", "message": ""}

    def cancel(self) -> None:
        if self.mode == "routine" and self.bridge:
            self.bridge.cancel()
        elif self.mode == "gripper" and self.bridge:
            self.bridge.cancel()
        elif self.mode == "waypoint":
            result = self.adapter.stop_cruise()
            if not result.success:
                raise RuntimeError(result.message)
        else:
            raise RuntimeError("没有正在执行的任务")


def create_app(test_config: dict[str, Any] | None = None) -> Flask:
    source_templates = Path(__file__).resolve().parent.parent / "templates"
    if not source_templates.is_dir():
        from ament_index_python.packages import get_package_share_directory
        source_templates = Path(get_package_share_directory("krt_human_robot")) / "templates"
    app = Flask(__name__, template_folder=str(source_templates))
    app.config.from_mapping(
        SECRET_KEY=None,
        ROBOT_DB=os.environ.get("KRT_ROBOT_DB", "~/maps/krt_robot.db"),
        WEB_DB=os.environ.get("KRT_WEB_DB", "~/.local/share/krt_human_robot/web.db"),
        MEDIA_DIR=os.environ.get("KRT_MEDIA_DIR", "~/music"),
        MAX_CONTENT_LENGTH=100 * 1000 * 1000,
        SESSION_COOKIE_SECURE=True,
        SESSION_COOKIE_HTTPONLY=True,
        SESSION_COOKIE_SAMESITE="Lax",
        ROS_ENABLED=os.environ.get("KRT_WEB_ROS_ENABLED", "1")
        not in {"0", "false", "False"},
    )
    if test_config:
        app.config.update(test_config)
    if not app.config["SECRET_KEY"]:
        app.config["SECRET_KEY"] = load_secret(os.environ.get(
            "KRT_WEB_SECRET_FILE", "~/.config/krt_human_robot/web_secret"
        ))

    database = RobotDatabase(app.config["ROBOT_DB"])
    auth = AuthDatabase(app.config["WEB_DB"])
    media_dir = Path(app.config["MEDIA_DIR"]).expanduser().resolve()
    media_dir.mkdir(parents=True, exist_ok=True)
    robot_config = load_config(os.environ.get("KRT_HUMAN_ROBOT_CONFIG"))
    adapter = RangerNavAdapter(robot_config)
    bridge = RosBridge(robot_config=robot_config) if app.config["ROS_ENABLED"] else None
    runtime = WebRuntime(adapter, bridge)
    app.extensions.update(robot_db=database, auth_db=auth, runtime=runtime)
    # ponytail: one-worker in-memory limiter; move to shared storage for multi-worker use.
    failures: dict[str, list[float]] = {}

    @app.errorhandler(Exception)
    def api_error(exc):
        code = exc.code if hasattr(exc, "code") else 400
        if request.path.startswith("/api/") and request.method not in {"GET", "HEAD"}:
            user = current_user(auth)
            auth.audit(
                user["username"] if user else "-", request.remote_addr or "-",
                f"{request.method} {request.path}", request.path, False, str(exc),
            )
        app.logger.exception("request failed")
        return jsonify(error=str(exc)), code

    @app.errorhandler(RequestEntityTooLarge)
    def too_large(_exc):
        return jsonify(error="文件超过 100MB"), 413

    @app.get("/")
    def index():
        return render_template("console.html")

    @app.get("/api/session")
    def session_info():
        user = current_user(auth)
        token = session.setdefault("csrf_token", secrets.token_urlsafe(24))
        return jsonify(user=public_user(user), csrf_token=token)

    @app.post("/api/login")
    def login():
        payload = request.get_json(silent=True) or {}
        username = str(payload.get("username", "")).strip()
        key = f"{request.remote_addr}:{username}"
        recent = [stamp for stamp in failures.get(key, []) if time.time() - stamp < 300]
        if len(recent) >= 5:
            return jsonify(error="登录失败次数过多，请稍后再试"), 429
        user = auth.authenticate(username, str(payload.get("password", "")))
        if user is None:
            recent.append(time.time())
            failures[key] = recent
            auth.audit(username or "-", request.remote_addr or "-", "login", username,
                       False, "用户名或密码错误")
            return jsonify(error="用户名或密码错误"), 401
        session.clear()
        session["user_id"] = user["id"]
        session["csrf_token"] = secrets.token_urlsafe(24)
        auth.audit(username, request.remote_addr or "-", "login", username, True)
        return jsonify(user=public_user(user), csrf_token=session["csrf_token"])

    @app.post("/api/logout")
    @protected(auth)
    def logout():
        session.clear()
        return jsonify(success=True)

    @app.get("/api/waypoints")
    @protected(auth, csrf=False)
    def waypoints():
        return jsonify([row.__dict__ for row in database.list_waypoints()])

    @app.post("/api/waypoints")
    @protected(auth)
    def mark_waypoint():
        payload = request.get_json() or {}
        result = adapter.mark_waypoint(str(payload.get("name", "")).strip() or None,
                                       routine=str(payload.get("routine", "")))
        return audited(auth, "mark_waypoint", str(payload.get("name", "")), result.success,
                       result.message)

    @app.patch("/api/waypoints/<name>")
    @protected(auth)
    def bind_waypoint(name: str):
        routine = str((request.get_json() or {}).get("routine", ""))
        database.bind_waypoint(name, routine)
        return audited(auth, "bind_waypoint", name, True, routine)

    @app.delete("/api/waypoints/<name>")
    @protected(auth)
    def delete_waypoint(name: str):
        database.delete_waypoint(name)
        return audited(auth, "delete_waypoint", name, True)

    @app.get("/api/routines")
    @protected(auth, csrf=False)
    def routines():
        return jsonify(database.list_routines())

    @app.get("/api/action-groups")
    @protected(auth, csrf=False)
    def action_groups():
        return jsonify(database.list_action_groups())

    @app.get("/api/gripper-actions")
    @protected(auth, csrf=False)
    def gripper_actions():
        return jsonify(database.list_gripper_actions())

    @app.put("/api/gripper-actions/<name>")
    @protected(auth)
    def save_gripper_action(name: str):
        targets = (request.get_json() or {}).get("targets", [])
        database.save_gripper_action(name, targets)
        return audited(auth, "save_gripper_action", name, True)

    @app.patch("/api/gripper-actions/<name>")
    @protected(auth)
    def rename_gripper_action(name: str):
        new_name = str((request.get_json() or {}).get("name", ""))
        database.rename_gripper_action(name, new_name)
        return audited(auth, "rename_gripper_action", name, True, new_name)

    @app.delete("/api/gripper-actions/<name>")
    @protected(auth)
    def delete_gripper_action(name: str):
        database.delete_gripper_action(name)
        return audited(auth, "delete_gripper_action", name, True)

    @app.get("/api/gripper-defaults")
    @protected(auth, csrf=False)
    def gripper_defaults():
        return jsonify(
            open_position=int(robot_config.gripper_open_value),
            close_position=int(robot_config.gripper_close_value),
            open_speed=int(robot_config.gripper_speed_open),
            close_speed=int(robot_config.gripper_speed_close),
            default_speed=int(robot_config.gripper_default_speed),
            default_force=int(robot_config.gripper_control_mode),
            default_wait_time=int(robot_config.gripper_exec_delay_ms),
            ranges={
                "finger_id": [0, 3], "position": [0, 1000], "speed": [0, 1000],
                "force": [0, 255], "wait_time": [0, 255],
            },
        )

    @app.post("/api/gripper/run")
    @protected(auth)
    def run_gripper():
        targets = validate_gripper_targets((request.get_json() or {}).get("targets", []))
        runtime.run_gripper(targets)
        sides = ",".join(str(target["side"]) for target in targets)
        return audited(auth, "run_gripper", sides, True)

    @app.post("/api/gripper/monitor")
    @protected(auth)
    def monitor_gripper():
        if runtime.bridge is None:
            raise RuntimeError("ROS bridge 未启用")
        enabled = bool((request.get_json() or {}).get("enabled", False))
        result = runtime.bridge.set_monitor(enabled)
        audited(auth, "monitor_gripper", "right", True, str(enabled))
        return jsonify(result)

    @app.get("/api/gripper/telemetry")
    @protected(auth, csrf=False)
    def gripper_telemetry():
        if runtime.bridge is None:
            raise RuntimeError("ROS bridge 未启用")
        return jsonify(runtime.bridge.telemetry())

    @app.patch("/api/action-groups/<name>")
    @protected(auth)
    def rename_action_group(name: str):
        new_name = str((request.get_json() or {}).get("name", ""))
        database.rename_action_group(name, new_name)
        return audited(auth, "rename_action_group", name, True, new_name)

    @app.put("/api/routines/<name>")
    @protected(auth)
    def save_routine(name: str):
        spec = request.get_json() or {}
        validate_media_refs(spec, database)
        database.save_routine(name, spec)
        return audited(auth, "save_routine", name, True)

    @app.delete("/api/routines/<name>")
    @protected(auth)
    def delete_routine(name: str):
        database.delete_routine(name)
        return audited(auth, "delete_routine", name, True)

    @app.get("/api/media")
    @protected(auth, csrf=False)
    def media():
        return jsonify(database.list_media())

    @app.post("/api/media")
    @protected(auth)
    def upload_media():
        upload = request.files.get("file")
        if upload is None or not upload.filename:
            raise ValueError("缺少 WAV 文件")
        display_name = secure_filename(upload.filename) or "audio.wav"
        media_key = uuid.uuid4().hex
        filename = f"{media_key}.wav"
        path = media_dir / filename
        upload.save(path)
        try:
            duration, rate, channels = inspect_wav(path)
            database.add_media({
                "media_key": media_key, "display_name": display_name,
                "filename": filename, "size_bytes": path.stat().st_size,
                "duration_sec": duration, "sample_rate": rate, "channels": channels,
            })
        except Exception:
            path.unlink(missing_ok=True)
            raise
        return audited(auth, "upload_media", display_name, True, data={"media_key": media_key})

    @app.delete("/api/media/<media_key>")
    @protected(auth)
    def delete_media(media_key: str):
        record = database.delete_media(media_key)
        (media_dir / record["filename"]).unlink(missing_ok=True)
        return audited(auth, "delete_media", record["display_name"], True)

    @app.post("/api/routines/<name>/run")
    @protected(auth)
    def run_routine(name: str):
        database.get_routine(name)
        runtime.run_routine(name)
        return audited(auth, "run_routine", name, True)

    @app.post("/api/waypoints/<name>/run")
    @protected(auth)
    def run_waypoint(name: str):
        if name not in {row.name for row in database.list_waypoints()}:
            raise KeyError("点位不存在")
        runtime.run_waypoint(name)
        return audited(auth, "run_waypoint", name, True)

    @app.get("/api/execution")
    @protected(auth, csrf=False)
    def execution():
        return jsonify(runtime.status())

    @app.post("/api/execution/cancel")
    @protected(auth)
    def cancel_execution():
        runtime.cancel()
        return audited(auth, "cancel_execution", runtime.mode, True)

    @app.get("/api/users")
    @protected(auth, role="admin", csrf=False)
    def users():
        return jsonify(auth.list_users())

    @app.post("/api/users")
    @protected(auth, role="admin")
    def add_user():
        payload = request.get_json() or {}
        auth.create_user(str(payload.get("username", "")), str(payload.get("password", "")),
                         str(payload.get("role", "operator")))
        return audited(auth, "create_user", str(payload.get("username", "")), True)

    @app.patch("/api/users/<int:user_id>")
    @protected(auth, role="admin")
    def update_user(user_id: int):
        payload = request.get_json() or {}
        auth.update_user(user_id, enabled=payload.get("enabled"), password=payload.get("password"))
        return audited(auth, "update_user", str(user_id), True)

    @app.get("/api/audit-logs")
    @protected(auth, role="admin", csrf=False)
    def audit_logs():
        return jsonify(auth.list_audit())

    return app


def protected(auth: AuthDatabase, *, role: str | None = None, csrf: bool = True):
    def decorator(function: Callable):
        @wraps(function)
        def wrapper(*args, **kwargs):
            user = current_user(auth)
            if not user or not user["enabled"]:
                return jsonify(error="未登录"), 401
            if role and user["role"] != role:
                return jsonify(error="权限不足"), 403
            if csrf and request.method not in {"GET", "HEAD", "OPTIONS"}:
                if not secrets.compare_digest(
                    request.headers.get("X-CSRF-Token", ""), session.get("csrf_token", "x")
                ):
                    return jsonify(error="CSRF 校验失败"), 403
            return function(*args, **kwargs)
        return wrapper
    return decorator


def current_user(auth: AuthDatabase) -> dict[str, Any] | None:
    user_id = session.get("user_id")
    return auth.get_user(int(user_id)) if user_id is not None else None


def public_user(user: dict[str, Any] | None) -> dict[str, Any] | None:
    return {"id": user["id"], "username": user["username"], "role": user["role"]} if user else None


def audited(auth: AuthDatabase, action: str, object_name: str, success: bool,
            detail: str = "", data: dict[str, Any] | None = None):
    user = current_user(auth)
    auth.audit(user["username"] if user else "-", request.remote_addr or "-",
               action, object_name, success, detail)
    return jsonify(success=success, message=detail, **(data or {})), 200 if success else 400


def inspect_wav(path: Path) -> tuple[float, int, int]:
    if path.suffix.lower() != ".wav":
        raise ValueError("仅支持 WAV")
    with wave.open(str(path), "rb") as handle:
        if handle.getcomptype() != "NONE" or handle.getsampwidth() != 2:
            raise ValueError("仅支持未压缩 16-bit PCM WAV")
        rate, channels, frames = handle.getframerate(), handle.getnchannels(), handle.getnframes()
    if rate <= 0 or channels not in {1, 2}:
        raise ValueError("WAV 采样率或声道无效")
    return frames / float(rate), int(rate), int(channels)


def validate_media_refs(spec: Any, database: RobotDatabase) -> None:
    if isinstance(spec, list):
        for item in spec:
            validate_media_refs(item, database)
    elif isinstance(spec, dict):
        if spec.get("type") == "play_audio":
            database.get_media(str(spec.get("media_key", "")))
        for value in spec.values():
            validate_media_refs(value, database)


def load_secret(path_value: str) -> str:
    path = Path(path_value).expanduser()
    if path.is_file():
        return path.read_text(encoding="utf-8").strip()
    path.parent.mkdir(parents=True, exist_ok=True)
    value = secrets.token_hex(32)
    path.write_text(value, encoding="utf-8")
    path.chmod(0o600)
    return value
