import io
import threading
import time
import wave
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from krt_task.robot_db import RobotDatabase
from krt_human_robot.web_app import RosBridge, create_app


def web_test_app(tmp_path):
    return create_app({
        "TESTING": True,
        "SECRET_KEY": "test",
        "SESSION_COOKIE_SECURE": False,
        "ROS_ENABLED": False,
        "ROBOT_DB": str(tmp_path / "robot.db"),
        "WEB_DB": str(tmp_path / "web.db"),
        "MEDIA_DIR": str(tmp_path / "media"),
    })


def test_create_app_restores_saved_gripper_indices(monkeypatch, tmp_path):
    robot_db = tmp_path / "robot.db"
    database = RobotDatabase(str(robot_db))
    database.ensure_gripper_settings({
        "left": {
            "adapter_type": "ZLG_MINI", "adapter_index": 4, "device_id": 1,
            "listen_enabled": False, "realtime_response_enabled": False,
        },
        "right": {
            "adapter_type": "ZLG_MINI", "adapter_index": 7, "device_id": 2,
            "listen_enabled": False, "realtime_response_enabled": False,
        },
    })

    class Bridge:
        node = SimpleNamespace()
        hand_clients = {}
        hand_adapter_indices = {"left": 0, "right": 1}
        _future_result = None

        def __init__(self, robot_config):
            del robot_config

    class Controller:
        def __init__(self, *_args, **_kwargs):
            pass

        def shutdown_owned_processes(self):
            pass

        def shutdown_owned_providers(self):
            pass

    monkeypatch.setattr("krt_human_robot.web_app.RosBridge", Bridge)
    monkeypatch.setattr("krt_human_robot.web_app.GripperSystemController", Controller)
    monkeypatch.setattr("krt_human_robot.web_app.RobotSystemController", Controller)

    app = create_app({
        "TESTING": True,
        "SECRET_KEY": "test",
        "ROS_ENABLED": True,
        "ROBOT_DB": str(robot_db),
        "WEB_DB": str(tmp_path / "web.db"),
        "MEDIA_DIR": str(tmp_path / "media"),
    })

    assert app.extensions["runtime"].bridge.hand_adapter_indices == {
        "left": 4, "right": 7,
    }


@pytest.mark.parametrize(("secure", "expected"), (("0", False), ("1", True)))
def test_session_cookie_security_matches_web_transport(
        monkeypatch, tmp_path, secure, expected):
    monkeypatch.setenv("KRT_WEB_SESSION_COOKIE_SECURE", secure)
    app = create_app({
        "TESTING": True,
        "SECRET_KEY": "test",
        "ROS_ENABLED": False,
        "ROBOT_DB": str(tmp_path / "robot.db"),
        "WEB_DB": str(tmp_path / "web.db"),
        "MEDIA_DIR": str(tmp_path / "media"),
    })
    app.extensions["auth_db"].create_user("admin", "123456", "admin")

    response = app.test_client().post("/api/login", json={
        "username": "admin", "password": "123456",
    })

    assert ("Secure" in response.headers["Set-Cookie"]) is expected


def test_first_admin_setup_creates_session_and_then_closes(tmp_path):
    app = web_test_app(tmp_path)
    client = app.test_client()

    assert client.get("/api/session").get_json()["needs_setup"] is True
    setup = client.post("/api/setup", json={
        "username": "admin", "password": "123456",
        "password_confirmation": "123456",
    })
    assert setup.status_code == 200
    assert setup.get_json()["user"] == {
        "id": 1, "username": "admin", "role": "admin",
    }
    assert setup.get_json()["csrf_token"]
    assert client.get("/api/session").get_json()["needs_setup"] is False
    assert client.post("/api/setup", json={
        "username": "second", "password": "123456",
        "password_confirmation": "123456",
    }).status_code == 409


def test_setup_validates_confirmation_and_six_character_password(tmp_path):
    client = web_test_app(tmp_path).test_client()
    assert client.post("/api/setup", json={
        "username": "admin", "password": "12345",
        "password_confirmation": "12345",
    }).status_code == 400
    assert client.post("/api/setup", json={
        "username": "admin", "password": "123456",
        "password_confirmation": "654321",
    }).status_code == 400


def test_all_user_password_paths_use_six_character_minimum(tmp_path):
    auth = web_test_app(tmp_path).extensions["auth_db"]
    auth.create_user("admin", "123456", "admin")
    user_id = auth.list_users()[0]["id"]
    auth.update_user(user_id, password="654321")
    assert auth.authenticate("admin", "654321") is not None
    with pytest.raises(ValueError, match="至少 6"):
        auth.create_user("short", "12345", "operator")


def test_concurrent_setup_creates_only_one_admin(tmp_path):
    app = web_test_app(tmp_path)
    barrier = threading.Barrier(2)
    statuses = []

    def submit(username):
        with app.test_client() as client:
            barrier.wait()
            response = client.post("/api/setup", json={
                "username": username, "password": "123456",
                "password_confirmation": "123456",
            })
            statuses.append(response.status_code)

    threads = [
        threading.Thread(target=submit, args=(username,))
        for username in ("first", "second")
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert sorted(statuses) == [200, 409]
    users = app.extensions["auth_db"].list_users()
    assert len(users) == 1
    assert users[0]["role"] == "admin"


def wav_bytes() -> bytes:
    output = io.BytesIO()
    with wave.open(output, "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(16000)
        handle.writeframes(b"\0\0" * 160)
    return output.getvalue()


def test_login_csrf_media_and_routine(tmp_path):
    app = create_app({
        "TESTING": True,
        "SECRET_KEY": "test",
        "SESSION_COOKIE_SECURE": False,
        "ROS_ENABLED": False,
        "ROBOT_DB": str(tmp_path / "robot.db"),
        "WEB_DB": str(tmp_path / "web.db"),
        "MEDIA_DIR": str(tmp_path / "media"),
    })
    auth = app.extensions["auth_db"]
    auth.create_user("admin", "long-test-password", "admin")
    client = app.test_client()

    assert client.get("/").status_code == 200
    assert client.get("/api/routines").status_code == 401
    login = client.post("/api/login", json={
        "username": "admin", "password": "long-test-password",
    })
    csrf = login.get_json()["csrf_token"]
    headers = {"X-CSRF-Token": csrf}
    assert client.put("/api/routines/test", json={
        "type": "sequence", "steps": [{"type": "wait", "wait_ms": 10}],
    }).status_code == 403

    upload = client.post(
        "/api/media", headers=headers,
        data={"file": (io.BytesIO(wav_bytes()), "song.wav")},
        content_type="multipart/form-data",
    )
    assert upload.status_code == 200
    media_key = upload.get_json()["media_key"]
    saved = client.put("/api/routines/test", headers=headers, json={
        "type": "sequence",
        "steps": [{"type": "play_audio", "media_key": media_key}],
    })
    assert saved.status_code == 200
    assert client.get("/api/routines").get_json()[0]["name"] == "test"
    app.extensions["robot_db"].save_action_group("挥手", "left", [{
        "name": ["joint_1"], "position": [0.1], "velocity": [], "effort": [],
    }])
    assert client.get("/api/action-groups").get_json()[0]["name"] == "挥手"
    assert client.get("/api/gripper-actions/").status_code == 200
    renamed = client.patch("/api/action-groups/%E6%8C%A5%E6%89%8B", headers=headers,
                           json={"name": "新挥手"})
    assert renamed.status_code == 200
    assert client.get("/api/action-groups").get_json()[0]["name"] == "新挥手"
    deleted = client.delete(
        "/api/action-groups/%E6%96%B0%E6%8C%A5%E6%89%8B", headers=headers
    )
    assert deleted.status_code == 200
    assert client.get("/api/action-groups").get_json() == []


def test_operator_cannot_manage_users(tmp_path):
    app = create_app({
        "TESTING": True, "SECRET_KEY": "test", "SESSION_COOKIE_SECURE": False,
        "ROS_ENABLED": False, "ROBOT_DB": str(tmp_path / "robot.db"),
        "WEB_DB": str(tmp_path / "web.db"), "MEDIA_DIR": str(tmp_path / "media"),
    })
    app.extensions["auth_db"].create_user("operator", "long-test-password", "operator")
    client = app.test_client()
    client.post("/api/login", json={"username": "operator", "password": "long-test-password"})
    assert client.get("/api/users").status_code == 403


def authenticated_app(tmp_path):
    app = create_app({
        "TESTING": True, "SECRET_KEY": "test", "SESSION_COOKIE_SECURE": False,
        "ROS_ENABLED": False, "ROBOT_DB": str(tmp_path / "robot.db"),
        "WEB_DB": str(tmp_path / "web.db"), "MEDIA_DIR": str(tmp_path / "media"),
    })
    app.extensions["auth_db"].create_user("admin", "long-test-password", "admin")
    client = app.test_client()
    login = client.post("/api/login", json={
        "username": "admin", "password": "long-test-password",
    })
    return app, client, {"X-CSRF-Token": login.get_json()["csrf_token"]}


class FakeNavigationAdapter:
    def __init__(self, success=True):
        self.success = success
        self.calls = []

    def _result(self, name):
        self.calls.append(name)
        return SimpleNamespace(success=self.success, message=f"{name} result")

    def start_mapping(self):
        return self._result("start_mapping")

    def save_mapping(self):
        return self._result("save_mapping")

    def start_navigation(self):
        return self._result("start_navigation")

    def stop_navigation(self):
        return self._result("stop_navigation")

    def start_cruise(self):
        return self._result("start_cruise")

    def stop_cruise(self):
        return self._result("stop_cruise")

    def continue_waypoint_input(self):
        return self._result("continue_waypoint_input")


def test_navigation_control_api_dispatches_all_commands(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    adapter = FakeNavigationAdapter()
    app.extensions["runtime"].adapter = adapter
    routes = (
        ("/api/navigation/mapping/start", "start_mapping"),
        ("/api/navigation/mapping/finish", "save_mapping"),
        ("/api/navigation/start", "start_navigation"),
        ("/api/navigation/stop", "stop_navigation"),
        ("/api/navigation/cruise/start", "start_cruise"),
        ("/api/navigation/cruise/stop", "stop_cruise"),
        ("/api/navigation/cruise/resume", "continue_waypoint_input"),
    )

    for path, command in routes:
        response = client.post(path, headers=headers)
        assert response.status_code == 200
        assert response.get_json() == {
            "success": True, "message": f"{command} result",
        }

    assert adapter.calls == [command for _path, command in routes]


def test_navigation_control_api_returns_adapter_failure(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    app.extensions["runtime"].adapter = FakeNavigationAdapter(success=False)

    response = client.post("/api/navigation/start", headers=headers)

    assert response.status_code == 400
    assert response.get_json() == {
        "success": False, "message": "start_navigation result",
    }


def gripper_target(side="left", position=500):
    return {
        "side": side, "finger_id": 0, "position": position,
        "speed": 300, "force": 85, "wait_time": 10,
    }


def test_gripper_action_crud_and_reference_protection(tmp_path):
    _app, client, headers = authenticated_app(tmp_path)

    saved = client.put("/api/gripper-actions/%E5%8D%8A%E6%8F%A1", headers=headers,
                       json={"targets": [gripper_target()]})
    assert saved.status_code == 200
    assert client.get("/api/gripper-actions").get_json()[0]["name"] == "半握"
    assert client.get("/api/gripper-defaults").get_json()["open_position"] == 0

    routine = {"type": "sequence", "steps": [{
        "type": "gripper", "action_name": "半握",
    }]}
    assert client.put("/api/routines/test", headers=headers, json=routine).status_code == 200
    renamed = client.patch("/api/gripper-actions/%E5%8D%8A%E6%8F%A1", headers=headers,
                           json={"name": "左手半握"})
    assert renamed.status_code == 200
    assert client.get("/api/routines").get_json()[0]["spec"]["steps"][0][
        "action_name"
    ] == "左手半握"
    assert client.delete(
        "/api/gripper-actions/%E5%B7%A6%E6%89%8B%E5%8D%8A%E6%8F%A1",
        headers=headers,
    ).status_code == 400


class FakeGripperBridge:
    def __init__(self):
        self.state = {
            "mode": "idle", "status": "idle", "current_step": "", "message": "",
        }
        self.targets = None
        self.monitor_enabled = False

    def run_gripper(self, targets):
        self.targets = targets
        self.state = {
            "mode": "gripper", "status": "running", "current_step": "gripper:left",
            "message": "", "hands": {"left": {"progress": 0.5}},
        }

    def run(self, name):
        self.state = {
            "mode": "routine", "status": "running", "current_step": name,
            "message": "",
        }

    def run_arm_group(self, arm_target, group_name, repeat_count):
        self.state = {
            "mode": "arm_group", "status": "running",
            "current_step": group_name,
            "message": f"{arm_target}:{repeat_count}",
        }

    def status(self):
        return self.state

    def cancel(self):
        self.state["status"] = "failed"

    def set_monitor(self, enabled):
        self.monitor_enabled = enabled
        return {"enabled": enabled}

    def telemetry(self):
        return {"hands": {"left": {"fingers": [{"id": 1, "position": 123.0}]}}}


class FakeGripperSystem:
    def __init__(self):
        self.controls = []
        self.settings = []
        self.runtime = []
        self.hand_states = {"left": "active", "right": "active"}

    def status(self):
        return {"hands": {
            side: {"lifecycle_state": self.hand_states[side], "action_ready": True}
            for side in ("left", "right")
        }}

    def control(self, target, enabled):
        self.controls.append((target, enabled))
        return {"success": True, "hands": {target: {"success": True}}}

    def update_settings(self, side, changes):
        self.settings.append((side, changes))
        if not changes:
            return {
                "settings": {"side": side},
                "restart": {
                    "success": True, "state": "unconfigured",
                    "message": "参数未修改",
                },
            }
        return {
            "settings": {"side": side, **changes},
            "restart": {"success": True, "state": "active", "message": "已重启"},
        }

    def update_runtime(self, side, changes):
        self.runtime.append((side, changes))
        return {"side": side, **changes}


def test_gripper_system_status_control_and_settings_api(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    system = FakeGripperSystem()
    app.extensions["gripper_system"] = system

    assert client.get("/api/gripper/system").get_json()["hands"]["left"][
        "lifecycle_state"
    ] == "active"
    control = client.post("/api/gripper/system/control", headers=headers, json={
        "target": "both", "enabled": True,
    })
    assert control.status_code == 200
    assert system.controls == [("both", True)]
    settings = client.put("/api/gripper/system/left/settings", headers=headers, json={
        "adapter_type": "ZLG_MINI", "adapter_index": 3, "device_id": 4,
    })
    assert settings.status_code == 200
    assert settings.get_json()["restart"]["state"] == "active"
    assert settings.get_json()["message"] == "硬件参数已保存，夹爪已重启并开启"
    unchanged = client.put(
        "/api/gripper/system/left/settings", headers=headers, json={}
    )
    assert unchanged.get_json()["message"] == "参数未修改"
    runtime = client.put("/api/gripper/system/right/runtime", headers=headers, json={
        "listen_enabled": True, "realtime_response_enabled": False,
    })
    assert runtime.status_code == 200
    assert system.runtime[-1][0] == "right"


class FakeRobotSystem:
    def __init__(self):
        self.controls = []
        self.teach_calls = []

    def status(self):
        return {"components": {"left_arm": {"active": False}},
                "teaching": {"active": False}}

    def control(self, component, enabled):
        self.controls.append((component, enabled))
        return {"success": True, "components": {}}

    def start_teach(self, arm_target, group_name):
        self.teach_calls.append(("start", arm_target, group_name))
        return {"active": True, "arm_target": arm_target, "group_name": group_name}

    def stop_teach(self, group_name=""):
        self.teach_calls.append(("stop", group_name))
        return {"active": False, "group_name": group_name, "sample_count": 3}

    def ensure_providers(self, requirements):
        self.requirements = requirements


def test_robot_system_and_teach_api(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    system = FakeRobotSystem()
    app.extensions["robot_system"] = system

    assert client.get("/api/robot-systems").status_code == 200
    control = client.post(
        "/api/robot-systems/arms/control", headers=headers,
        json={"enabled": True},
    )
    assert control.status_code == 200
    assert system.controls == [("arms", True)]
    started = client.post(
        "/api/arm/teach/start", headers=headers,
        json={"arm_target": "left", "group_name": "挥手"},
    )
    assert started.status_code == 200
    stopped = client.post(
        "/api/arm/teach/stop", headers=headers, json={"group_name": "挥手"},
    )
    assert stopped.get_json()["sample_count"] == 3


def test_action_group_run_starts_dependencies_and_tracks_execution(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    system = FakeRobotSystem()
    bridge = FakeGripperBridge()
    app.extensions["robot_system"] = system
    app.extensions["runtime"].bridge = bridge
    app.extensions["robot_db"].save_action_group("挥手", "left", [{
        "name": ["joint_1"], "position": [0.1], "velocity": [], "effort": [],
    }])

    response = client.post(
        "/api/action-groups/%E6%8C%A5%E6%89%8B/run",
        headers=headers, json={"repeat_count": 2},
    )

    assert response.status_code == 200
    assert system.controls == [("left_arm", True), ("action_group_stack", True)]
    assert bridge.state["mode"] == "arm_group"


def test_operator_cannot_change_gripper_hardware_settings(tmp_path):
    app = create_app({
        "TESTING": True, "SECRET_KEY": "test", "SESSION_COOKIE_SECURE": False,
        "ROS_ENABLED": False, "ROBOT_DB": str(tmp_path / "robot.db"),
        "WEB_DB": str(tmp_path / "web.db"), "MEDIA_DIR": str(tmp_path / "media"),
    })
    app.extensions["gripper_system"] = FakeGripperSystem()
    app.extensions["auth_db"].create_user(
        "operator", "long-test-password", "operator"
    )
    client = app.test_client()
    login = client.post("/api/login", json={
        "username": "operator", "password": "long-test-password",
    })
    headers = {"X-CSRF-Token": login.get_json()["csrf_token"]}

    denied = client.put("/api/gripper/system/left/settings", headers=headers, json={
        "adapter_index": 2,
    })
    assert denied.status_code == 403
    allowed = client.post("/api/gripper/system/control", headers=headers, json={
        "target": "left", "enabled": True,
    })
    assert allowed.status_code == 200


def test_routine_automatically_starts_inactive_gripper(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    system = FakeGripperSystem()
    system.hand_states["left"] = "unconfigured"
    app.extensions["gripper_system"] = system
    app.extensions["runtime"].bridge = FakeGripperBridge()
    client.put("/api/gripper-actions/open-left", headers=headers, json={
        "targets": [gripper_target("left")],
    })
    client.put("/api/routines/grip", headers=headers, json={
        "type": "sequence",
        "steps": [{"type": "gripper", "action_name": "open-left"}],
    })

    response = client.post("/api/routines/grip/run", headers=headers)

    assert response.status_code == 200
    assert system.controls == [("both", True)]


def test_gripper_direct_run_is_tracked_and_mutually_exclusive(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    bridge = FakeGripperBridge()
    runtime = app.extensions["runtime"]
    runtime.bridge = bridge

    response = client.post("/api/gripper/run", headers=headers,
                           json={"targets": [gripper_target()]})
    assert response.status_code == 200
    assert bridge.targets[0]["position"] == 500
    execution = client.get("/api/execution").get_json()
    assert execution["mode"] == "gripper"
    assert execution["hands"]["left"]["progress"] == 0.5
    assert client.put("/api/routines/wait", headers=headers, json={
        "type": "sequence", "steps": [{"type": "wait", "wait_ms": 1}],
    }).status_code == 200
    blocked = client.post("/api/routines/wait/run", headers=headers)
    assert blocked.status_code == 400
    assert "已有任务" in blocked.get_json()["error"]


def test_gripper_monitor_and_telemetry_endpoints(tmp_path):
    app, client, headers = authenticated_app(tmp_path)
    bridge = FakeGripperBridge()
    app.extensions["runtime"].bridge = bridge

    started = client.post("/api/gripper/monitor", headers=headers,
                          json={"enabled": True})
    assert started.get_json()["enabled"] is True
    assert bridge.monitor_enabled is True
    snapshot = client.get("/api/gripper/telemetry").get_json()
    assert snapshot["hands"]["left"]["fingers"][0]["position"] == 123.0


class ImmediateFuture:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        callback(self)


class ImmediateGoalHandle:
    accepted = True

    def __init__(self, side):
        self.side = side
        self.canceled = False

    def get_result_async(self):
        result = type("Result", (), {
            "success": True, "message": f"{self.side} ok",
            "final_positions": [100, 200, 300],
        })()
        return ImmediateFuture(type("Wrapped", (), {"result": result})())

    def cancel_goal_async(self):
        self.canceled = True


class ImmediateActionClient:
    def __init__(self, side):
        self.side = side
        self.goals = []

    def wait_for_server(self, timeout_sec):
        return timeout_sec > 0

    def send_goal_async(self, goal, feedback_callback):
        self.goals.append(goal)
        feedback = type("Feedback", (), {
            "progress": 0.5, "current_positions": [10, 20, 30],
        })()
        feedback_callback(type("Message", (), {"feedback": feedback})())
        return ImmediateFuture(ImmediateGoalHandle(self.side))


class FailingActionClient(ImmediateActionClient):
    def send_goal_async(self, goal, feedback_callback):
        raise RuntimeError("send failed")


def test_ros_bridge_runs_both_hands_and_aggregates_feedback():
    bridge = RosBridge.__new__(RosBridge)
    bridge.lock = __import__("threading").Lock()
    bridge.state = {"mode": "idle", "status": "idle", "current_step": "", "message": ""}
    bridge.cancel_requested = False
    bridge.hand_clients = {
        "left": ImmediateActionClient("left"),
        "right": ImmediateActionClient("right"),
    }
    bridge.hand_adapter_indices = {"left": 4, "right": 7}
    bridge.gripper_goal_handles = {}

    bridge.run_gripper([gripper_target("left"), gripper_target("right")])

    assert bridge.status()["status"] == "succeeded"
    assert bridge.status()["hands"]["left"]["current_positions"] == [100, 200, 300]
    assert bridge.hand_clients["left"].goals[0].adapter_index == 4
    assert bridge.hand_clients["right"].goals[0].adapter_index == 7


def test_ros_bridge_finishes_when_sending_one_hand_goal_fails():
    bridge = RosBridge.__new__(RosBridge)
    bridge.lock = threading.Lock()
    bridge.state = {"mode": "idle", "status": "idle", "current_step": "", "message": ""}
    bridge.cancel_requested = False
    bridge.hand_clients = {
        "left": ImmediateActionClient("left"),
        "right": FailingActionClient("right"),
    }
    bridge.hand_adapter_indices = {"left": 0, "right": 1}
    bridge.gripper_goal_handles = {}

    bridge.run_gripper([gripper_target("left"), gripper_target("right")])

    assert bridge.status()["status"] == "failed"
    assert bridge.status()["hands"]["right"]["message"] == "send failed"


class ImmediateServiceClient:
    class srv_type:
        class Request:
            finger_id = 0

    def __init__(self, value, *, success=True, available=True):
        self.value = value
        self.success = success
        self.available = available

    def wait_for_service(self, timeout_sec):
        return timeout_sec > 0

    def call_async(self, _request):
        response = type("Response", (), {
            "success": self.success, "available": self.available,
            "value": self.value, "error_message": "read failed",
        })()
        return ImmediateFuture(response)


def test_ros_bridge_telemetry_keeps_partial_results():
    bridge = RosBridge.__new__(RosBridge)
    bridge.telemetry_lock = threading.Lock()
    bridge.monitor_lock = threading.Lock()
    bridge.monitor_active = True
    bridge.monitor_deadline = 0.0
    bridge.telemetry_clients = {
        "left": {"position": ImmediateServiceClient(123.0)},
        "right": {
            "position": ImmediateServiceClient(456.0),
            "normal_pressure": ImmediateServiceClient(1.25),
            "tangent_pressure": ImmediateServiceClient(0.0, success=False),
            "approaching": ImmediateServiceClient(8.0),
        },
    }

    snapshot = bridge.telemetry()

    assert snapshot["hands"]["left"]["fingers"][0]["position"]["value"] == 123.0
    right = snapshot["hands"]["right"]["fingers"][0]
    assert right["normal_pressure"]["value"] == 1.25
    assert right["tangent_pressure"]["available"] is False
    assert snapshot["hands"]["left"]["pressure_supported"] is False


def test_ros_bridge_monitor_lease_restores_previous_parameters():
    bridge = RosBridge.__new__(RosBridge)
    bridge.monitor_lock = threading.Lock()
    bridge.monitor_active = False
    bridge.monitor_previous = None
    bridge.monitor_deadline = 0.0
    states = []
    bridge._get_monitor_parameters = lambda: {
        "listen_enabled": False,
        "realtime_response_enabled": False,
    }
    bridge._set_monitor_parameters = lambda values: states.append(values)

    assert bridge.set_monitor(True)["enabled"] is True
    assert states[-1] == {"listen_enabled": True}
    bridge.monitor_deadline = time.monotonic() - 1.0
    bridge._monitor_watchdog()

    assert bridge.monitor_active is False
    assert states[-1] == {"listen_enabled": False, "realtime_response_enabled": False}


def test_ros_bridge_monitor_keeps_restore_state_after_failure():
    bridge = RosBridge.__new__(RosBridge)
    bridge.monitor_lock = threading.Lock()
    bridge.monitor_active = True
    bridge.monitor_previous = {
        "listen_enabled": False,
        "realtime_response_enabled": False,
    }
    bridge.monitor_deadline = time.monotonic() - 1.0
    bridge.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(warning=lambda *_args: None)
    )
    bridge._set_monitor_parameters = lambda _values: (_ for _ in ()).throw(
        RuntimeError("restore failed")
    )

    bridge._monitor_watchdog()

    assert bridge.monitor_active is True
    assert bridge.monitor_previous is not None


def test_monitor_watchdog_restores_parameters_through_executor(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    parameter_node = Node("hand_control_server", namespace="/right")
    parameter_node.declare_parameter("listen_enabled", False)
    parameter_node.declare_parameter("realtime_response_enabled", False)
    parameter_executor = MultiThreadedExecutor(num_threads=2)
    parameter_executor.add_node(parameter_node)
    parameter_thread = threading.Thread(target=parameter_executor.spin, daemon=True)
    parameter_thread.start()
    bridge = RosBridge()
    try:
        bridge.set_monitor(True)
        assert parameter_node.get_parameter("listen_enabled").value is True
        assert parameter_node.get_parameter("realtime_response_enabled").value is False

        bridge.monitor_deadline = time.monotonic() - 1.0
        deadline = time.monotonic() + 3.0
        while bridge.monitor_active and time.monotonic() < deadline:
            time.sleep(0.02)

        assert bridge.monitor_active is False
        assert parameter_node.get_parameter("listen_enabled").value is False
        assert parameter_node.get_parameter("realtime_response_enabled").value is False
    finally:
        bridge.executor.shutdown(timeout_sec=2.0)
        bridge.node.destroy_node()
        parameter_executor.shutdown(timeout_sec=2.0)
        parameter_node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_web_console_launch_passes_robot_config():
    package = Path(__file__).parents[1]
    web_launch = (package / "launch/web_console.launch.py").read_text(encoding="utf-8")
    assert 'if certfile and keyfile:' in web_launch
    assert 'cmd.extend(["--certfile", certfile, "--keyfile", keyfile])' in web_launch
    assert '"KRT_WEB_SESSION_COOKIE_SECURE": "1" if certfile and keyfile else "0"' in web_launch
    robot_launch = (package / "launch/robot.launch.py").read_text(encoding="utf-8")

    assert '"config_file", default_value=' in web_launch
    assert '"KRT_HUMAN_ROBOT_CONFIG": LaunchConfiguration("config_file")' in web_launch
    assert '"config_file": config_file' in robot_launch
    assert 'DeclareLaunchArgument("hands_autostart", default_value="true")' in robot_launch
    assert '"autostart": LaunchConfiguration("hands_autostart")' in robot_launch
