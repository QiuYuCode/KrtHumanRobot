import io
import threading
import time
import wave
from pathlib import Path

from krt_human_robot.web_app import RosBridge, create_app


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
    renamed = client.patch("/api/action-groups/%E6%8C%A5%E6%89%8B", headers=headers,
                           json={"name": "新挥手"})
    assert renamed.status_code == 200
    assert client.get("/api/action-groups").get_json()[0]["name"] == "新挥手"


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

    def status(self):
        return self.state

    def cancel(self):
        self.state["status"] = "failed"

    def set_monitor(self, enabled):
        self.monitor_enabled = enabled
        return {"enabled": enabled}

    def telemetry(self):
        return {"hands": {"left": {"fingers": [{"id": 1, "position": 123.0}]}}}


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
    assert states[-1] == {"listen_enabled": True, "realtime_response_enabled": True}
    bridge.monitor_deadline = time.monotonic() - 1.0
    bridge._monitor_watchdog()

    assert bridge.monitor_active is False
    assert states[-1] == {"listen_enabled": False, "realtime_response_enabled": False}


def test_web_console_launch_passes_robot_config():
    package = Path(__file__).parents[1]
    web_launch = (package / "launch/web_console.launch.py").read_text(encoding="utf-8")
    robot_launch = (package / "launch/robot.launch.py").read_text(encoding="utf-8")

    assert '"config_file", default_value=' in web_launch
    assert '"KRT_HUMAN_ROBOT_CONFIG": LaunchConfiguration("config_file")' in web_launch
    assert '"config_file": config_file' in robot_launch
