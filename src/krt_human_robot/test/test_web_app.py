import io
import wave

from krt_human_robot.web_app import create_app


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
