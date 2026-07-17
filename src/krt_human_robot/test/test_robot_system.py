from krt_human_robot.robot_system import (
    RobotSystemController,
    collect_routine_requirements,
    provider_commands,
)


def test_collects_nested_routine_requirements():
    spec = {
        "type": "sequence",
        "steps": [
            {"type": "speak", "text": "hello"},
            {"type": "arm_group", "arm_target": "left", "group_name": "wave"},
            {"type": "parallel", "steps": [
                {"type": "gripper", "action_name": "open"},
                {"type": "describe", "camera_id": "right_palm"},
                {"type": "photo", "topic": "/custom/image"},
            ]},
        ],
    }

    assert collect_routine_requirements(spec) == {
        "routine",
        "action_group_stack",
        "left_arm",
        "grippers",
        "tts",
        "playback",
        "vision",
        "camera:right_palm",
        "topic:/custom/image",
    }


def test_provider_commands_are_minimal():
    commands = provider_commands(
        {"tts", "playback", "vision", "camera:right_palm"},
        "/tmp/robot.yaml",
    )

    voice = commands["voice"]
    assert "enable_tts:=true" in voice
    assert "enable_playback:=true" in voice
    assert "enable_asr:=false" in voice
    assert commands["camera:right_palm"][2:4] == [
        "hand_camera_driver", "hand_cameras.launch.py"
    ]
    assert commands["vision"][-1] == "config_file:=/tmp/robot.yaml"


def test_shutdown_stops_only_owned_provider_process_groups(monkeypatch):
    class Process:
        pid = 321

        @staticmethod
        def poll():
            return None

    stopped = []
    monkeypatch.setattr(
        "krt_human_robot.robot_system.os.killpg",
        lambda pid, sig: stopped.append((pid, sig)),
    )
    controller = object.__new__(RobotSystemController)
    controller.provider_processes = [Process()]

    controller.shutdown_owned_providers()

    assert stopped and stopped[0][0] == 321
    assert controller.provider_processes == []
