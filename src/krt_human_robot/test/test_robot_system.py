from types import SimpleNamespace

from krt_human_robot.robot_system import (
    RobotSystemController,
    collect_routine_requirements,
    provider_commands,
)


def test_routine_command_uses_configured_hand_adapter_indices(monkeypatch):
    monkeypatch.setattr(
        "krt_human_robot.robot_system.ActionClient",
        lambda *_args, **_kwargs: object(),
    )

    class Node:
        def create_client(self, *_args, **_kwargs):
            return object()

    controller = RobotSystemController(
        Node(), SimpleNamespace(robot_arm_channels={}),
        future_result=lambda *_args: None,
        robot_db="/tmp/robot.db", media_dir="/tmp/music",
        hand_adapter_indices={"left": 4, "right": 7},
    )

    assert "left_hand_adapter_index:=4" in controller.commands["routine"]
    assert "right_hand_adapter_index:=7" in controller.commands["routine"]


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


def test_describe_requires_voice_output_resources():
    assert collect_routine_requirements({
        "type": "describe", "camera_id": "head",
    }) == {"routine", "vision", "camera:head", "tts", "playback"}


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


def test_controller_uses_compressed_camera_topics_for_readiness(monkeypatch):
    monkeypatch.setattr(
        "krt_human_robot.robot_system.ActionClient",
        lambda *_args, **_kwargs: object(),
    )

    class Node:
        def create_client(self, *_args, **_kwargs):
            return object()

    config = SimpleNamespace(
        robot_arm_channels={},
        camera_ros_transport="compressed",
        cameras={
            "head": {
                "ros_topic": "/camera/camera/color/image_raw",
                "ros_compressed_topic": "/camera/camera/color/image_jpeg",
            },
            "left_palm": {"ros_topic": "/left_gripper/image_raw"},
        },
    )
    controller = RobotSystemController(
        Node(), config, future_result=lambda *_args: None,
        robot_db="/tmp/robot.db", media_dir="/tmp/music",
    )

    assert controller._camera_topics == {
        "head": "/camera/camera/color/image_jpeg",
        "left_palm": "/left_gripper/image_raw/compressed",
    }
    assert "camera:head" not in provider_commands({"camera:head"}, "")


def test_controller_uses_raw_camera_topics_for_readiness(monkeypatch):
    monkeypatch.setattr(
        "krt_human_robot.robot_system.ActionClient",
        lambda *_args, **_kwargs: object(),
    )

    class Node:
        def create_client(self, *_args, **_kwargs):
            return object()

    config = SimpleNamespace(
        robot_arm_channels={},
        camera_ros_transport="raw",
        cameras={"head": {"ros_topic": "/camera/camera/color/image_raw"}},
    )
    controller = RobotSystemController(
        Node(), config, future_result=lambda *_args: None,
        robot_db="/tmp/robot.db", media_dir="/tmp/music",
    )

    assert controller._camera_topics["head"] == "/camera/camera/color/image_raw"


def test_head_readiness_checks_selected_topic(monkeypatch):
    controller = object.__new__(RobotSystemController)
    controller._camera_topics = {
        "head": "/camera/camera/color/image_jpeg",
    }
    queried = []

    def publishers(topic):
        queried.append(topic)
        return [object()] if topic.endswith("image_jpeg") else []

    controller.node = SimpleNamespace(
        get_publishers_info_by_topic=publishers
    )

    assert controller._provider_ready("camera:head")
    assert queried == ["/camera/camera/color/image_jpeg"]


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
