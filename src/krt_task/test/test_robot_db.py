import sqlite3

import pytest

from krt_task.robot_db import RobotDatabase, WaypointRecord


def test_robot_database_routine_waypoint_and_media(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.add_media({
        "media_key": "song", "display_name": "song.wav", "filename": "song.wav",
        "size_bytes": 4, "duration_sec": 1.0, "sample_rate": 16000, "channels": 1,
    })
    database.save_routine("表演", {
        "type": "sequence",
        "steps": [{"type": "play_audio", "media_key": "song"}],
    })
    database.save_waypoint(WaypointRecord(
        "大厅", "map", 1, 2, 0, 0, 0, 0, 1, "表演"
    ))
    assert database.list_waypoints()[0].routine == "表演"

    database.delete_routine("表演")
    assert database.list_waypoints()[0].routine == ""


def test_routine_validation_rejects_nested_parallel(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    with pytest.raises(ValueError, match="parallel"):
        database.save_routine("坏流程", {
            "type": "parallel",
            "steps": [{"type": "parallel", "steps": [{"type": "wait", "wait_ms": 1}]}],
        })


def test_action_group_storage_and_legacy_migration(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.save_action_group("挥手", "left", [{
        "name": ["joint_1"], "position": [0.1], "velocity": [], "effort": [],
    }])
    assert database.get_action_group("挥手")["arm_target"] == "left"

    steps = tmp_path / "steps"
    steps.mkdir()
    (steps / "step.yaml").write_text("name: [joint_1]\nposition: [0.2]\n", encoding="utf-8")
    legacy = tmp_path / "action_groups.yaml"
    legacy.write_text(
        "groups:\n  敬礼:\n    steps:\n      - payload_file: steps/step.yaml\n",
        encoding="utf-8",
    )
    assert database.migrate_action_groups(str(legacy)) == 1
    assert database.get_action_group("敬礼")["arm_target"] == "unknown"


def test_rename_action_group_updates_routines(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.save_action_group("未命名-左臂", "left", [{
        "name": ["joint_1"], "position": [0.1], "velocity": [], "effort": [],
    }])
    database.save_routine("表演", {
        "type": "sequence",
        "steps": [{"type": "parallel", "steps": [{
            "type": "arm_group", "arm_target": "left", "group_name": "未命名-左臂",
        }]}],
    })
    database.rename_action_group("未命名-左臂", "挥手")
    assert database.get_action_group("挥手")["arm_target"] == "left"
    assert database.get_routine("表演")["steps"][0]["steps"][0]["group_name"] == "挥手"


def test_delete_action_group_rejects_routine_reference(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    sample = [{
        "name": ["joint_1"], "position": [0.1], "velocity": [], "effort": [],
    }]
    database.save_action_group("挥手", "left", sample)
    database.save_routine("表演", {
        "type": "sequence",
        "steps": [{"type": "arm_group", "arm_target": "left", "group_name": "挥手"}],
    })

    with pytest.raises(ValueError, match="正在被 routine 使用"):
        database.delete_action_group("挥手")

    database.save_action_group("点头", "left", sample)
    database.delete_action_group("点头")
    with pytest.raises(KeyError):
        database.get_action_group("点头")


def test_gripper_action_storage_and_schema_migration(tmp_path):
    path = tmp_path / "robot.db"
    database = RobotDatabase(str(path))
    database.save_gripper_action("左手张开", [{
        "side": "left", "finger_id": 0, "position": 0,
        "speed": 240, "force": 85, "wait_time": 10,
    }])

    assert database.get_gripper_action("左手张开")["targets"][0]["position"] == 0
    assert database.list_gripper_actions()[0]["name"] == "左手张开"
    assert "targets_json" not in database.list_gripper_actions()[0]
    with sqlite3.connect(path) as connection:
        assert connection.execute("PRAGMA user_version").fetchone()[0] == 4


def test_gripper_system_settings_are_initialized_and_updated(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    defaults = {
        "left": {
            "adapter_type": "ZLG_MINI", "adapter_index": 0, "device_id": 1,
            "listen_enabled": False, "realtime_response_enabled": False,
        },
        "right": {
            "adapter_type": "ZLG_200U", "adapter_index": 1, "device_id": 2,
            "listen_enabled": True, "realtime_response_enabled": False,
        },
    }

    database.ensure_gripper_settings(defaults)
    database.update_gripper_settings("left", {
        "adapter_index": 3, "listen_enabled": True,
    })

    settings = {item["side"]: item for item in database.list_gripper_settings()}
    assert settings["left"]["adapter_index"] == 3
    assert settings["left"]["listen_enabled"] is True
    assert settings["right"]["adapter_type"] == "ZLG_200U"


def test_gripper_system_settings_validate_hardware_ranges(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    defaults = {
        "left": {
            "adapter_type": "ZLG_MINI", "adapter_index": 0, "device_id": 1,
            "listen_enabled": False, "realtime_response_enabled": False,
        },
    }
    database.ensure_gripper_settings(defaults)

    with pytest.raises(ValueError, match="adapter_type"):
        database.update_gripper_settings("left", {"adapter_type": "LYS_MINI"})
    with pytest.raises(ValueError, match="adapter_index"):
        database.update_gripper_settings("left", {"adapter_index": 16})
    with pytest.raises(ValueError, match="device_id"):
        database.update_gripper_settings("left", {"device_id": 0})
    with pytest.raises(ValueError, match="side"):
        database.update_gripper_settings("middle", {"device_id": 3})


def test_gripper_action_migrates_existing_v2_database(tmp_path):
    path = tmp_path / "robot.db"
    RobotDatabase(str(path))
    with sqlite3.connect(path) as connection:
        connection.execute("DROP TABLE gripper_actions")
        connection.execute("PRAGMA user_version = 2")

    database = RobotDatabase(str(path))
    database.save_gripper_action("右手张开", [{
        "side": "right", "finger_id": 0, "position": 0,
        "speed": 240, "force": 85, "wait_time": 10,
    }])
    assert database.get_gripper_action("右手张开")["name"] == "右手张开"


def test_gripper_action_validates_targets(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    target = {
        "side": "left", "finger_id": 0, "position": 500,
        "speed": 500, "force": 85, "wait_time": 10,
    }

    with pytest.raises(ValueError, match="左右手目标不能重复"):
        database.save_gripper_action("重复", [target, dict(target)])
    with pytest.raises(ValueError, match="position"):
        database.save_gripper_action("越界", [{**target, "position": 1001}])


def test_gripper_action_rename_updates_routine_and_delete_is_protected(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.save_gripper_action("半握", [{
        "side": "right", "finger_id": 0, "position": 500,
        "speed": 300, "force": 85, "wait_time": 10,
    }])
    database.save_routine("抓取", {
        "type": "sequence",
        "steps": [{"type": "gripper", "action_name": "半握"}],
    })

    database.rename_gripper_action("半握", "右手半握")
    assert database.get_routine("抓取")["steps"][0]["action_name"] == "右手半握"
    with pytest.raises(ValueError, match="正在被 routine 使用"):
        database.delete_gripper_action("右手半握")


def test_routine_rejects_missing_gripper_action_and_keeps_inline_compatibility(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    with pytest.raises(KeyError, match="夹爪动作不存在"):
        database.save_routine("缺失", {
            "type": "sequence",
            "steps": [{"type": "gripper", "action_name": "不存在"}],
        })

    database.save_routine("旧格式", {
        "type": "sequence",
        "steps": [{
            "type": "gripper", "side": "left", "finger_id": 0,
            "position": 0, "speed": 240, "force": 85, "wait_time": 10,
        }],
    })
    assert database.get_routine("旧格式")["steps"][0]["side"] == "left"
