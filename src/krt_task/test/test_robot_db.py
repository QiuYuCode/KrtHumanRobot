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
