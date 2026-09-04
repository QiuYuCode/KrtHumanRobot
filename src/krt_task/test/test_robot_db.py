import sqlite3

import pytest
from krt_task.robot_db import RobotDatabase, WaypointRecord


def test_robot_database_routine_waypoint_and_media(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.add_media(
        {
            "media_key": "song",
            "display_name": "song.wav",
            "filename": "song.wav",
            "size_bytes": 4,
            "duration_sec": 1.0,
            "sample_rate": 16000,
            "channels": 1,
        }
    )
    database.save_routine(
        "表演",
        {
            "type": "sequence",
            "steps": [{"type": "play_audio", "media_key": "song"}],
        },
    )
    database.save_waypoint(WaypointRecord("大厅", "map", 1, 2, 0, 0, 0, 0, 1, "表演"))
    assert database.list_waypoints()[0].routine == "表演"

    database.delete_routine("表演")
    assert database.list_waypoints()[0].routine == ""


def test_routine_validation_rejects_nested_parallel(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    with pytest.raises(ValueError, match="parallel"):
        database.save_routine(
            "坏流程",
            {
                "type": "parallel",
                "steps": [
                    {"type": "parallel", "steps": [{"type": "wait", "wait_ms": 1}]}
                ],
            },
        )


def test_action_group_storage_and_legacy_migration(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.save_action_group(
        "挥手",
        "left",
        [
            {
                "name": ["joint_1"],
                "position": [0.1],
                "velocity": [],
                "effort": [],
            }
        ],
    )
    assert database.get_action_group("挥手")["arm_target"] == "left"

    steps = tmp_path / "steps"
    steps.mkdir()
    (steps / "step.yaml").write_text(
        "name: [joint_1]\nposition: [0.2]\n", encoding="utf-8"
    )
    legacy = tmp_path / "action_groups.yaml"
    legacy.write_text(
        "groups:\n  敬礼:\n    steps:\n      - payload_file: steps/step.yaml\n",
        encoding="utf-8",
    )
    assert database.migrate_action_groups(str(legacy)) == 1
    assert database.get_action_group("敬礼")["arm_target"] == "unknown"


def test_rename_action_group_updates_routines(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.save_action_group(
        "未命名-左臂",
        "left",
        [
            {
                "name": ["joint_1"],
                "position": [0.1],
                "velocity": [],
                "effort": [],
            }
        ],
    )
    database.save_routine(
        "表演",
        {
            "type": "sequence",
            "steps": [
                {
                    "type": "parallel",
                    "steps": [
                        {
                            "type": "arm_group",
                            "arm_target": "left",
                            "group_name": "未命名-左臂",
                        }
                    ],
                }
            ],
        },
    )
    database.rename_action_group("未命名-左臂", "挥手")
    assert database.get_action_group("挥手")["arm_target"] == "left"
    assert database.get_routine("表演")["steps"][0]["steps"][0]["group_name"] == "挥手"


def test_delete_action_group_rejects_routine_reference(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    sample = [
        {
            "name": ["joint_1"],
            "position": [0.1],
            "velocity": [],
            "effort": [],
        }
    ]
    database.save_action_group("挥手", "left", sample)
    database.save_routine(
        "表演",
        {
            "type": "sequence",
            "steps": [
                {"type": "arm_group", "arm_target": "left", "group_name": "挥手"}
            ],
        },
    )

    with pytest.raises(ValueError, match="正在被 routine 使用"):
        database.delete_action_group("挥手")

    database.save_action_group("点头", "left", sample)
    database.delete_action_group("点头")
    with pytest.raises(KeyError):
        database.get_action_group("点头")


def test_gripper_action_storage_and_schema_migration(tmp_path):
    path = tmp_path / "robot.db"
    database = RobotDatabase(str(path))
    database.save_gripper_action(
        "左手张开",
        [
            {
                "side": "left",
                "finger_id": 0,
                "position": 0,
                "speed": 240,
                "force": 85,
                "wait_time": 10,
            }
        ],
    )

    assert database.get_gripper_action("左手张开")["targets"][0]["position"] == 0
    assert database.list_gripper_actions()[0]["name"] == "左手张开"
    assert "targets_json" not in database.list_gripper_actions()[0]
    with sqlite3.connect(path) as connection:
        assert connection.execute("PRAGMA user_version").fetchone()[0] == 7


def test_routine_configuration_saves_normalized_voice_trigger(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    spec = {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 10}]}

    database.save_routine_configuration(
        "迎宾",
        spec,
        {
            "keywords": ["  开始迎宾  ", "开始迎宾", "迎接客人"],
            "response_text": "  迎宾完成。  ",
        },
    )

    routine = database.list_routines()[0]
    assert routine["spec"] == spec
    assert routine["voice_trigger"] == {
        "keywords": ["开始迎宾", "迎接客人"],
        "response_text": "迎宾完成。",
    }


def test_routine_voice_trigger_rejects_duplicate_without_partial_update(
    tmp_path,
):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    old_spec = {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 10}]}
    new_spec = {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 20}]}
    database.save_routine_configuration(
        "迎宾",
        old_spec,
        {"keywords": ["开始表演"], "response_text": ""},
    )
    database.save_routine("送客", old_spec)

    with pytest.raises(ValueError, match="已绑定到 routine.*迎宾"):
        database.save_routine_configuration(
            "送客",
            new_spec,
            {"keywords": ["开始表演"], "response_text": "完成"},
        )

    assert database.get_routine("送客") == old_spec
    assert database.list_routines()[1]["voice_trigger"] == {
        "keywords": [],
        "response_text": "",
    }


def test_empty_voice_trigger_stays_disabled_and_cascades_on_delete(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    spec = {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 10}]}
    database.save_routine_configuration(
        "迎宾",
        spec,
        {"keywords": [], "response_text": "不会播报"},
    )

    assert database.list_routine_voice_triggers() == [
        {
            "routine_name": "迎宾",
            "keywords": [],
            "response_text": "不会播报",
        }
    ]
    database.delete_routine("迎宾")
    assert database.list_routine_voice_triggers() == []


def test_legacy_voice_import_retries_after_routine_is_created(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    entries = [
        {
            "routine_name": "稍后创建",
            "keywords": ["开始稍后流程"],
            "response_text": "流程完成",
        }
    ]

    assert database.import_legacy_routine_voice_triggers(entries) == 0
    database.save_routine(
        "稍后创建",
        {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 10}]},
    )

    assert database.import_legacy_routine_voice_triggers(entries) == 1
    assert database.list_routine_voice_triggers() == [
        {
            "routine_name": "稍后创建",
            "keywords": ["开始稍后流程"],
            "response_text": "流程完成",
        }
    ]


def test_schema_v5_migrates_existing_routine_to_voice_trigger_tables(tmp_path):
    path = tmp_path / "v5.db"
    spec = '{"type":"sequence","steps":[{"type":"wait","wait_ms":10}]}'
    with sqlite3.connect(path) as connection:
        connection.executescript(
            """
            CREATE TABLE routines (
              id INTEGER PRIMARY KEY,
              name TEXT NOT NULL UNIQUE,
              spec_json TEXT NOT NULL,
              created_at TEXT NOT NULL,
              updated_at TEXT NOT NULL
            );
            PRAGMA user_version = 5;
            """
        )
        connection.execute(
            """INSERT INTO routines(name, spec_json, created_at, updated_at)
               VALUES(?, ?, ?, ?)""",
            ("旧流程", spec, "2026-01-01T00:00:00+00:00", "2026-01-01T00:00:00+00:00"),
        )

    database = RobotDatabase(str(path))

    assert database.get_routine("旧流程")["steps"][0]["wait_ms"] == 10
    with sqlite3.connect(path) as connection:
        assert connection.execute("PRAGMA user_version").fetchone()[0] == 7
        assert {
            row[0]
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type = 'table'"
            )
        } >= {"routine_voice_triggers", "metadata"}


def test_legacy_routine_voice_triggers_import_only_once(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    spec = {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 10}]}
    database.save_routine("唱跳表演", spec)
    entries = [
        {
            "routine_name": "唱跳表演",
            "keywords": ["唱跳表演", "开始表演"],
            "response_text": "表演完成",
        },
        {"routine_name": "不存在", "keywords": ["不会导入"]},
    ]

    assert database.import_legacy_routine_voice_triggers(entries) == 1
    database.save_routine_configuration(
        "唱跳表演",
        spec,
        {"keywords": [], "response_text": ""},
    )
    assert database.import_legacy_routine_voice_triggers(entries) == 0
    assert database.list_routines()[0]["voice_trigger"]["keywords"] == []


def test_maps_are_selected_and_waypoints_are_scoped(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    first = database.create_map("一楼", "fast_lio")
    first = database.complete_map(
        first.id,
        {
            "session_dir": "/maps/one",
            "yaml_path": "/maps/one/map.yaml",
            "pgm_path": "/maps/one/map.pgm",
            "pcd_path": "/maps/one/cloud.pcd",
            "metadata_path": "/maps/one/metadata.yaml",
        },
    )
    second = database.create_map("二楼", "spark_sam")
    second = database.complete_map(
        second.id,
        {
            "session_dir": "/maps/two",
            "yaml_path": "/maps/two/map.yaml",
            "pgm_path": "/maps/two/map.pgm",
            "pcd_path": "/maps/two/cloud.pcd",
            "metadata_path": "/maps/two/metadata.yaml",
        },
    )

    assert first.selected is True
    assert second.selected is False
    database.select_map(second.id)
    selected = database.get_selected_map()
    assert selected is not None
    assert selected.id == second.id

    database.save_waypoint(
        WaypointRecord("二楼走廊", "map", 1, 2, 0, 0, 0, 0, 1, map_id=second.id)
    )
    assert database.list_waypoints(first.id) == []
    assert database.list_waypoints(second.id)[0].map_id == second.id
    assert database.list_maps()[0].waypoint_count == 1


def test_waypoint_map_binding_requires_ready_map(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    pending = database.create_map("保存中", "fast_lio")
    database.save_waypoint(WaypointRecord("旧点位", "map", 0, 0, 0, 0, 0, 0, 1))

    with pytest.raises(KeyError, match="已就绪地图不存在"):
        database.bind_waypoint_map("旧点位", pending.id)


def test_gripper_system_settings_are_initialized_and_updated(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    defaults = {
        "left": {
            "adapter_type": "ZLG_MINI",
            "adapter_index": 0,
            "device_id": 1,
            "listen_enabled": False,
            "realtime_response_enabled": False,
        },
        "right": {
            "adapter_type": "ZLG_200U",
            "adapter_index": 1,
            "device_id": 2,
            "listen_enabled": True,
            "realtime_response_enabled": False,
        },
    }

    database.ensure_gripper_settings(defaults)
    database.update_gripper_settings(
        "left",
        {
            "adapter_index": 3,
            "listen_enabled": True,
        },
    )

    settings = {item["side"]: item for item in database.list_gripper_settings()}
    assert settings["left"]["adapter_index"] == 3
    assert settings["left"]["listen_enabled"] is True
    assert settings["right"]["adapter_type"] == "ZLG_200U"


def test_gripper_system_settings_validate_hardware_ranges(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    defaults = {
        "left": {
            "adapter_type": "ZLG_MINI",
            "adapter_index": 0,
            "device_id": 1,
            "listen_enabled": False,
            "realtime_response_enabled": False,
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
    database.save_gripper_action(
        "右手张开",
        [
            {
                "side": "right",
                "finger_id": 0,
                "position": 0,
                "speed": 240,
                "force": 85,
                "wait_time": 10,
            }
        ],
    )
    assert database.get_gripper_action("右手张开")["name"] == "右手张开"


def test_gripper_action_validates_targets(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    target = {
        "side": "left",
        "finger_id": 0,
        "position": 500,
        "speed": 500,
        "force": 85,
        "wait_time": 10,
    }

    with pytest.raises(ValueError, match="左右手目标不能重复"):
        database.save_gripper_action("重复", [target, dict(target)])
    with pytest.raises(ValueError, match="position"):
        database.save_gripper_action("越界", [{**target, "position": 1001}])


def test_gripper_action_rename_updates_routine_and_delete_is_protected(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    database.save_gripper_action(
        "半握",
        [
            {
                "side": "right",
                "finger_id": 0,
                "position": 500,
                "speed": 300,
                "force": 85,
                "wait_time": 10,
            }
        ],
    )
    database.save_routine(
        "抓取",
        {
            "type": "sequence",
            "steps": [{"type": "gripper", "action_name": "半握"}],
        },
    )

    database.rename_gripper_action("半握", "右手半握")
    assert database.get_routine("抓取")["steps"][0]["action_name"] == "右手半握"
    with pytest.raises(ValueError, match="正在被 routine 使用"):
        database.delete_gripper_action("右手半握")


def test_routine_rejects_missing_gripper_action_and_keeps_inline_compatibility(
    tmp_path,
):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    with pytest.raises(KeyError, match="夹爪动作不存在"):
        database.save_routine(
            "缺失",
            {
                "type": "sequence",
                "steps": [{"type": "gripper", "action_name": "不存在"}],
            },
        )

    database.save_routine(
        "旧格式",
        {
            "type": "sequence",
            "steps": [
                {
                    "type": "gripper",
                    "side": "left",
                    "finger_id": 0,
                    "position": 0,
                    "speed": 240,
                    "force": 85,
                    "wait_time": 10,
                }
            ],
        },
    )
    assert database.get_routine("旧格式")["steps"][0]["side"] == "left"
