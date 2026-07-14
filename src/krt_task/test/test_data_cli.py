from argparse import Namespace

import yaml

from krt_task.data_cli import export_yaml, import_yaml
from krt_task.robot_db import RobotDatabase


def test_yaml_import_export(tmp_path):
    routines = tmp_path / "routines.yaml"
    waypoints = tmp_path / "waypoints.yaml"
    routines.write_text(yaml.safe_dump({
        "routines": {
            "等待": {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 1}]}
        }
    }, allow_unicode=True), encoding="utf-8")
    waypoints.write_text(yaml.safe_dump({
        "waypoints": [{
            "name": "大厅", "frame_id": "map",
            "position": {"x": 1, "y": 2, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
            "routine": "等待",
        }]
    }, allow_unicode=True), encoding="utf-8")
    database_path = tmp_path / "robot.db"
    import_yaml(Namespace(
        db=str(database_path), routines=str(routines), waypoints=str(waypoints),
        media_dir=str(tmp_path / "media"),
    ))
    assert RobotDatabase(str(database_path)).list_waypoints()[0].routine == "等待"

    output = tmp_path / "backup.yaml"
    export_yaml(Namespace(db=str(database_path), output=str(output)))
    assert yaml.safe_load(output.read_text(encoding="utf-8"))["routines"]["等待"]
