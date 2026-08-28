from geometry_msgs.msg import PoseStamped
from krt_task.robot_db import RobotDatabase
from ranger_nav.waypoint_manager import Waypoint, WaypointStore, build_parser


def ready_map(database: RobotDatabase, name: str):
    record = database.create_map(name, "fast_lio")
    return database.complete_map(record.id, {
        "session_dir": f"/maps/{name}",
        "yaml_path": f"/maps/{name}/map.yaml",
        "pgm_path": f"/maps/{name}/map.pgm",
        "pcd_path": f"/maps/{name}/cloud.pcd",
        "metadata_path": f"/maps/{name}/metadata.yaml",
    })


def test_waypoint_store_uses_selected_map(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    first = ready_map(database, "一楼")
    second = ready_map(database, "二楼")
    database.select_map(second.id)
    store = WaypointStore(str(tmp_path / "robot.db"))
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0

    store.save(Waypoint("走廊", pose))

    assert store.load(first.id) == []
    saved = store.load(second.id)
    assert len(saved) == 1
    assert saved[0].map_id == second.id


def test_waypoint_parser_accepts_global_map_id():
    args = build_parser().parse_args(["--map-id", "map-1", "mark", "大厅"])

    assert args.map_id == "map-1"
    assert args.command == "mark"
    assert args.name == "大厅"
