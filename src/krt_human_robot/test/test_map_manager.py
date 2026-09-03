from pathlib import Path

from krt_task.robot_db import RobotDatabase, WaypointRecord

from krt_human_robot.adapters.navigation import NavigationResult
from krt_human_robot.map_manager import MapManager


class FakeAdapter:
    def __init__(self, map_root: Path):
        self.map_root = map_root
        self.mapping = False
        self.navigation = False
        self.mapping_calls = []
        self.navigation_calls = []

    def mapping_running(self):
        return self.mapping

    def navigation_running(self):
        return self.navigation

    def default_navigation_mode(self):
        return "3dloc"

    def start_mapping(self, backend=None, *, rviz=True):
        self.mapping_calls.append((backend, rviz))
        self.mapping = True
        return NavigationResult(True, "已开始建图。")

    def save_mapping(self):
        session = self.map_root / "20260828_120000"
        session.mkdir(parents=True)
        files = {
            "yaml_path": session / "map.yaml",
            "pgm_path": session / "map.pgm",
            "pcd_path": session / "cloud.pcd",
            "metadata_path": session / "metadata.yaml",
        }
        files["yaml_path"].write_text(
            "image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            encoding="utf-8",
        )
        files["pgm_path"].write_bytes(b"P5\n2 2\n255\n\x00\xff\xcd\x00")
        files["pcd_path"].write_bytes(b"pcd")
        files["metadata_path"].write_bytes(b"metadata")
        self.mapping = False
        return NavigationResult(
            True,
            "地图保存完成。",
            {
                "session_dir": str(session),
                **{key: str(path) for key, path in files.items()},
            },
        )

    def start_navigation(
        self, map_yaml=None, pcd_map_path=None, mode=None, *, initial_pose=None, rviz=True
    ):
        self.navigation_calls.append((map_yaml, pcd_map_path, mode, initial_pose, rviz))
        self.navigation = True
        return NavigationResult(True, "已开始导航。")

    def stop_navigation(self):
        self.navigation = False
        return NavigationResult(True, "导航已退出。")


def test_mapping_is_registered_and_selected(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))

    started = manager.start_mapping("一楼大厅", "fast_lio", rviz=True)
    assert started.success is True
    assert adapter.mapping_calls == [("fast_lio", True)]
    assert database.list_maps()[0].status == "saving"

    finished = manager.finish_mapping()
    assert finished.success is True
    saved = database.get_selected_map()
    assert saved is not None
    assert saved.name == "一楼大厅"
    assert saved.yaml_path.endswith("20260828_120000/map.yaml")
    assert manager.status()["state"] == "idle"


def test_navigation_uses_selected_map_paths(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))
    manager.start_mapping("仓库", "spark_sam")
    manager.finish_mapping()
    selected = database.get_selected_map()
    assert selected is not None
    initial_pose = WaypointRecord(
        "入口", "map", 1.2, -0.8, 0.0, 0.0, 0.0, 0.0, 1.0,
        map_id=selected.id,
    )
    database.save_waypoint(initial_pose)

    result = manager.start_navigation(
        selected.id, "3dloc", initial_waypoint="入口", rviz=True
    )

    assert result.success is True
    assert adapter.navigation_calls == [
        (selected.yaml_path, selected.pcd_path, "3dloc", initial_pose, True)
    ]
    assert manager.status()["navigation_map_id"] == selected.id


def test_2d_navigation_uses_selected_initial_waypoint(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))
    manager.start_mapping("仓库", "fast_lio")
    manager.finish_mapping()
    selected = database.get_selected_map()
    assert selected is not None
    initial_pose = WaypointRecord(
        "入口", "map", 1.2, -0.8, 0.0, 0.0, 0.0, 0.0, 1.0,
        map_id=selected.id,
    )
    database.save_waypoint(initial_pose)

    result = manager.start_navigation(
        selected.id, "amcl", initial_waypoint="入口", rviz=True
    )

    assert result.success is True
    assert adapter.navigation_calls == [
        (selected.yaml_path, selected.pcd_path, "amcl", initial_pose, True)
    ]


def test_editor_atomically_overwrites_selected_yaml_and_pgm(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))
    manager.start_mapping("编辑地图", "fast_lio")
    manager.finish_mapping()
    selected = database.get_selected_map()
    assert selected is not None
    pcd_before = Path(selected.pcd_path).read_bytes()

    edited_yaml = (
        b"image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
        b"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    edited_pgm = b"P5\n2 2\n255\n\xff\xff\x00\x00"
    record = manager.replace_edited_map(selected.id, edited_yaml, edited_pgm)

    assert Path(record.pgm_path).read_bytes() == edited_pgm
    assert "image: map.pgm" in Path(record.yaml_path).read_text(encoding="utf-8")
    assert Path(record.pcd_path).read_bytes() == pcd_before
    assert not list(Path(record.session_dir).glob("*.tmp"))


def test_editor_rejects_dimension_change_and_running_navigation(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))
    manager.start_mapping("编辑保护", "fast_lio")
    manager.finish_mapping()
    selected = database.get_selected_map()
    assert selected is not None
    yaml_bytes = Path(selected.yaml_path).read_bytes()

    try:
        manager.replace_edited_map(
            selected.id, yaml_bytes, b"P5\n3 1\n255\n\x00\x00\x00"
        )
    except ValueError as exc:
        assert "尺寸" in str(exc)
    else:
        raise AssertionError("dimension change should be rejected")

    adapter.navigation = True
    try:
        manager.replace_edited_map(
            selected.id, yaml_bytes, Path(selected.pgm_path).read_bytes()
        )
    except RuntimeError as exc:
        assert "停止导航" in str(exc)
    else:
        raise AssertionError("editing during navigation should be rejected")


def test_map_manager_rejects_paths_outside_map_root(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))
    record = database.create_map("非法路径", "fast_lio")
    outside = tmp_path / "outside"
    outside.mkdir()
    for name in ("map.yaml", "map.pgm", "cloud.pcd", "metadata.yaml"):
        (outside / name).write_text("map", encoding="utf-8")
    database.complete_map(
        record.id,
        {
            "session_dir": str(outside),
            "yaml_path": str(outside / "map.yaml"),
            "pgm_path": str(outside / "map.pgm"),
            "pcd_path": str(outside / "cloud.pcd"),
            "metadata_path": str(outside / "metadata.yaml"),
        },
    )

    try:
        manager.start_navigation(record.id, "3dloc")
    except ValueError as exc:
        assert "允许的目录" in str(exc)
    else:
        raise AssertionError("outside path should be rejected")


def test_delete_map_removes_archive_and_bound_waypoints(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))

    manager.start_mapping("当前地图", "fast_lio")
    manager.finish_mapping()
    current = database.get_selected_map()
    assert current is not None
    archive = adapter.map_root / "待删除地图"
    archive.mkdir(parents=True)
    files = {
        "session_dir": archive,
        "yaml_path": archive / "map.yaml",
        "pgm_path": archive / "map.pgm",
        "pcd_path": archive / "cloud.pcd",
        "metadata_path": archive / "metadata.yaml",
    }
    for path in files.values():
        if path != archive:
            path.write_bytes(b"map")
    removable = database.complete_map(
        database.create_map("待删除地图", "spark_sam").id,
        {key: str(path) for key, path in files.items()},
    )
    database.save_waypoint(
        WaypointRecord("删除点位", "map", 0, 0, 0, 0, 0, 0, 1, map_id=removable.id)
    )
    archive = Path(removable.session_dir)
    assert archive.is_dir()

    manager.delete_map(removable.id)

    assert not archive.exists()
    assert all(item.id != removable.id for item in database.list_maps())
    assert database.list_waypoints(removable.id) == []


def test_delete_map_rejects_selected_map(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    adapter = FakeAdapter(tmp_path / "maps")
    manager = MapManager(adapter, database, str(adapter.map_root))
    manager.start_mapping("当前地图", "fast_lio")
    manager.finish_mapping()
    selected = database.get_selected_map()
    assert selected is not None

    try:
        manager.delete_map(selected.id)
    except RuntimeError as exc:
        assert "当前地图" in str(exc)
    else:
        raise AssertionError("selected map should not be deleted")
