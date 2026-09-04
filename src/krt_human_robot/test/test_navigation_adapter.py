from subprocess import CompletedProcess
from types import SimpleNamespace

import pytest
from krt_task.robot_db import WaypointRecord

from krt_human_robot.adapters.navigation import RangerNavAdapter


class FakeProcess:
    def __init__(self, pid=123):
        self.pid = pid
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        del timeout
        self.returncode = 0

    def terminate(self):
        self.returncode = 0

    def kill(self):
        self.returncode = -9


def config(tmp_path, **changes):
    navigation = {
        "enabled": True,
        "launch_package": "ranger_nav",
        "mapping_backend": "fast_lio",
        "mapping_launch_fast_lio": "mapping.launch.py",
        "mapping_launch_spark_sam": "mapping_sam.launch.py",
        "navigation_mode": "3dloc",
        "navigation_launch_2d": "navigation.launch.py",
        "navigation_launch_3d": "navigation_3dloc.launch.py",
        "map_yaml": str(tmp_path / "default.yaml"),
        "pcd_map_path": str(tmp_path / "default.pcd"),
        "navigation_ready_timeout_s": 0,
    }
    navigation.update(changes)
    return SimpleNamespace(adapters={"navigation": navigation})


def test_mapping_launch_keeps_backend_rviz_argument(tmp_path):
    calls = []

    def popen(command, **kwargs):
        calls.append((command, kwargs))
        return FakeProcess()

    adapter = RangerNavAdapter(config(tmp_path), popen=popen)

    result = adapter.start_mapping("fast_lio", rviz=True)

    assert result.success is True
    assert calls[0][0] == [
        "ros2", "launch", "ranger_nav", "mapping.launch.py", "rviz:=true"
    ]


def test_sam_mapping_uses_sam_rviz_argument(tmp_path):
    calls = []
    adapter = RangerNavAdapter(
        config(tmp_path),
        popen=lambda command, **_kwargs: calls.append(command) or FakeProcess(),
    )

    adapter.start_mapping("spark_sam", rviz=True)

    assert calls[0][-1] == "sam_rviz:=true"


def test_sam_mapping_saves_pcd_in_timestamp_directory(tmp_path):
    session = tmp_path / "20260828_120000"
    calls = []

    def run(command, **_kwargs):
        calls.append(command)
        if command[:3] == ["ros2", "topic", "pub"]:
            session.mkdir(parents=True, exist_ok=True)
            (session / "ranger_map.pcd").write_bytes(b"pcd")
        return CompletedProcess(command, 0, "", "")

    adapter = RangerNavAdapter(
        config(
            tmp_path,
            mapping_backend="spark_sam",
            map_archive_dir=str(tmp_path),
            auto_convert_pcd=False,
            mapping_stop_delay_s=0,
        ),
        popen=lambda command, **_kwargs: FakeProcess(),
        run=run,
        sleep=lambda _seconds: None,
    )
    adapter._create_map_session_dir = lambda: session
    adapter.start_mapping("spark_sam")

    result = adapter.save_mapping()

    assert result.success is True
    assert (session / "ranger_map.pcd").is_file()
    assert (session / "cloud.pcd").read_bytes() == b"pcd"
    assert "/ranger/" not in str(result.data["pcd_path"])
    assert calls[0][4] == "/km_sam/save_dir"


def test_navigation_resolves_rviz_environment_when_web_launches_it(tmp_path):
    yaml_path = tmp_path / "dated" / "map.yaml"
    pcd_path = tmp_path / "dated" / "cloud.pcd"
    yaml_path.parent.mkdir()
    yaml_path.write_text("image: map.pgm\n", encoding="utf-8")
    pcd_path.write_bytes(b"pcd")
    calls = []
    adapter = RangerNavAdapter(
        config(tmp_path),
        popen=lambda command, **_kwargs: calls.append(command) or FakeProcess(),
    )

    result = adapter.start_navigation(
        str(yaml_path), str(pcd_path), "3dloc", rviz=True
    )

    assert result.success is True
    assert calls[0] == [
        "/bin/bash", "-lc",
        'source "$KRT_WORKSPACE/deploy/systemd/krt-rviz-env.sh"; exec "$@"',
        "bash", "ros2", "launch", "ranger_nav", "navigation_3dloc.launch.py",
        f"map:={yaml_path}", f"pcd_map_path:={pcd_path}", "rviz:=true",
    ]


def test_3d_navigation_passes_selected_waypoint_and_waits_for_diagnostics(tmp_path):
    yaml_path = tmp_path / "dated" / "map.yaml"
    pcd_path = tmp_path / "dated" / "cloud.pcd"
    yaml_path.parent.mkdir()
    yaml_path.write_text("image: map.pgm\n", encoding="utf-8")
    pcd_path.write_bytes(b"pcd")
    launches = []
    diagnostics = []
    initial_pose = WaypointRecord(
        "入口", "map", 1.5, -2.0, 0.0, 0.0, 0.0, 0.70710678, 0.70710678
    )

    def run(command, **_kwargs):
        diagnostics.append(command)
        return CompletedProcess(command, 0, "TF graph ready", "")

    adapter = RangerNavAdapter(
        config(tmp_path, navigation_ready_timeout_s=5),
        popen=lambda command, **_kwargs: launches.append(command) or FakeProcess(),
        run=run,
        sleep=lambda _seconds: None,
    )

    result = adapter.start_navigation(
        str(yaml_path), str(pcd_path), "3dloc", initial_pose=initial_pose, rviz=False
    )

    assert result.success is True
    assert launches[0][-5:-1] == [
        "rviz:=false",
        "set_initial_pose:=true",
        "initial_pose_x:=1.5",
        "initial_pose_y:=-2.0",
    ]
    assert float(launches[0][-1].removeprefix("initial_pose_yaw:=")) == pytest.approx(1.5707963268)
    assert diagnostics == [["ros2", "run", "ranger_nav", "nav_tf_diagnostics"]]


def test_2d_navigation_passes_selected_waypoint(tmp_path):
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text("image: map.pgm\n", encoding="utf-8")
    launches = []
    initial_pose = WaypointRecord(
        "入口", "map", 1.5, -2.0, 0.0, 0.0, 0.0, 0.70710678, 0.70710678
    )
    adapter = RangerNavAdapter(
        config(tmp_path, navigation_mode="amcl"),
        popen=lambda command, **_kwargs: launches.append(command) or FakeProcess(),
    )

    result = adapter.start_navigation(
        str(yaml_path), mode="amcl", initial_pose=initial_pose, rviz=False
    )

    assert result.success is True
    assert launches[0][:-1] == [
        "ros2", "launch", "ranger_nav", "navigation.launch.py",
        f"map:={yaml_path}", "rviz:=false", "set_initial_pose:=true",
        "initial_pose_x:=1.5", "initial_pose_y:=-2.0",
    ]
    assert float(launches[0][-1].removeprefix("initial_pose_yaw:=")) == pytest.approx(
        1.5707963268
    )


def test_3d_navigation_stops_launch_when_diagnostics_timeout(tmp_path):
    yaml_path = tmp_path / "dated" / "map.yaml"
    pcd_path = tmp_path / "dated" / "cloud.pcd"
    yaml_path.parent.mkdir()
    yaml_path.write_text("image: map.pgm\n", encoding="utf-8")
    pcd_path.write_bytes(b"pcd")
    process = FakeProcess()
    initial_pose = WaypointRecord("入口", "map", 0, 0, 0, 0, 0, 0, 1)

    adapter = RangerNavAdapter(
        config(tmp_path, navigation_ready_timeout_s=1),
        popen=lambda *_args, **_kwargs: process,
        run=lambda command, **_kwargs: CompletedProcess(command, 1, "", "missing TF odom -> base_footprint"),
        sleep=lambda _seconds: None,
    )

    result = adapter.start_navigation(
        str(yaml_path), str(pcd_path), "3dloc", initial_pose=initial_pose, rviz=False
    )

    assert result.success is False
    assert "missing TF odom -> base_footprint" in result.message
    assert process.returncode == 0


def test_start_cruise_rejects_existing_process(tmp_path):
    process = FakeProcess()
    adapter = RangerNavAdapter(config(tmp_path), popen=lambda *_args, **_kwargs: process)
    adapter._cruise_process = process

    result = adapter.start_cruise(["入口"])

    assert result.success is False
    assert "已经在执行" in result.message
