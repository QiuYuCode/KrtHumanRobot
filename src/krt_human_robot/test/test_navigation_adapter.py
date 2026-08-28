from subprocess import CompletedProcess
from types import SimpleNamespace

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


def test_navigation_uses_explicit_map_paths_without_changing_cli_launch(tmp_path):
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
        "ros2", "launch", "ranger_nav", "navigation_3dloc.launch.py",
        f"map:={yaml_path}", f"pcd_map_path:={pcd_path}", "rviz:=true",
    ]
