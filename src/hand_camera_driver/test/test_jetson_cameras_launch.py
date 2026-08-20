"""Launch contract for the Jetson camera stack."""

import importlib.util
from pathlib import Path

from launch_ros.actions import Node


def test_jetson_launch_uses_standard_compressed_transport(
    monkeypatch, tmp_path
):
    """The head image relay uses the ROS-supported transport plugin."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    launch_file = (
        Path(__file__).parents[1] / "launch" / "jetson_cameras.launch.py"
    )
    spec = importlib.util.spec_from_file_location("jetson_cameras", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    nodes = [
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, Node)
    ]

    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ("image_transport", "republish")
    ]
