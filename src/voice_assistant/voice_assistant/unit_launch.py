from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_ALL_UNITS = (
    "capture",
    "kws",
    "vad",
    "asr",
    "tts",
    "playback",
    "media",
    "volume",
)


def make_unit_launch(enabled_units: set[str]) -> LaunchDescription:
    """Create a minimal launch by selecting nodes from voice_stack.launch.py."""
    unknown = enabled_units.difference(_ALL_UNITS)
    if unknown:
        raise ValueError(f"未知语音单元: {sorted(unknown)}")

    package_share = get_package_share_directory("voice_assistant")
    default_config = os.path.join(package_share, "config", "voice_assistant.yaml")
    launch_arguments = {
        "config_file": LaunchConfiguration("config_file"),
        **{
            f"enable_{unit}": "true" if unit in enabled_units else "false"
            for unit in _ALL_UNITS
        },
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="语音单元测试使用的 voice_assistant.yaml 路径",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "voice_stack.launch.py")
                ),
                launch_arguments=launch_arguments.items(),
            ),
        ]
    )
