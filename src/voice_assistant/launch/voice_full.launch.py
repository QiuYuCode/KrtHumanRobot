"""一键启动 voice_stack + krt_human_robot core."""

from __future__ import annotations

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _start_after_cleanup(context, *args, **kwargs):
    """阻塞清理残留语音进程后，再启动新的 voice_stack / voice_assistant。"""
    pkg_share = get_package_share_directory("voice_assistant")
    kill_script = os.path.join(pkg_share, "scripts", "kill_stale_voice_stack.sh")

    subprocess.run(["bash", kill_script], check=True)

    config_file = LaunchConfiguration("config_file")
    core_config_file = LaunchConfiguration("core_config_file")
    tick_interval_ms = LaunchConfiguration("tick_interval_ms")
    enable_monitor = LaunchConfiguration("enable_monitor")
    assistant_start_delay_s = LaunchConfiguration("assistant_start_delay_s")
    enable_robot_entry = (
        LaunchConfiguration("enable_robot_entry").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )

    if not enable_robot_entry:
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "voice_stack.launch.py")
                ),
                launch_arguments={"config_file": config_file}.items(),
            ),
        ]

    core_share = get_package_share_directory("krt_human_robot")
    core_config = core_config_file.perform(context).strip() or os.path.join(
        core_share, "config", "krt_human_robot.yaml"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(core_share, "launch", "robot.launch.py")
            ),
            launch_arguments={
                "config_file": core_config,
                "voice_config_file": config_file,
                "enable_voice_stack": "true",
                "core_start_delay_s": assistant_start_delay_s,
                "tick_interval_ms": tick_interval_ms,
                "enable_monitor": enable_monitor,
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("voice_assistant")
    default_config = os.path.join(pkg_share, "config", "voice_assistant.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("core_config_file", default_value=""),
        DeclareLaunchArgument("tick_interval_ms", default_value="100"),
        DeclareLaunchArgument("enable_monitor", default_value="true"),
        DeclareLaunchArgument("enable_robot_entry", default_value="true"),
        DeclareLaunchArgument(
            "assistant_start_delay_s",
            default_value="8.0",
            description="voice_stack 启动后延迟多少秒再启动 krt_human_robot",
        ),
        OpaqueFunction(function=_start_after_cleanup),
    ])
