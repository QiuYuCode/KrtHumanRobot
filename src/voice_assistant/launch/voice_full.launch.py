"""一键启动 voice_stack + voice_assistant（本地/云端由 voice_assistant.yaml 控制）。"""

from __future__ import annotations

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _start_after_cleanup(context, *args, **kwargs):
    """阻塞清理残留语音进程后，再启动新的 voice_stack / voice_assistant。"""
    pkg_share = get_package_share_directory("voice_assistant")
    kill_script = os.path.join(pkg_share, "scripts", "kill_stale_voice_stack.sh")
    run_script = os.path.join(pkg_share, "scripts", "run_voice_node.sh")

    subprocess.run(["bash", kill_script], check=True)

    config_file = LaunchConfiguration("config_file")
    tick_interval_ms = LaunchConfiguration("tick_interval_ms")
    enable_monitor = LaunchConfiguration("enable_monitor")
    assistant_start_delay_s = LaunchConfiguration("assistant_start_delay_s")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "voice_stack.launch.py")
            ),
            launch_arguments={"config_file": config_file}.items(),
        ),
        TimerAction(
            period=assistant_start_delay_s,
            actions=[
                ExecuteProcess(
                    cmd=[run_script],
                    output="screen",
                    additional_env={
                        "VOICE_ASSISTANT_CONFIG": config_file,
                        "VOICE_TICK_INTERVAL_MS": tick_interval_ms,
                        "VOICE_ENABLE_MONITOR": enable_monitor,
                    },
                ),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("voice_assistant")
    default_config = os.path.join(pkg_share, "config", "voice_assistant.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("tick_interval_ms", default_value="100"),
        DeclareLaunchArgument("enable_monitor", default_value="true"),
        DeclareLaunchArgument(
            "assistant_start_delay_s",
            default_value="8.0",
            description="voice_stack 启动后延迟多少秒再启动 voice_assistant",
        ),
        OpaqueFunction(function=_start_after_cleanup),
    ])
