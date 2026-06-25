"""Launch the krtHumanRobot core behavior tree entry."""

from __future__ import annotations

import os

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


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    pkg_share = get_package_share_directory("krt_human_robot")
    voice_share = get_package_share_directory("voice_assistant")
    run_script = os.path.join(pkg_share, "scripts", "run_krt_human_robot_node.sh")

    config_file = LaunchConfiguration("config_file").perform(context)
    voice_config_file = LaunchConfiguration("voice_config_file").perform(context)
    tick_interval_ms = LaunchConfiguration("tick_interval_ms").perform(context)
    enable_monitor = LaunchConfiguration("enable_monitor").perform(context)
    snapshot_period_s = LaunchConfiguration("snapshot_period_s").perform(context)
    snapshot_blackboard_data = LaunchConfiguration(
        "snapshot_blackboard_data"
    ).perform(context)
    snapshot_blackboard_activity = LaunchConfiguration(
        "snapshot_blackboard_activity"
    ).perform(context)
    core_start_delay_s = LaunchConfiguration("core_start_delay_s")
    enable_voice_stack = (
        LaunchConfiguration("enable_voice_stack").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    enable_camera_stack = (
        LaunchConfiguration("enable_camera_stack").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )

    core_node = ExecuteProcess(
        cmd=["bash", run_script],
        output="screen",
        additional_env={
            "KRT_HUMAN_ROBOT_CONFIG": config_file,
            "KRT_HUMAN_ROBOT_TICK_INTERVAL_MS": tick_interval_ms,
            "KRT_HUMAN_ROBOT_ENABLE_MONITOR": enable_monitor,
            "KRT_HUMAN_ROBOT_SNAPSHOT_PERIOD_S": snapshot_period_s,
            "KRT_HUMAN_ROBOT_SNAPSHOT_BLACKBOARD_DATA": snapshot_blackboard_data,
            "KRT_HUMAN_ROBOT_SNAPSHOT_BLACKBOARD_ACTIVITY": (
                snapshot_blackboard_activity
            ),
        },
    )

    actions = []

    if enable_camera_stack:
        realsense_share = get_package_share_directory("realsense2_camera")
        hand_camera_share = get_package_share_directory("hand_camera_driver")
        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_share, "launch", "rs_launch.py")
                ),
                launch_arguments={
                    "config_file": "''",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hand_camera_share, "launch", "hand_cameras.launch.py"
                    )
                ),
            ),
        ])

    if enable_voice_stack:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(voice_share, "launch", "voice_stack.launch.py")
            ),
            launch_arguments={"config_file": voice_config_file}.items(),
        ))

    if enable_voice_stack or enable_camera_stack:
        actions.append(TimerAction(period=core_start_delay_s, actions=[core_node]))
    else:
        actions.append(core_node)

    return actions


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("krt_human_robot")
    voice_share = get_package_share_directory("voice_assistant")
    default_config = os.path.join(pkg_share, "config", "krt_human_robot.yaml")
    default_voice_config = os.path.join(
        voice_share, "config", "voice_assistant.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("voice_config_file", default_value=default_voice_config),
        DeclareLaunchArgument("enable_voice_stack", default_value="true"),
        DeclareLaunchArgument("enable_camera_stack", default_value="true"),
        DeclareLaunchArgument("core_start_delay_s", default_value="8.0"),
        DeclareLaunchArgument("tick_interval_ms", default_value="100"),
        DeclareLaunchArgument("enable_monitor", default_value="true"),
        DeclareLaunchArgument("snapshot_period_s", default_value="2.0"),
        DeclareLaunchArgument("snapshot_blackboard_data", default_value="true"),
        DeclareLaunchArgument("snapshot_blackboard_activity", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
