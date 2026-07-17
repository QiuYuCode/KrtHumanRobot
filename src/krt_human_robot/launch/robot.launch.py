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
    hands_share = get_package_share_directory("hands_control")
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
    enable_hands_control_stack = (
        LaunchConfiguration("enable_hands_control_stack")
        .perform(context)
        .strip()
        .lower()
        in {"1", "true", "yes", "on"}
    )
    enable_task_runner = (
        LaunchConfiguration("enable_task_runner").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    enable_web_console = (
        LaunchConfiguration("enable_web_console").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    enable_action_group_stack = (
        LaunchConfiguration("enable_action_group_stack").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    enable_arm_control_stack = (
        LaunchConfiguration("enable_arm_control_stack").perform(context).strip().lower()
        in {"1", "true", "yes", "on"}
    )
    robot_db = LaunchConfiguration("robot_db").perform(context)
    legacy_action_groups_file = LaunchConfiguration("legacy_action_groups_file").perform(context)
    media_dir = LaunchConfiguration("media_dir").perform(context)
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

    if enable_hands_control_stack:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    hands_share, "launch", "hand_control_launch.py"
                )
            ),
            launch_arguments={
                "left_hand_listen": "false",
                "right_hand_listen": "false",
                "left_hand_realtime_response": "false",
                "right_hand_realtime_response": "false",
            }.items(),
        ))

    if enable_task_runner:
        task_share = get_package_share_directory("krt_task")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(task_share, "launch", "routine_runner.launch.py")
            ),
            launch_arguments={"robot_db": robot_db, "media_dir": media_dir}.items(),
        ))

    if enable_action_group_stack:
        action_group_share = get_package_share_directory("agx_action_group_runner")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(action_group_share, "launch", "teach_action_group.launch.py")
            ),
            launch_arguments={
                "robot_db": robot_db,
                "legacy_groups_file": legacy_action_groups_file,
            }.items(),
        ))

    if enable_arm_control_stack:
        arm_share = get_package_share_directory("agx_arm_ctrl")
        arm_launch_path = os.path.join(
            arm_share, "launch", "start_single_agx_arm.launch.py"
        )
        common_args = {
            "arm_type": LaunchConfiguration("arm_type").perform(context),
            "effector_type": LaunchConfiguration("arm_effector_type").perform(context),
            "auto_enable": LaunchConfiguration("arm_auto_enable").perform(context),
            "speed_percent": LaunchConfiguration("arm_speed_percent").perform(context),
        }
        for side in ("left", "right"):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(arm_launch_path),
                launch_arguments={
                    **common_args,
                    "namespace": side,
                    "can_port": LaunchConfiguration(f"{side}_arm_can_port").perform(context),
                }.items(),
            ))

    if enable_web_console:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "web_console.launch.py")
            ),
            launch_arguments={
                "host": LaunchConfiguration("web_host").perform(context),
                "port": LaunchConfiguration("web_port").perform(context),
                "certfile": LaunchConfiguration("web_certfile").perform(context),
                "keyfile": LaunchConfiguration("web_keyfile").perform(context),
                "config_file": config_file,
                "robot_db": robot_db,
                "web_db": LaunchConfiguration("web_db").perform(context),
                "media_dir": media_dir,
            }.items(),
        ))

    if (
        enable_voice_stack
        or enable_camera_stack
        or enable_hands_control_stack
        or enable_task_runner
        or enable_action_group_stack
        or enable_arm_control_stack
    ):
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
        DeclareLaunchArgument("enable_hands_control_stack", default_value="true"),
        DeclareLaunchArgument("enable_camera_stack", default_value="true"),
        DeclareLaunchArgument("enable_task_runner", default_value="true"),
        DeclareLaunchArgument("enable_action_group_stack", default_value="true"),
        DeclareLaunchArgument("enable_arm_control_stack", default_value="false"),
        DeclareLaunchArgument("left_arm_can_port", default_value="can_left"),
        DeclareLaunchArgument("right_arm_can_port", default_value="can_right"),
        DeclareLaunchArgument("arm_type", default_value="nero"),
        DeclareLaunchArgument("arm_effector_type", default_value="none"),
        DeclareLaunchArgument("arm_auto_enable", default_value="true"),
        DeclareLaunchArgument("arm_speed_percent", default_value="30"),
        DeclareLaunchArgument("robot_db", default_value="~/maps/krt_robot.db"),
        DeclareLaunchArgument(
            "legacy_action_groups_file", default_value="~/maps/action_groups.yaml"
        ),
        DeclareLaunchArgument("media_dir", default_value="~/music"),
        DeclareLaunchArgument("enable_web_console", default_value="false"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8443"),
        DeclareLaunchArgument("web_certfile", default_value=""),
        DeclareLaunchArgument("web_keyfile", default_value=""),
        DeclareLaunchArgument(
            "web_db", default_value="~/.local/share/krt_human_robot/web.db"
        ),
        DeclareLaunchArgument("core_start_delay_s", default_value="8.0"),
        DeclareLaunchArgument("tick_interval_ms", default_value="100"),
        DeclareLaunchArgument("enable_monitor", default_value="true"),
        DeclareLaunchArgument("snapshot_period_s", default_value="2.0"),
        DeclareLaunchArgument("snapshot_blackboard_data", default_value="true"),
        DeclareLaunchArgument("snapshot_blackboard_activity", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
