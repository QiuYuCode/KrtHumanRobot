from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_db_arg = DeclareLaunchArgument("robot_db", default_value="~/maps/krt_robot.db")
    legacy_groups_file_arg = DeclareLaunchArgument("legacy_groups_file", default_value="")
    left_namespace_arg = DeclareLaunchArgument(
        "left_namespace", default_value="/left", description="Left arm namespace"
    )
    right_namespace_arg = DeclareLaunchArgument(
        "right_namespace", default_value="/right", description="Right arm namespace"
    )
    stream_step_interval_arg = DeclareLaunchArgument(
        "stream_step_interval_sec", default_value="0.02", description="Streaming replay interval"
    )
    sample_rate_arg = DeclareLaunchArgument(
        "sample_rate_hz", default_value="50.0", description="Synchronized teach sample rate"
    )
    autostart_arg = DeclareLaunchArgument("autostart", default_value="true")

    runner = Node(
        package="agx_action_group_runner",
        executable="action_group_runner",
        namespace="agx_action_group",
        name="agx_action_group_runner",
        output="screen",
        parameters=[{
                "robot_db": LaunchConfiguration("robot_db"),
                "legacy_groups_file": LaunchConfiguration("legacy_groups_file"),
            "left_namespace": LaunchConfiguration("left_namespace"),
            "right_namespace": LaunchConfiguration("right_namespace"),
            "stream_step_interval_sec": LaunchConfiguration("stream_step_interval_sec"),
            "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
        }],
    )

    teach = Node(
        package="agx_action_group_runner",
        executable="teach_action_group",
        namespace="agx_action_group",
        name="teach_action_group",
        output="screen",
        parameters=[{
                "robot_db": LaunchConfiguration("robot_db"),
                "legacy_groups_file": LaunchConfiguration("legacy_groups_file"),
            "left_namespace": LaunchConfiguration("left_namespace"),
            "right_namespace": LaunchConfiguration("right_namespace"),
            "playback_step_interval_sec": LaunchConfiguration("stream_step_interval_sec"),
            "sample_rate_hz": LaunchConfiguration("sample_rate_hz"),
            "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
        }],
    )

    return LaunchDescription([
        robot_db_arg,
        legacy_groups_file_arg,
        left_namespace_arg,
        right_namespace_arg,
        stream_step_interval_arg,
        sample_rate_arg,
        autostart_arg,
        runner,
        teach,
    ])
