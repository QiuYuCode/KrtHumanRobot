from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    groups_file_arg = DeclareLaunchArgument(
        "groups_file",
        description="Absolute path to action groups yaml file",
    )
    left_namespace_arg = DeclareLaunchArgument(
        "left_namespace", default_value="/left", description="Left arm namespace"
    )
    right_namespace_arg = DeclareLaunchArgument(
        "right_namespace", default_value="/right", description="Right arm namespace"
    )

    runner = Node(
        package="agx_action_group_runner",
        executable="action_group_runner",
        namespace="agx_action_group",
        name="agx_action_group_runner",
        output="screen",
        parameters=[{
            "groups_file": LaunchConfiguration("groups_file"),
            "left_namespace": LaunchConfiguration("left_namespace"),
            "right_namespace": LaunchConfiguration("right_namespace"),
        }],
    )

    teach = Node(
        package="agx_action_group_runner",
        executable="teach_action_group",
        namespace="agx_action_group",
        name="teach_action_group",
        output="screen",
        parameters=[{
            "groups_file": LaunchConfiguration("groups_file"),
            "left_namespace": LaunchConfiguration("left_namespace"),
            "right_namespace": LaunchConfiguration("right_namespace"),
        }],
    )

    return LaunchDescription([groups_file_arg, left_namespace_arg, right_namespace_arg, runner, teach])
