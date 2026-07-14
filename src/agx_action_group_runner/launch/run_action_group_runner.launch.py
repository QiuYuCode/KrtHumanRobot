from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_db_arg = DeclareLaunchArgument("robot_db", default_value="~/maps/krt_robot.db")
    legacy_groups_file_arg = DeclareLaunchArgument("legacy_groups_file", default_value="")
    left_namespace_arg = DeclareLaunchArgument(
        "left_namespace", default_value="/left_arm", description="Left arm namespace"
    )
    right_namespace_arg = DeclareLaunchArgument(
        "right_namespace", default_value="/right_arm", description="Right arm namespace"
    )

    node = Node(
        package="agx_action_group_runner",
        executable="action_group_runner",
        name="agx_action_group_runner",
        output="screen",
        parameters=[
            {
                "robot_db": LaunchConfiguration("robot_db"),
                "legacy_groups_file": LaunchConfiguration("legacy_groups_file"),
                "left_namespace": LaunchConfiguration("left_namespace"),
                "right_namespace": LaunchConfiguration("right_namespace"),
            }
        ],
    )

    return LaunchDescription(
        [robot_db_arg, legacy_groups_file_arg, left_namespace_arg, right_namespace_arg, node]
    )
