from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("robot_db", default_value="~/maps/krt_robot.db"),
        DeclareLaunchArgument("media_dir", default_value="~/music"),
        Node(
            package="krt_task",
            executable="routine_runner",
            name="routine_runner",
            output="screen",
            parameters=[{
                "robot_db": LaunchConfiguration("robot_db"),
                "media_dir": LaunchConfiguration("media_dir"),
            }],
        ),
    ])
