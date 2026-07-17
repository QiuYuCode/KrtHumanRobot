"""Launch the standalone vision service in the voice uv environment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("krt_human_robot")
    script = os.path.join(share, "scripts", "run_krt_human_robot_node.sh")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=""),
        ExecuteProcess(
            cmd=["bash", script],
            additional_env={
                "KRT_HUMAN_ROBOT_MODULE": "krt_human_robot.vision_service_node",
                "KRT_HUMAN_ROBOT_CONFIG": LaunchConfiguration("config_file"),
            },
            output="screen",
        ),
    ])
