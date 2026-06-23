"""启动语音子系统。

用法:
    ros2 launch voice_assistant voice_assistant.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("voice_assistant")
    default_config = os.path.join(pkg_share, "config", "voice_assistant.yaml")

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="voice_assistant.yaml 路径。",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "voice_stack.launch.py")
            ),
            launch_arguments={"config_file": config_file}.items(),
        ),
    ])
