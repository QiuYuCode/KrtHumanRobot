"""Launch the single-worker HTTPS robot Web console."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    run_script = os.path.join(
        get_package_share_directory("krt_human_robot"),
        "scripts", "run_krt_web_console.sh",
    )
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8443"),
        DeclareLaunchArgument("certfile"),
        DeclareLaunchArgument("keyfile"),
        DeclareLaunchArgument("robot_db", default_value="~/maps/krt_robot.db"),
        DeclareLaunchArgument("web_db", default_value="~/.local/share/krt_human_robot/web.db"),
        DeclareLaunchArgument("media_dir", default_value="~/music"),
        ExecuteProcess(
            cmd=[
                "bash", run_script, "krt_human_robot.web_app:create_app()",
                "--bind", [LaunchConfiguration("host"), ":", LaunchConfiguration("port")],
                "--workers", "1", "--worker-class", "gthread", "--threads", "4",
                "--timeout", "120", "--graceful-timeout", "30",
                "--certfile", LaunchConfiguration("certfile"),
                "--keyfile", LaunchConfiguration("keyfile"),
            ],
            additional_env={
                "KRT_ROBOT_DB": LaunchConfiguration("robot_db"),
                "KRT_WEB_DB": LaunchConfiguration("web_db"),
                "KRT_MEDIA_DIR": LaunchConfiguration("media_dir"),
            },
            output="screen",
        ),
    ])
