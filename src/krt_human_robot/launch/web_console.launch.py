"""Launch the single-worker HTTPS robot Web console."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    package_share = get_package_share_directory("krt_human_robot")
    run_script = os.path.join(
        package_share,
        "scripts", "run_krt_web_console.sh",
    )
    certfile = LaunchConfiguration("certfile").perform(context).strip()
    keyfile = LaunchConfiguration("keyfile").perform(context).strip()
    cmd = [
        "bash", run_script, "krt_human_robot.web_app:create_app()",
        "--bind", [LaunchConfiguration("host"), ":", LaunchConfiguration("port")],
        "--workers", "1", "--worker-class", "gthread", "--threads", "4",
        "--timeout", "120", "--graceful-timeout", "30",
    ]
    if certfile and keyfile:
        cmd.extend(["--certfile", certfile, "--keyfile", keyfile])
    return [ExecuteProcess(
        cmd=cmd,
        additional_env={
            "KRT_ROBOT_DB": LaunchConfiguration("robot_db"),
            "KRT_WEB_DB": LaunchConfiguration("web_db"),
            "KRT_MEDIA_DIR": LaunchConfiguration("media_dir"),
            "KRT_HUMAN_ROBOT_CONFIG": LaunchConfiguration("config_file"),
        },
        output="screen",
    )]


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("krt_human_robot")
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8443"),
        DeclareLaunchArgument("certfile"),
        DeclareLaunchArgument("keyfile"),
        DeclareLaunchArgument(
            "config_file", default_value=os.path.join(
                package_share, "config", "krt_human_robot.yaml"
            )
        ),
        DeclareLaunchArgument("robot_db", default_value="~/maps/krt_robot.db"),
        DeclareLaunchArgument("web_db", default_value="~/.local/share/krt_human_robot/web.db"),
        DeclareLaunchArgument("media_dir", default_value="~/music"),
        OpaqueFunction(function=_launch_setup),
    ])
