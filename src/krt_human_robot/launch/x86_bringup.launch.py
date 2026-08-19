"""Bring up x86-owned control, navigation, voice, task, and Web stacks."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _navigation(context):
    """Select one navigation launch from the navigation_mode argument."""
    navigation_share = get_package_share_directory("ranger_nav")
    mode = LaunchConfiguration("navigation_mode").perform(context).strip().lower()
    if mode not in {"2d", "3dloc"}:
        raise RuntimeError("navigation_mode must be 2d or 3dloc")
    filename = (
        "navigation_3dloc.launch.py" if mode == "3dloc" else "navigation.launch.py"
    )
    arguments = {
        "map": LaunchConfiguration("map"),
        "rviz": "false",
    }
    if mode == "3dloc":
        arguments["pcd_map_path"] = LaunchConfiguration("pcd_map_path")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", filename)
        ),
        launch_arguments=arguments.items(),
        condition=IfCondition(LaunchConfiguration("enable_navigation")),
    )]


def generate_launch_description() -> LaunchDescription:
    """Return the target x86 launch composition."""
    robot_share = get_package_share_directory("krt_human_robot")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share, "launch", "robot.launch.py")
        ),
        launch_arguments={
            "enable_camera_stack": "false",
            "enable_arm_control_stack": "true",
            "enable_web_console": "true",
            "enable_hands_control_stack": "false",
            "web_host": LaunchConfiguration("web_host"),
            "web_port": LaunchConfiguration("web_port"),
            "web_certfile": LaunchConfiguration("web_certfile"),
            "web_keyfile": LaunchConfiguration("web_keyfile"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value=os.path.expanduser("~/maps/map.yaml")
        ),
        DeclareLaunchArgument(
            "pcd_map_path",
            default_value=os.path.expanduser("~/maps/scans.pcd"),
        ),
        DeclareLaunchArgument(
            "navigation_mode",
            default_value="3dloc",
            description="Navigation mode: 2d or 3dloc",
        ),
        DeclareLaunchArgument("enable_navigation", default_value="false"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8443"),
        DeclareLaunchArgument("web_certfile", default_value=""),
        DeclareLaunchArgument("web_keyfile", default_value=""),
        robot,
        OpaqueFunction(function=_navigation),
    ])
