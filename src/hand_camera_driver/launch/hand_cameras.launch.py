"""Launch left and right hand USB cameras."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(side: str, device: str, topic: str, frame_id: str) -> Node:
    return Node(
        package="hand_camera_driver",
        executable="usb_camera_node",
        name=f"{side}_hand_camera",
        output="screen",
        condition=IfCondition(LaunchConfiguration(f"enable_{side}")),
        parameters=[{
            "device": LaunchConfiguration(device),
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fps": LaunchConfiguration("fps"),
            "topic": topic,
            "frame_id": frame_id,
        }],
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "left_device", default_value="/dev/camera_left"
        ),
        DeclareLaunchArgument(
            "right_device", default_value="/dev/camera_right"
        ),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30.0"),
        _camera_node(
            "left",
            "left_device",
            "/left_gripper/image_raw",
            "left_hand_camera_frame",
        ),
        _camera_node(
            "right",
            "right_device",
            "/right_gripper/image_raw",
            "right_hand_camera_frame",
        ),
    ])
