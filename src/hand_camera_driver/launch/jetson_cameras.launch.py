"""Bring up the D435 and both hand cameras on the Jetson."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return the camera-only Jetson launch composition."""
    realsense_share = get_package_share_directory("realsense2_camera")
    hand_camera_share = get_package_share_directory("hand_camera_driver")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "enable_sync": "true",
            "rgb_camera.color_profile": "640x480x15",
        }.items(),
    )
    hand_cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                hand_camera_share,
                "launch",
                "hand_cameras.launch.py",
            )
        ),
        launch_arguments={
            "fps": "15.0",
            "jpeg_quality": "70",
        }.items(),
    )
    head_color_compressor = Node(
        package="hand_camera_driver",
        executable="compressed_image_relay",
        name="head_color_compressor",
        parameters=[{
            "input_topic": "/camera/camera/color/image_raw",
            "output_topic": "/camera/camera/color/image_jpeg",
            "jpeg_quality": 70,
        }],
    )

    return LaunchDescription([realsense, hand_cameras, head_color_compressor])
