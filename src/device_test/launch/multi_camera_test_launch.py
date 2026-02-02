#!/usr/bin/env python3
"""
多摄像头测试 Launch 文件
同时测试多个USB摄像头
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    # USB Camera (video0)
    usb_camera_node = Node(
        package='device_test',
        executable='camera_test',
        name='usb_camera_test',
        output='screen',
        namespace='usb_camera',
        parameters=[{
            'device_id': 0,
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'show_preview': True,
        }],
        remappings=[
            ('camera/image_raw', 'image_raw'),
        ]
    )
    nodes.append(usb_camera_node)

    # RealSense RGB (video2) - 如果需要测试RealSense的RGB流
    # 注意: 通常应该使用realsense-ros驱动来获取RealSense数据
    # realsense_rgb_node = Node(
    #     package='device_test',
    #     executable='camera_test',
    #     name='realsense_rgb_test',
    #     output='screen',
    #     namespace='realsense',
    #     parameters=[{
    #         'device_id': 2,
    #         'width': 640,
    #         'height': 480,
    #         'fps': 30.0,
    #         'show_preview': True,
    #     }],
    #     remappings=[
    #         ('camera/image_raw', 'image_raw'),
    #     ]
    # )
    # nodes.append(realsense_rgb_node)

    return LaunchDescription(nodes)
