#!/usr/bin/env python3
"""
多摄像头测试 Launch 文件
同时测试多个USB摄像头
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    # USB Camera (video0)
    usb_camera_node = Node(
        package='device_test',
        executable='usb_camera_test',
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
            ('usb_camera/image_raw', 'image_raw'),
        ]
    )
    nodes.append(usb_camera_node)

    return LaunchDescription(nodes)
