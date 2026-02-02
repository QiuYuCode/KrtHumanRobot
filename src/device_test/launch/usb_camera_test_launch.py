#!/usr/bin/env python3
"""
USB 摄像头测试 Launch 文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='USB摄像头设备ID (/dev/videoX 中的 X)'
    )

    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='图像宽度'
    )

    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='图像高度'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30.0',
        description='帧率'
    )

    show_preview_arg = DeclareLaunchArgument(
        'show_preview',
        default_value='true',
        description='是否显示预览窗口'
    )

    # 创建节点
    usb_camera_node = Node(
        package='device_test',
        executable='usb_camera_test',
        name='usb_camera_test_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'show_preview': LaunchConfiguration('show_preview'),
        }]
    )

    return LaunchDescription([
        device_id_arg,
        width_arg,
        height_arg,
        fps_arg,
        show_preview_arg,
        usb_camera_node,
    ])
