#!/usr/bin/env python3
"""
所有设备测试 Launch 文件
同时启动摄像头、麦克风、扬声器测试节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 摄像头参数
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='摄像头设备ID'
    )

    # 创建节点列表
    nodes = []

    # 设备列表节点 - 先运行一次显示所有设备
    device_list_node = Node(
        package='device_test',
        executable='device_list',
        name='device_list_node',
        output='screen',
    )
    nodes.append(device_list_node)

    # 摄像头测试节点
    camera_test_node = Node(
        package='device_test',
        executable='camera_test',
        name='camera_test_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('camera_device'),
            'width': 640,
            'height': 480,
            'fps': 30.0,
            'show_preview': True,
        }]
    )
    nodes.append(camera_test_node)

    # 麦克风测试节点
    mic_test_node = Node(
        package='device_test',
        executable='mic_test',
        name='mic_test_node',
        output='screen',
        parameters=[{
            'device': 'default',
            'sample_rate': 16000,
            'channels': 1,
            'duration': 5,
        }]
    )
    nodes.append(mic_test_node)

    # 扬声器测试节点
    speaker_test_node = Node(
        package='device_test',
        executable='speaker_test',
        name='speaker_test_node',
        output='screen',
        parameters=[{
            'device': 'default',
            'test_duration': 2.0,
            'frequency': 440.0,
            'volume': 0.5,
        }]
    )
    nodes.append(speaker_test_node)

    return LaunchDescription([
        camera_device_arg,
    ] + nodes)
