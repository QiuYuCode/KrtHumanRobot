#!/usr/bin/env python3
"""
扬声器测试 Launch 文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='default',
        description='音频输出设备名称'
    )
    
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='2.0',
        description='测试音频时长 (秒)'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='440.0',
        description='测试音频频率 (Hz)'
    )
    
    volume_arg = DeclareLaunchArgument(
        'volume',
        default_value='0.5',
        description='音量 (0.0-1.0)'
    )

    # 创建节点
    speaker_test_node = Node(
        package='device_test',
        executable='speaker_test',
        name='speaker_test_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'test_duration': LaunchConfiguration('test_duration'),
            'frequency': LaunchConfiguration('frequency'),
            'volume': LaunchConfiguration('volume'),
        }]
    )

    return LaunchDescription([
        device_arg,
        test_duration_arg,
        frequency_arg,
        volume_arg,
        speaker_test_node,
    ])
