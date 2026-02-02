#!/usr/bin/env python3
"""
麦克风测试 Launch 文件
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
        description='音频输入设备名称'
    )
    
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='采样率 (Hz)'
    )
    
    channels_arg = DeclareLaunchArgument(
        'channels',
        default_value='1',
        description='音频通道数'
    )
    
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='5',
        description='录音时长 (秒)'
    )

    # 创建节点
    mic_test_node = Node(
        package='device_test',
        executable='mic_test',
        name='mic_test_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'channels': LaunchConfiguration('channels'),
            'duration': LaunchConfiguration('duration'),
        }]
    )

    return LaunchDescription([
        device_arg,
        sample_rate_arg,
        channels_arg,
        duration_arg,
        mic_test_node,
    ])
