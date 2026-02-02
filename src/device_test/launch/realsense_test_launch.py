#!/usr/bin/env python3
"""
Intel RealSense 摄像头测试 Launch 文件
通过 OpenCV 直接访问 RealSense 设备进行简单测试

设备映射 (Intel RealSense D435 via V4L2):
  /dev/video2: Depth (Z16)
  /dev/video4: Infrared (GREY)
  /dev/video6: RGB (YUYV)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数 - D435 V4L2 设备映射
    rgb_device_arg = DeclareLaunchArgument(
        'rgb_device_id',
        default_value='6',
        description='RGB流设备ID (YUYV)'
    )

    depth_device_arg = DeclareLaunchArgument(
        'depth_device_id',
        default_value='2',
        description='深度流设备ID (Z16)'
    )

    ir_device_arg = DeclareLaunchArgument(
        'ir_device_id',
        default_value='4',
        description='红外流设备ID (GREY)'
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

    enable_rgb_arg = DeclareLaunchArgument(
        'enable_rgb',
        default_value='true',
        description='启用RGB流'
    )

    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='启用深度流'
    )

    enable_ir_arg = DeclareLaunchArgument(
        'enable_ir',
        default_value='false',
        description='启用红外流'
    )

    # 创建节点
    realsense_node = Node(
        package='device_test',
        executable='realsense_test',
        name='realsense_test_node',
        output='screen',
        parameters=[{
            'rgb_device_id': LaunchConfiguration('rgb_device_id'),
            'depth_device_id': LaunchConfiguration('depth_device_id'),
            'ir_device_id': LaunchConfiguration('ir_device_id'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'show_preview': LaunchConfiguration('show_preview'),
            'enable_rgb': LaunchConfiguration('enable_rgb'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_ir': LaunchConfiguration('enable_ir'),
        }]
    )

    return LaunchDescription([
        rgb_device_arg,
        depth_device_arg,
        ir_device_arg,
        width_arg,
        height_arg,
        fps_arg,
        show_preview_arg,
        enable_rgb_arg,
        enable_depth_arg,
        enable_ir_arg,
        realsense_node,
    ])
