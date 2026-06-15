"""SPARK-FAST-LIO 建图基础栈：MID360 + spark_fast_lio + Ranger 底盘 + URDF。

供 mapping_sam.launch.py include，不与 fast_lio 混用。
TF: spark 发布 odom→body，URDF 发布 body→base_footprint→base_link。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    livox_share = get_package_share_directory('livox_ros_driver2')
    agx_share = get_package_share_directory('agx_bringup')
    ranger_nav_share = get_package_share_directory('ranger_nav')

    use_rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false', description='是否启动 SPARK-FAST-LIO RViz')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {'xfer_format': 1},
            {'multi_topic': 0},
            {'data_src': 0},
            {'publish_freq': 10.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_frame'},
            {'user_config_path': os.path.join(
                livox_share, 'config', 'MID360_config.json')},
            {'cmdline_input_bd_code': 'livox0000000001'},
        ],
    )

    spark_lio_node = Node(
        package='spark_fast_lio',
        executable='spark_lio_mapping',
        name='lio_mapping',
        output='screen',
        parameters=[
            os.path.join(ranger_nav_share, 'config', 'spark_fast_lio_mid360.yaml'),
            {
                'common.map_frame': 'odom',
                'common.lidar_frame': 'body',
                'common.imu_frame': 'body',
                'common.base_frame': '',
                # spark 内部坐标系名：imu/lidar/base，不是 TF 名 body
                'common.visualization_frame': 'imu',
                'gravity_alignment.enable_gravity_alignment': False,
            },
        ],
        remappings=[
            ('lidar', '/livox/lidar'),
            ('imu', '/livox/imu'),
        ],
    )

    chassis_node = Node(
        package='agx_bringup',
        executable='agx_bringup_node',
        name='agx_bringup_node',
        output='screen',
        parameters=[os.path.join(agx_share, 'config', 'agx_bringup.yaml')],
        remappings=[
            ('/sub_cmd_vel', '/cmd_vel'),
            ('/wheel/odom', '/odom'),
        ],
    )

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(
            ranger_nav_share, 'urdf', 'ranger_mini.urdf.xacro')]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    return LaunchDescription([
        declare_rviz,
        livox_driver,
        spark_lio_node,
        chassis_node,
        robot_state_publisher,
    ])
