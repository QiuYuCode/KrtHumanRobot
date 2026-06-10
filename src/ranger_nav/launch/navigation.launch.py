"""导航 launch：MID360 + FAST-LIO 里程计 + AMCL 定位 + Nav2。

用法:
    ros2 launch ranger_nav navigation.launch.py map:=$HOME/maps/map.yaml

启动后在 RViz 中用 "2D Pose Estimate" 设置初始位姿，
再用 "Nav2 Goal" 发送导航目标。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 点云转激光的切片高度（相对地面，米）。
# 地面不平、地面毛刺被误判为障碍时调大 SCAN_MIN_HEIGHT（如 0.20）
SCAN_MIN_HEIGHT = 0.15
SCAN_MAX_HEIGHT = 1.2


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    livox_share = get_package_share_directory('livox_ros_driver2')
    agx_share = get_package_share_directory('agx_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    ranger_nav_share = get_package_share_directory('ranger_nav')

    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/map.yaml'),
        description='2D 栅格地图 yaml 路径（由 pcd2pgm 生成）')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='是否启动 RViz')

    # MID360 驱动
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py'))
    )

    # FAST-LIO 作为激光里程计（关闭 PCD 累积，避免长时间运行内存增长）
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            os.path.join(fast_lio_share, 'config', 'mid360.yaml'),
            {'pcd_save.pcd_save_en': False},
        ],
        output='screen',
    )

    # Ranger 底盘
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

    # TF 链: map -> odom (AMCL) -> camera_init -> body -> base_footprint
    tf_odom_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
    )
    # URDF: body -> base_footprint -> base_link（雷达偏移在 urdf 中配置）
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

    # FAST-LIO 配准后的机体系点云 -> 2D 激光扫描，供 AMCL 与代价地图使用
    cloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            'min_height': SCAN_MIN_HEIGHT,
            'max_height': SCAN_MAX_HEIGHT,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0058,  # ~0.33 度
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 30.0,
            'use_inf': True,
        }],
    )

    # Nav2（map_server + AMCL + 规划/控制）
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': os.path.join(
                ranger_nav_share, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            nav2_share, 'rviz', 'nav2_default_view.rviz')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_map,
        declare_rviz,
        livox_driver,
        fast_lio_node,
        chassis_node,
        tf_odom_camera_init,
        robot_state_publisher,
        cloud_to_scan,
        nav2_bringup,
        rviz_node,
    ])
