"""建图 launch：MID360 雷达 + FAST-LIO + Ranger 底盘 + 静态 TF。

建图流程：
1. ros2 launch ranger_nav mapping.launch.py
2. 另开终端遥控: ros2 run teleop_twist_keyboard teleop_twist_keyboard
   (底盘订阅 /cmd_vel)
3. 建图完成后保存:
   ros2 service call /map_save std_srvs/srv/Trigger
   或直接 Ctrl+C 退出, FAST-LIO 会把累计点云存到
   src/FAST_LIO_ROS2/PCD/scans.pcd
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 雷达在底盘 base_footprint 坐标系下的安装位置（米），按实际安装测量修改
LIDAR_X = 0.40
LIDAR_Y = 0.0
LIDAR_Z = 0.40


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    livox_share = get_package_share_directory('livox_ros_driver2')
    agx_share = get_package_share_directory('agx_bringup')

    use_rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='是否启动 RViz')

    map_save_path = os.path.expanduser('~/maps/scans.pcd')
    os.makedirs(os.path.dirname(map_save_path), exist_ok=True)

    # MID360 驱动（CustomMsg 格式，FAST-LIO 必需）
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py'))
    )

    # FAST-LIO 建图节点（直接起节点以便覆盖 map_file_path）
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            os.path.join(fast_lio_share, 'config', 'mid360.yaml'),
            {'map_file_path': map_save_path},
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(fast_lio_share, 'rviz', 'fastlio.rviz')],
        condition=IfCondition(use_rviz),
    )

    # Ranger 底盘（cmd_vel 驱动，轮式里程计仅作记录）
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

    # TF 链: odom -> camera_init (FAST-LIO 起点) -> body (雷达) -> base_footprint
    tf_odom_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
    )
    tf_body_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_base',
        arguments=[str(-LIDAR_X), str(-LIDAR_Y), str(-LIDAR_Z),
                   '0', '0', '0', 'body', 'base_footprint'],
    )

    return LaunchDescription([
        declare_rviz,
        livox_driver,
        fast_lio_node,
        rviz_node,
        chassis_node,
        tf_odom_camera_init,
        tf_body_base,
    ])
