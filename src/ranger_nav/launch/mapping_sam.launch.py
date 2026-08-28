"""方式 A：spark_fast_lio + KISS-Matcher-SAM 回环建图。

与方式 B（mapping.launch.py / fast_lio）并行存在，互不 include。

流程：
1. ros2 launch ranger_nav mapping_sam.launch.py
2. ros2 run teleop_twist_keyboard teleop_twist_keyboard
3. ros2 topic pub --once /km_sam/save_dir std_msgs/msg/String "data: '$HOME/maps'"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ranger_nav_share = get_package_share_directory('ranger_nav')
    kiss_share = get_package_share_directory('kiss_matcher_ros')

    use_rviz = LaunchConfiguration('sam_rviz')
    declare_rviz = DeclareLaunchArgument(
        'sam_rviz', default_value='true', description='是否启动 KISS-Matcher-SAM RViz')

    spark_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ranger_nav_share, 'launch', 'mapping_spark.launch.py')),
        launch_arguments={'rviz': 'false'}.items(),
    )

    sam_node = Node(
        package='kiss_matcher_ros',
        executable='kiss_matcher_sam',
        name='kiss_matcher_sam',
        namespace='km_sam',
        output='screen',
        parameters=[
            os.path.join(ranger_nav_share, 'config', 'kiss_matcher_sam.yaml'),
            {'map_frame': 'map', 'base_frame': 'base_sam'},
        ],
        remappings=[
            ('/cloud', '/cloud_registered'),
            ('/odom', '/odometry'),
        ],
    )

    sam_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='km_sam_rviz',
        arguments=['-d', os.path.join(kiss_share, 'rviz', 'kiss_matcher_sam.rviz')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_rviz,
        spark_base,
        sam_node,
        sam_rviz,
    ])
