"""KRT wrapper for pcl_localization_ros2 as the map->odom provider."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_localizer(context, *args, **kwargs):
    pcd_map_path = LaunchConfiguration('pcd_map_path').perform(context)
    if not pcd_map_path:
        raise RuntimeError('pcd_map_path must not be empty')

    yaw = float(LaunchConfiguration('initial_pose_yaw').perform(context))
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)

    ranger_nav_share = get_package_share_directory('ranger_nav')
    pcl_share = get_package_share_directory('pcl_localization_ros2')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pcl_share, 'launch', 'nav2_lidar_localization.launch.py')),
        launch_arguments={
            'localization_param_dir': os.path.join(
                ranger_nav_share, 'config', 'lidar_localization_ros2_krt.yaml'),
            'pcd_map_path': pcd_map_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_frame_id': LaunchConfiguration('global_frame_id'),
            'odom_frame_id': LaunchConfiguration('odom_frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'use_imu': LaunchConfiguration('use_imu'),
            'use_odom': LaunchConfiguration('use_odom'),
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_z': LaunchConfiguration('initial_pose_z'),
            'initial_pose_qx': '0.0',
            'initial_pose_qy': '0.0',
            'initial_pose_qz': str(qz),
            'initial_pose_qw': str(qw),
            'points_topic': '/cloud_registered_body',
            'odom_topic': '/odom',
            'imu_topic': '/livox/imu',
            'initialpose_topic': '/initialpose',
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pcd_map_path', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='body'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('use_odom', default_value='false'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        OpaqueFunction(function=_include_localizer),
    ])
