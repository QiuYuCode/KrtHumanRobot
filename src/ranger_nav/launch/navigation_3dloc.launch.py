"""Navigation with 3D lidar localization publishing map->odom."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SCAN_MIN_HEIGHT = 0.15
SCAN_MAX_HEIGHT = 1.2


def _validate_paths(context, *args, **kwargs):
    if not LaunchConfiguration('map').perform(context):
        raise RuntimeError('map must not be empty')
    if not LaunchConfiguration('pcd_map_path').perform(context):
        raise RuntimeError('pcd_map_path must not be empty')
    return []


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    livox_share = get_package_share_directory('livox_ros_driver2')
    agx_share = get_package_share_directory('agx_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    ranger_nav_share = get_package_share_directory('ranger_nav')

    map_yaml = LaunchConfiguration('map')
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    use_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py'))
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            os.path.join(fast_lio_share, 'config', 'mid360.yaml'),
            {'pcd_save.pcd_save_en': False},
        ],
        output='screen',
    )

    chassis_node = Node(
        package='agx_bringup',
        executable='agx_bringup_node',
        name='agx_bringup_node',
        output='screen',
        parameters=[os.path.join(agx_share, 'config', 'agx_bringup.yaml')],
        remappings=[
            ('/sub_cmd_vel', '/cmd_vel_safe'),
            ('/wheel/odom', '/odom'),
        ],
    )

    tf_odom_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
    )

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(
            ranger_nav_share, 'urdf', 'ranger_mini.urdf.xacro')]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

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
            'angle_increment': 0.0058,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 30.0,
            'use_inf': True,
            'use_sim_time': use_sim_time,
        }],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            os.path.join(ranger_nav_share, 'config', 'nav2_params_3dloc.yaml'),
            {'yaml_filename': map_yaml, 'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server_3dloc',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': '',
            'params_file': os.path.join(
                ranger_nav_share, 'config', 'nav2_params_3dloc.yaml'),
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[
            os.path.join(ranger_nav_share, 'config', 'nav2_params_3dloc.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager_collision_monitor = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    lidar_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ranger_nav_share, 'launch', 'lidar_localization_ros2_krt.launch.py')),
        launch_arguments={
            'pcd_map_path': pcd_map_path,
            'use_sim_time': use_sim_time,
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_z': LaunchConfiguration('initial_pose_z'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            'use_imu': LaunchConfiguration('use_imu'),
            'use_odom': LaunchConfiguration('use_odom'),
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
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/map.yaml')),
        DeclareLaunchArgument('pcd_map_path', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('use_odom', default_value='false'),
        OpaqueFunction(function=_validate_paths),
        livox_driver,
        fast_lio_node,
        chassis_node,
        tf_odom_camera_init,
        robot_state_publisher,
        cloud_to_scan,
        map_server,
        lifecycle_manager_map_server,
        nav2_navigation,
        collision_monitor_node,
        lifecycle_manager_collision_monitor,
        lidar_localization,
        rviz_node,
    ])
