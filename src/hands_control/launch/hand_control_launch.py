"""Launch file for DexHand021S dual hand control."""
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for hand control server."""
    # 声明启动参数
    adapter_type_arg = DeclareLaunchArgument(
        'adapter_type',
        default_value='ZLG_MINI',
        description='ZLG adapter type (ZLG_MINI or ZLG_200U)'
    )

    left_device_id_arg = DeclareLaunchArgument(
        'left_hand_device_id',
        default_value='1',
        description='Left hand device ID (default: 0x01)'
    )

    right_device_id_arg = DeclareLaunchArgument(
        'right_hand_device_id',
        default_value='2',
        description='Right hand device ID (default: 0x02)'
    )

    left_listen_arg = DeclareLaunchArgument(
        'left_hand_listen',
        default_value='false',
        description='Enable left hand listen on startup'
    )

    right_listen_arg = DeclareLaunchArgument(
        'right_hand_listen',
        default_value='false',
        description='Enable right hand listen on startup'
    )

    left_realtime_response_arg = DeclareLaunchArgument(
        'left_hand_realtime_response',
        default_value='false',
        description='Enable left hand realtime response on startup'
    )

    right_realtime_response_arg = DeclareLaunchArgument(
        'right_hand_realtime_response',
        default_value='false',
        description='Enable right hand realtime response on startup'
    )

    # 创建节点（左右手各一个 Action Server）
    left_hand_control_server = Node(
        package='hands_control',
        executable='hand_control_server',
        name='hand_control_server',
        namespace='left',
        output='screen',
        parameters=[{
            'adapter_type': LaunchConfiguration('adapter_type'),
            'adapter_index': 0,
            'device_id': ParameterValue(
                LaunchConfiguration('left_hand_device_id'),
                value_type=int,
            ),
            'hand_name': '左手',
            'has_pressure_sensor': False,
            'listen_enabled': ParameterValue(
                LaunchConfiguration('left_hand_listen'),
                value_type=bool,
            ),
            'realtime_response_enabled': ParameterValue(
                LaunchConfiguration('left_hand_realtime_response'),
                value_type=bool,
            ),
        }]
    )

    right_hand_control_server = Node(
        package='hands_control',
        executable='hand_control_server',
        name='hand_control_server',
        namespace='right',
        output='screen',
        parameters=[{
            'adapter_type': LaunchConfiguration('adapter_type'),
            'adapter_index': 1,
            'device_id': ParameterValue(
                LaunchConfiguration('right_hand_device_id'),
                value_type=int,
            ),
            'hand_name': '右手',
            'has_pressure_sensor': True,
            'listen_enabled': ParameterValue(
                LaunchConfiguration('right_hand_listen'),
                value_type=bool,
            ),
            'realtime_response_enabled': ParameterValue(
                LaunchConfiguration('right_hand_realtime_response'),
                value_type=bool,
            ),
        }]
    )

    return LaunchDescription([
        adapter_type_arg,
        left_device_id_arg,
        right_device_id_arg,
        left_listen_arg,
        right_listen_arg,
        left_realtime_response_arg,
        right_realtime_response_arg,
        left_hand_control_server,
        right_hand_control_server,
    ])
