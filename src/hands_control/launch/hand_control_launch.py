"""Launch file for DexHand021S dual hand control."""
from launch import LaunchDescription
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
            'device_id': LaunchConfiguration('left_hand_device_id'),
            'hand_name': '左手',
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
            'device_id': LaunchConfiguration('right_hand_device_id'),
            'hand_name': '右手',
        }]
    )

    return LaunchDescription([
        adapter_type_arg,
        left_device_id_arg,
        right_device_id_arg,
        left_hand_control_server,
        right_hand_control_server,
    ])
