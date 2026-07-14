from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _arm_node(namespace: str, can_port_cfg: str):
    installation_cfg = (
        "left_installation_pos" if namespace == "left_arm" else "right_installation_pos"
    )
    return Node(
        package="agx_arm_ctrl",
        executable="agx_arm_ctrl_single",
        namespace=namespace,
        name="agx_arm_ctrl_single_node",
        output="screen",
        parameters=[
            {
                "can_port": LaunchConfiguration(can_port_cfg),
                "arm_type": LaunchConfiguration("arm_type"),
                "effector_type": LaunchConfiguration("effector_type"),
                "auto_enable": True,
                "speed_percent": LaunchConfiguration("speed_percent"),
                "pub_rate": 200,
                "enable_timeout": 5.0,
                "installation_pos": LaunchConfiguration(installation_cfg),
                "payload": "empty",
                "tcp_offset": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }
        ],
    )


def generate_launch_description():
    can_left_arg = DeclareLaunchArgument("can_left", default_value="can_left")
    can_right_arg = DeclareLaunchArgument("can_right", default_value="can_right")
    arm_type_arg = DeclareLaunchArgument("arm_type", default_value="piper")
    effector_type_arg = DeclareLaunchArgument("effector_type", default_value="none")
    speed_percent_arg = DeclareLaunchArgument("speed_percent", default_value="10")
    left_installation_pos_arg = DeclareLaunchArgument(
        "left_installation_pos", default_value="left"
    )
    right_installation_pos_arg = DeclareLaunchArgument(
        "right_installation_pos", default_value="right"
    )
    robot_db_arg = DeclareLaunchArgument("robot_db", default_value="~/maps/krt_robot.db")
    legacy_groups_file_arg = DeclareLaunchArgument("legacy_groups_file", default_value="")

    left_node = _arm_node("left_arm", "can_left")
    right_node = _arm_node("right_arm", "can_right")

    runner = Node(
        package="agx_action_group_runner",
        executable="action_group_runner",
        name="agx_action_group_runner",
        output="screen",
        parameters=[
            {
                "robot_db": LaunchConfiguration("robot_db"),
                "legacy_groups_file": LaunchConfiguration("legacy_groups_file"),
                "left_namespace": "/left_arm",
                "right_namespace": "/right_arm",
            }
        ],
    )

    return LaunchDescription(
        [
            can_left_arg,
            can_right_arg,
            arm_type_arg,
            effector_type_arg,
            speed_percent_arg,
            left_installation_pos_arg,
            right_installation_pos_arg,
            robot_db_arg,
            legacy_groups_file_arg,
            left_node,
            right_node,
            runner,
        ]
    )
