"""Start whole robot MoveIt planning with simulated joint execution."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = Path(get_package_share_directory("krthumanrobot_urdf"))
    config_share = Path(
        get_package_share_directory("krthumanrobot_moveit_config")
    )
    config = (
        MoveItConfigsBuilder(
            "eAI2810_dual_arm_review",
            package_name="krthumanrobot_moveit_config",
        )
        .robot_description(
            file_path=str(description_share / "config/robot_moveit.urdf")
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl", pipelines=["ompl"]
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    def build_execution(context):
        if LaunchConfiguration("execution_backend").perform(context) == "mock":
            return [
                Node(
                    package="controller_manager",
                    executable="ros2_control_node",
                    output="screen",
                    parameters=[
                        config.robot_description,
                        str(config_share / "config/ros2_controllers.yaml"),
                    ],
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
                    output="screen",
                ),
            ]

        agx_launch = str(
            Path(get_package_share_directory("agx_arm_ctrl"))
            / "launch/start_single_agx_arm.launch.py"
        )
        driver_args = {
            "arm_type": "nero",
            "effector_type": "none",
            "control_enabled": "false",
            "auto_enable": "true",
        }
        actions = []
        for side, port in (
            ("left", LaunchConfiguration("left_can_port").perform(context)),
            ("right", LaunchConfiguration("right_can_port").perform(context)),
        ):
            namespace = f"{side}_arm"
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(agx_launch),
                    launch_arguments={
                        **driver_args,
                        "namespace": namespace,
                        "can_port": port,
                    }.items(),
                )
            )
            moveit_names = [f"{namespace}_link{i}_joint" for i in range(1, 8)]
            driver_names = [f"joint{i}" for i in range(1, 8)]
            actions.append(
                Node(
                    package="agx_arm_moveit",
                    executable="agx_arm_trajectory_bridge",
                    output="screen",
                    parameters=[
                        {
                            "arm_type": "nero",
                            "moveit_joint_names": moveit_names,
                            "driver_joint_names": driver_names,
                            "action_name": f"{namespace}_controller/follow_joint_trajectory",
                            "command_topic": f"/{namespace}/control/move_j",
                            "feedback_topic": f"/{namespace}/feedback/joint_states",
                            "state_topic": "/joint_states",
                            "control_gate_service": f"/{namespace}/control_enable",
                            "emergency_stop_service": f"/{namespace}/emergency_stop",
                        }
                    ],
                )
            )
        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("execution_backend", default_value="mock"),
            DeclareLaunchArgument("left_can_port", default_value="can_left"),
            DeclareLaunchArgument("right_can_port", default_value="can_right"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[config.robot_description],
            ),
            OpaqueFunction(function=build_execution),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    config.to_dict(),
                    {
                        "allow_trajectory_execution": True,
                        "publish_robot_description_semantic": True,
                        "publish_planning_scene": True,
                        "publish_geometry_updates": True,
                        "publish_state_updates": True,
                        "publish_transforms_updates": True,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                arguments=["-d", str(config_share / "config/moveit.rviz")],
                parameters=[
                    config.robot_description,
                    config.robot_description_semantic,
                    config.robot_description_kinematics,
                    config.planning_pipelines,
                ],
            ),
        ]
    )
