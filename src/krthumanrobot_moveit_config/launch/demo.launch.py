"""Start whole robot MoveIt planning with simulated joint execution."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[config.robot_description],
            ),
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
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "left_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "right_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
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
