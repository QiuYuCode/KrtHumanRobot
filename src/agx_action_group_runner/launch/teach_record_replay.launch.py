from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_namespace_arg = DeclareLaunchArgument(
        "arm_namespace", default_value="/left_arm", description="Target arm namespace"
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="",
        description="Directory to save generated teaching replay files",
    )
    auto_replay_once_arg = DeclareLaunchArgument(
        "auto_replay_once", default_value="true", description="Replay once automatically"
    )
    sample_rate_arg = DeclareLaunchArgument(
        "sample_rate_hz", default_value="20.0", description="Sampling rate for joint states"
    )
    min_joint_delta_arg = DeclareLaunchArgument(
        "min_joint_delta_rad",
        default_value="0.01",
        description="Min joint delta rad to keep one sample",
    )
    replay_group_prefix_arg = DeclareLaunchArgument(
        "replay_group_prefix", default_value="teach_replay", description="Generated group prefix"
    )
    prefer_offline_teach_arg = DeclareLaunchArgument(
        "prefer_offline_teach",
        default_value="true",
        description="Prefer offline teach trajectory (fallback to sampled joint states)",
    )

    node = Node(
        package="agx_action_group_runner",
        executable="teach_record_replay",
        name="teach_record_replay",
        output="screen",
        parameters=[
            {
                "arm_namespace": LaunchConfiguration("arm_namespace"),
                "output_dir": LaunchConfiguration("output_dir"),
                "auto_replay_once": LaunchConfiguration("auto_replay_once"),
                "sample_rate_hz": LaunchConfiguration("sample_rate_hz"),
                "min_joint_delta_rad": LaunchConfiguration("min_joint_delta_rad"),
                "replay_group_prefix": LaunchConfiguration("replay_group_prefix"),
                "prefer_offline_teach": LaunchConfiguration("prefer_offline_teach"),
            }
        ],
    )

    return LaunchDescription(
        [
            arm_namespace_arg,
            output_dir_arg,
            auto_replay_once_arg,
            sample_rate_arg,
            min_joint_delta_arg,
            replay_group_prefix_arg,
            prefer_offline_teach_arg,
            node,
        ]
    )
