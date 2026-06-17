"""启动语音助手节点（uv 虚拟环境 + py_trees_ros）。

用法:
    ros2 launch voice_assistant voice_assistant.launch.py

依赖:
    - apt: ros-humble-py-trees / py-trees-ros
    - uv: bash src/voice_assistant/scripts/setup_uv.sh
    - 模型: voice_assistant/model/voice_models/（见 model/README.md）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("voice_assistant")
    default_config = os.path.join(pkg_share, "config", "voice_assistant.yaml")
    run_script = os.path.join(pkg_share, "scripts", "run_voice_node.sh")

    config_file = LaunchConfiguration("config_file")
    tick_interval_ms = LaunchConfiguration("tick_interval_ms")
    enable_monitor = LaunchConfiguration("enable_monitor")
    snapshot_period_s = LaunchConfiguration("snapshot_period_s")
    snapshot_blackboard_data = LaunchConfiguration("snapshot_blackboard_data")
    snapshot_blackboard_activity = LaunchConfiguration("snapshot_blackboard_activity")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="voice_assistant.yaml 路径",
        ),
        DeclareLaunchArgument(
            "tick_interval_ms",
            default_value="100",
            description="行为树 tick 周期（毫秒）",
        ),
        DeclareLaunchArgument(
            "enable_monitor",
            default_value="true",
            description="是否启用 py_trees_ros 监控发布",
        ),
        DeclareLaunchArgument(
            "snapshot_period_s",
            default_value="2.0",
            description="快照发布周期（秒）",
        ),
        DeclareLaunchArgument(
            "snapshot_blackboard_data",
            default_value="true",
            description="快照中附加 blackboard 数据",
        ),
        DeclareLaunchArgument(
            "snapshot_blackboard_activity",
            default_value="false",
            description="快照中附加 blackboard 活动流",
        ),
        ExecuteProcess(
            cmd=[run_script],
            output="screen",
            additional_env={
                "VOICE_ASSISTANT_CONFIG": config_file,
                "VOICE_TICK_INTERVAL_MS": tick_interval_ms,
                "VOICE_ENABLE_MONITOR": enable_monitor,
                "VOICE_SNAPSHOT_PERIOD_S": snapshot_period_s,
                "VOICE_SNAPSHOT_BLACKBOARD_DATA": snapshot_blackboard_data,
                "VOICE_SNAPSHOT_BLACKBOARD_ACTIVITY": snapshot_blackboard_activity,
            },
        ),
    ])
