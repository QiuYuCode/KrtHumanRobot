"""Launch lifecycle-managed DexHand021S servers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def _hand_node(side, device_id, enabled):
    return LifecycleNode(
        package="hands_control",
        executable="hand_control_server",
        name="hand_control_server",
        namespace=side,
        output="screen",
        condition=IfCondition(LaunchConfiguration(enabled)),
        parameters=[{
            "adapter_type": LaunchConfiguration("adapter_type"),
            "adapter_index": ParameterValue(
                LaunchConfiguration(f"{side}_hand_adapter_index"), value_type=int
            ),
            "device_id": ParameterValue(
                LaunchConfiguration(device_id), value_type=int
            ),
            "hand_name": "左手" if side == "left" else "右手",
            "has_pressure_sensor": side == "right",
            "listen_enabled": ParameterValue(
                LaunchConfiguration(f"{side}_hand_listen"), value_type=bool
            ),
            "realtime_response_enabled": ParameterValue(
                LaunchConfiguration(f"{side}_hand_realtime_response"), value_type=bool
            ),
        }],
    )


def _autostart(node):
    condition = IfCondition(LaunchConfiguration("autostart"))
    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda action: action == node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))],
        ),
        condition=condition,
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda action: action == node,
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        ),
        condition=condition,
    )
    return [configure, activate]


def generate_launch_description():
    """Generate left/right lifecycle nodes with backward-compatible autostart."""
    arguments = [
        DeclareLaunchArgument("adapter_type", default_value="ZLG_MINI"),
        DeclareLaunchArgument("enable_left", default_value="true"),
        DeclareLaunchArgument("enable_right", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("left_hand_adapter_index", default_value="0"),
        DeclareLaunchArgument("right_hand_adapter_index", default_value="1"),
        DeclareLaunchArgument("left_hand_device_id", default_value="1"),
        DeclareLaunchArgument("right_hand_device_id", default_value="2"),
        DeclareLaunchArgument("left_hand_listen", default_value="false"),
        DeclareLaunchArgument("right_hand_listen", default_value="false"),
        DeclareLaunchArgument("left_hand_realtime_response", default_value="false"),
        DeclareLaunchArgument("right_hand_realtime_response", default_value="false"),
    ]
    left = _hand_node("left", "left_hand_device_id", "enable_left")
    right = _hand_node("right", "right_hand_device_id", "enable_right")
    return LaunchDescription([
        *arguments,
        left,
        right,
        *_autostart(left),
        *_autostart(right),
    ])
