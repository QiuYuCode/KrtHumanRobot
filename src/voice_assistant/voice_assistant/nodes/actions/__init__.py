"""动作节点模块"""

from voice_assistant.nodes.actions.camera import (
    TakePhotoAction,
    RecordVideoAction,
    execute_take_photo,
    execute_record_video,
    capture_frame_as_base64,
)
from voice_assistant.nodes.actions.robot_arm import RobotArmAction, execute_robot_arm
from voice_assistant.nodes.actions.navigation import NavigationAction, execute_navigate
from voice_assistant.nodes.actions.llm_dialog import LLMDialogAction
from voice_assistant.nodes.actions.vision import (
    DescribeSceneAction,
    DescribeLeftPalmAction,
    DescribeRightPalmAction,
    execute_describe_scene,
)
from voice_assistant.nodes.actions.back_to_wakeup import BackToWakeUp
from voice_assistant.nodes.actions.default_response import DefaultResponse
from voice_assistant.nodes.actions.fixed_response import FixedResponseAction
from voice_assistant.nodes.actions.gripper import GripperAction, execute_gripper_action

__all__ = [
    "TakePhotoAction",
    "RecordVideoAction",
    "DescribeSceneAction",
    "DescribeLeftPalmAction",
    "DescribeRightPalmAction",
    "RobotArmAction",
    "NavigationAction",
    "LLMDialogAction",
    "BackToWakeUp",
    "FixedResponseAction",
    "DefaultResponse",
    "GripperAction",
    "execute_take_photo",
    "execute_record_video",
    "execute_describe_scene",
    "capture_frame_as_base64",
    "execute_robot_arm",
    "execute_navigate",
    "execute_gripper_action",
]
