"""构建语音对话行为树（从 smart-voice-robot/main.py 迁入）。"""

from __future__ import annotations

import py_trees
from py_trees.behaviour import Behaviour

from voice_assistant.config import RobotConfig
from voice_assistant.nodes import (
    BackToWakeUp,
    DefaultResponse,
    DescribeLeftPalmAction,
    DescribeRightPalmAction,
    DescribeSceneAction,
    DialogContinueGuard,
    FixedResponseAction,
    GripperAction,
    InitializeDialogBlackboard,
    ListenCloudCommand,
    ListenCommand,
    LLMDialogAction,
    LLMTaskPlanner,
    NavigationAction,
    PlanExecutor,
    RecognizeIntent,
    RecordVideoAction,
    ResetWakeWordInterruptState,
    RobotArmAction,
    SpeakResponse,
    TakePhotoAction,
    WakeWordInterruptMonitor,
    WaitForWakeWord,
    WakeupResponse,
)


def _create_speak_stage(config: RobotConfig) -> Behaviour:
    """构建播报与打断阶段。"""
    speak_stage = py_trees.composites.Sequence(name="SpeakStage", memory=False)
    speak_or_interrupt = py_trees.composites.Parallel(
        name="SpeakOrInterrupt",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    speak_or_interrupt.add_children([
        SpeakResponse("Speak"),
        WakeWordInterruptMonitor("WakeWordInterrupt", config),
    ])
    speak_stage.add_children([
        speak_or_interrupt,
        ResetWakeWordInterruptState("ResetWakeWordInterrupt", config),
    ])
    return speak_stage


def _create_keyword_dialog_loop(
    config: RobotConfig,
    listen_node: Behaviour,
    speak_stage: Behaviour,
) -> Behaviour:
    """构建关键词模式对话循环。"""
    action_selector = py_trees.composites.Selector(name="ActionSelector", memory=False)
    action_selector.add_children([
        FixedResponseAction("FixedResponse", config=config),
        DescribeLeftPalmAction("DescribeLeftPalm", config=config),
        DescribeRightPalmAction("DescribeRightPalm", config=config),
        DescribeSceneAction("DescribeScene", config=config),
        TakePhotoAction("TakePhoto", config=config),
        RecordVideoAction("RecordVideo", config=config),
        GripperAction("Gripper", config=config),
        RobotArmAction("RobotArm", config=config),
        NavigationAction("Navigation"),
        LLMDialogAction("LLMDialog", config=config),
        BackToWakeUp("BackToWakeUp"),
        DefaultResponse("DefaultResponse"),
    ])
    dialog_loop = py_trees.composites.Sequence(name="DialogLoopKeyword", memory=True)
    dialog_loop.add_children([
        InitializeDialogBlackboard(),
        listen_node,
        RecognizeIntent("Intent", config=config),
        action_selector,
        speak_stage,
        DialogContinueGuard("ContinueGuard"),
    ])
    return dialog_loop


def _create_planner_dialog_loop(
    config: RobotConfig,
    listen_node: Behaviour,
    speak_stage: Behaviour,
) -> Behaviour:
    """构建 LLM 规划模式对话循环。"""
    dialog_loop = py_trees.composites.Sequence(name="DialogLoopPlanner", memory=True)
    dialog_loop.add_children([
        InitializeDialogBlackboard(),
        listen_node,
        LLMTaskPlanner("Planner", config=config),
        PlanExecutor("Executor", config=config),
        speak_stage,
        DialogContinueGuard("ContinueGuard"),
    ])
    return dialog_loop


def create_tree(config: RobotConfig) -> Behaviour:
    """构建完整语音对话行为树（仅结构，不负责执行控制）。"""
    listen_node = (
        ListenCloudCommand("ListenCloud", config)
        if config.asr_backend == "iflytek_cloud"
        else ListenCommand("Listen", config)
    )
    speak_stage = _create_speak_stage(config)

    dialog_loop = (
        _create_planner_dialog_loop(config, listen_node, speak_stage)
        if config.use_llm_planner
        else _create_keyword_dialog_loop(config, listen_node, speak_stage)
    )

    dialog_entry = dialog_loop
    if config.continuous_dialog:
        dialog_entry = py_trees.decorators.SuccessIsRunning(
            name="DialogRepeat",
            child=dialog_loop,
        )

    root = py_trees.composites.Sequence(name="Root", memory=True)
    root.add_children([
        WaitForWakeWord("WakeWord"),
        WakeupResponse("WakeupSound", config),
        dialog_entry,
    ])
    return root
