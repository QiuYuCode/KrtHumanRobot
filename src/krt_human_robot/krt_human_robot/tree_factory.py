"""Build the KrtHumanRobot core behavior tree."""

from __future__ import annotations

import py_trees
from py_trees.behaviour import Behaviour

from krt_human_robot.actions import (
    BackToWakeUp,
    CorePlanExecutor,
    DefaultResponse,
    FixedResponseAction,
    GripperAction,
    LLMDialogAction,
    NavigationAction,
    RoutineAction,
    RobotArmAction,
    RoutineVoiceAction,
)
from krt_human_robot.behaviors.core.actions.camera import (
    RecordVideoAction,
    TakePhotoAction,
)
from krt_human_robot.behaviors.core.actions.vision import (
    DescribeLeftPalmAction,
    DescribeRightPalmAction,
    DescribeSceneAction,
)
from krt_human_robot.behaviors.core.blackboard_init import InitializeDialogBlackboard
from krt_human_robot.behaviors.core.guards import DialogContinueGuard
from krt_human_robot.behaviors.core.intent import RecognizeIntent
from krt_human_robot.behaviors.core.interrupt import (
    ResetWakeWordInterruptState,
    WakeWordInterruptMonitor,
)
from krt_human_robot.behaviors.core.listen import ListenCommand
from krt_human_robot.behaviors.core.listen_cloud import ListenCloudCommand
from krt_human_robot.behaviors.core.planner import LLMTaskPlanner
from krt_human_robot.behaviors.core.speak import SpeakResponse, WakeupResponse
from krt_human_robot.behaviors.core.wake_word import WaitForWakeWord


def _create_speak_stage(config) -> Behaviour:
    speak_stage = py_trees.composites.Sequence(name="SpeakStage", memory=False)
    speak_or_interrupt = py_trees.composites.Parallel(
        name="SpeakOrInterrupt",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    speak_or_interrupt.add_children([
        SpeakResponse("Speak", config),
        WakeWordInterruptMonitor("WakeWordInterrupt", config),
    ])
    speak_stage.add_children([
        speak_or_interrupt,
        ResetWakeWordInterruptState("ResetWakeWordInterrupt", config),
    ])
    return speak_stage


def _create_keyword_dialog_loop(
    config,
    listen_node: Behaviour,
    speak_stage: Behaviour,
    routine_database=None,
) -> Behaviour:
    action_selector = py_trees.composites.Selector(name="CoreActionSelector", memory=False)
    action_selector.add_children([
        FixedResponseAction("FixedResponse", config=config),
        DescribeLeftPalmAction("DescribeLeftPalm", config=config),
        DescribeRightPalmAction("DescribeRightPalm", config=config),
        DescribeSceneAction("DescribeScene", config=config),
        TakePhotoAction("TakePhoto", config=config),
        RecordVideoAction("RecordVideo", config=config),
        GripperAction("Gripper", config=config),
        RobotArmAction("RobotArm", config=config),
        NavigationAction("Navigation", config=config),
        RoutineAction("Routine", config=config),
        LLMDialogAction("LLMDialog", config=config),
        BackToWakeUp("BackToWakeUp"),
        DefaultResponse("DefaultResponse"),
    ])
    keyword_handler = py_trees.composites.Sequence(
        name="KeywordHandler", memory=True
    )
    keyword_handler.add_children([
        RecognizeIntent("Intent", config=config),
        action_selector,
    ])
    command_handler = py_trees.composites.Selector(
        name="CommandHandler", memory=False
    )
    command_handler.add_children([
        RoutineVoiceAction("RoutineVoice", config, routine_database),
        keyword_handler,
    ])
    dialog_loop = py_trees.composites.Sequence(
        name="CoreDialogLoopKeyword", memory=True
    )
    dialog_loop.add_children([
        InitializeDialogBlackboard(),
        listen_node,
        command_handler,
        speak_stage,
        DialogContinueGuard("ContinueGuard"),
    ])
    return dialog_loop


def _create_planner_dialog_loop(
    config,
    listen_node: Behaviour,
    speak_stage: Behaviour,
    routine_database=None,
) -> Behaviour:
    planner_handler = py_trees.composites.Sequence(
        name="PlannerHandler", memory=True
    )
    planner_handler.add_children([
        LLMTaskPlanner("Planner", config=config),
        CorePlanExecutor("Executor", config=config),
    ])
    command_handler = py_trees.composites.Selector(
        name="CommandHandler", memory=False
    )
    command_handler.add_children([
        RoutineVoiceAction("RoutineVoice", config, routine_database),
        planner_handler,
    ])
    dialog_loop = py_trees.composites.Sequence(name="CoreDialogLoopPlanner", memory=True)
    dialog_loop.add_children([
        InitializeDialogBlackboard(),
        listen_node,
        command_handler,
        speak_stage,
        DialogContinueGuard("ContinueGuard"),
    ])
    return dialog_loop


def create_tree(config, routine_database=None) -> Behaviour:
    listen_node = (
        ListenCloudCommand("ListenCloud", config)
        if config.asr_backend == "iflytek_cloud"
        else ListenCommand("Listen", config)
    )
    speak_stage = _create_speak_stage(config)
    dialog_loop = (
        _create_planner_dialog_loop(
            config, listen_node, speak_stage, routine_database
        )
        if config.use_llm_planner
        else _create_keyword_dialog_loop(
            config, listen_node, speak_stage, routine_database
        )
    )
    dialog_entry = dialog_loop
    if config.continuous_dialog:
        dialog_entry = py_trees.decorators.SuccessIsRunning(
            name="DialogRepeat",
            child=dialog_loop,
        )

    root = py_trees.composites.Sequence(name="KrtHumanRobotRoot", memory=True)
    root.add_children([
        WaitForWakeWord("WakeWord"),
        WakeupResponse("WakeupSound", config),
        dialog_entry,
    ])
    return root
