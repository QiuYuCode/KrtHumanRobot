from types import SimpleNamespace

import py_trees
import pytest

from krt_human_robot.behaviors.core.intent import RecognizeIntent


@pytest.mark.parametrize(("command", "expected"), [
    ("描述一下你左手看到了什么", "describe_left_palm"),
    ("看一看左手有什么", "describe_left_palm"),
    ("看一下左手有什么", "describe_left_palm"),
    ("描述一下你右手看到了什么", "describe_right_palm"),
    ("看一看右手有什么", "describe_right_palm"),
    ("看一下右手有什么", "describe_right_palm"),
    ("描述一下前面有什么", "describe_scene"),
    ("左手张开", "gripper_control"),
    ("右手张开", "gripper_control"),
])
def test_recognize_intent_prefers_longest_keyword(command, expected):
    config = SimpleNamespace(intent_patterns={
        "describe_left_palm": ["描述一下你左手", "看一看左手", "看一下左手"],
        "describe_right_palm": ["描述一下你右手", "看一看右手", "看一下右手"],
        "describe_scene": ["描述一下", "看一下", "看看", "前面有什么"],
        "gripper_control": ["左手张开", "右手张开", "左手", "右手"],
    })
    py_trees.blackboard.Blackboard.set("dialog/user_command", command)
    behaviour = RecognizeIntent("Intent", config)

    assert behaviour.update() == py_trees.common.Status.SUCCESS
    assert py_trees.blackboard.Blackboard.get("dialog/intent") == expected
