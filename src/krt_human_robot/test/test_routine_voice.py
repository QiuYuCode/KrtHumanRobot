from pathlib import Path
from types import SimpleNamespace

import py_trees
import pytest

from krt_human_robot.behaviors.core.actions.routine import (
    RoutineVoiceAction,
    RoutineVoiceTriggerAmbiguous,
    resolve_routine_voice_trigger,
)
from krt_human_robot.config import load_config
from krt_human_robot.robot_node import create_routine_database
from krt_human_robot.tree_factory import create_tree
from krt_task.robot_db import RobotDatabase


CONFIG = Path(__file__).parents[1] / "config" / "krt_human_robot.yaml"
WAIT_SPEC = {"type": "sequence", "steps": [{"type": "wait", "wait_ms": 10}]}


def save_trigger(database, name, keywords, response_text=""):
    database.save_routine_configuration(
        name,
        WAIT_SPEC,
        {"keywords": keywords, "response_text": response_text},
    )


def test_resolver_reads_latest_state_and_prefers_longest_keyword(tmp_path):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    save_trigger(database, "普通表演", ["表演"])
    save_trigger(database, "迎宾表演", ["开始迎宾表演"], "迎宾完成")

    matched = resolve_routine_voice_trigger(database, "请开始迎宾表演")
    assert matched is not None
    assert matched.routine_name == "迎宾表演"
    assert matched.response_text == "迎宾完成"

    save_trigger(database, "迎宾表演", [])
    matched = resolve_routine_voice_trigger(database, "请开始迎宾表演")
    assert matched is not None
    assert matched.routine_name == "普通表演"


def test_resolver_rejects_equal_length_matches_for_different_routines(
    tmp_path,
):
    database = RobotDatabase(str(tmp_path / "robot.db"))
    save_trigger(database, "左手动作", ["左手比心"])
    save_trigger(database, "右手动作", ["右手挥手"])

    with pytest.raises(RoutineVoiceTriggerAmbiguous) as error:
        resolve_routine_voice_trigger(database, "先左手比心再右手挥手")
    assert "左手动作" in str(error.value)
    assert "右手动作" in str(error.value)


def test_routine_voice_action_fails_closed_when_database_is_unavailable():
    class BrokenDatabase:
        def list_routine_voice_triggers(self):
            raise OSError("database unavailable")

    py_trees.blackboard.Blackboard.set("dialog/user_command", "开始迎宾")
    behaviour = RoutineVoiceAction("RoutineVoice", object(), BrokenDatabase())
    behaviour.initialise()

    assert behaviour.update() == py_trees.common.Status.SUCCESS
    assert "语音动作配置暂不可用" in py_trees.blackboard.Blackboard.get(
        "dialog/response_text"
    )


def test_routine_voice_action_reports_busy_routine_without_failure(tmp_path):
    class Future:
        def __init__(self, value):
            self.value = value

        def done(self):
            return True

        def result(self):
            return self.value

    class GoalHandle:
        accepted = True

        def get_result_async(self):
            return Future(
                SimpleNamespace(
                    result=SimpleNamespace(
                        success=False, message="routine 正在执行"
                    )
                )
            )

    class Client:
        def server_is_ready(self):
            return True

        def send_goal_async(self, _goal):
            return Future(GoalHandle())

    database = RobotDatabase(str(tmp_path / "robot.db"))
    save_trigger(database, "夹爪动作", ["口渴"])
    py_trees.blackboard.Blackboard.set("dialog/user_command", "我口渴了")
    behaviour = RoutineVoiceAction(
        "RoutineVoice",
        SimpleNamespace(routine_action_timeout_s=30.0),
        database,
    )
    behaviour._client = Client()
    behaviour.initialise()

    assert behaviour.update() == py_trees.common.Status.RUNNING
    assert behaviour.update() == py_trees.common.Status.RUNNING
    assert behaviour.update() == py_trees.common.Status.SUCCESS
    response = py_trees.blackboard.Blackboard.get("dialog/response_text")
    assert "正在执行" in response
    assert "流程执行失败" not in response


def test_routine_voice_action_dispatches_without_spinning_the_node(
    monkeypatch, tmp_path
):
    """Routine dispatch must progress on tree ticks, not recursively spin its node."""

    class Future:
        def __init__(self, value=None, done=False):
            self.value = value
            self._done = done

        def done(self):
            return self._done

        def result(self):
            return self.value

    class GoalHandle:
        accepted = True

        def __init__(self, result_future):
            self._result_future = result_future

        def get_result_async(self):
            return self._result_future

    class Client:
        def __init__(self, *_args):
            self.send_count = 0
            self.result_future = Future()
            self.send_future = Future(GoalHandle(self.result_future))

        def wait_for_server(self, timeout_sec):
            return True

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            self.send_count += 1
            assert goal.routine_name == "夹爪动作"
            return self.send_future

    client = Client()
    monkeypatch.setattr(
        "krt_human_robot.behaviors.core.actions.routine.ActionClient",
        lambda *_args: client,
    )
    database = RobotDatabase(str(tmp_path / "robot.db"))
    save_trigger(database, "夹爪动作", ["口渴"], "给你饮料")
    config = SimpleNamespace(
        routine_action="/krt_task/run_routine", routine_action_timeout_s=30.0
    )
    py_trees.blackboard.Blackboard.set("dialog/user_command", "我口渴了")
    behaviour = RoutineVoiceAction("RoutineVoice", config, database)
    behaviour.setup(node=object())
    behaviour.initialise()

    assert behaviour.update() == py_trees.common.Status.RUNNING
    assert client.send_count == 1

    client.send_future._done = True
    assert behaviour.update() == py_trees.common.Status.RUNNING

    client.result_future.value = SimpleNamespace(
        result=SimpleNamespace(success=True, message="routine completed")
    )
    client.result_future._done = True
    assert behaviour.update() == py_trees.common.Status.SUCCESS
    assert py_trees.blackboard.Blackboard.get("dialog/response_text") == "给你饮料"


@pytest.mark.parametrize("use_llm_planner", [False, True])
def test_tree_places_one_routine_voice_action_before_normal_dispatch(
    tmp_path, use_llm_planner
):
    config = load_config(CONFIG)
    config.use_llm_planner = use_llm_planner
    config.continuous_dialog = False
    database = RobotDatabase(str(tmp_path / "robot.db"))

    root = create_tree(config, routine_database=database)
    routine_nodes = [
        node for node in root.iterate() if node.name == "RoutineVoice"
    ]

    assert len(routine_nodes) == 1
    assert routine_nodes[0].parent.name == "CommandHandler"
    assert routine_nodes[0].parent.children[0] is routine_nodes[0]


def test_keyword_tree_keeps_legacy_routine_intent_before_llm_fallback(tmp_path):
    config = load_config(CONFIG)
    config.use_llm_planner = False
    config.continuous_dialog = False

    root = create_tree(
        config,
        routine_database=RobotDatabase(str(tmp_path / "robot.db")),
    )
    action_selector = next(
        node for node in root.iterate() if node.name == "CoreActionSelector"
    )
    action_names = [child.name for child in action_selector.children]

    assert "Routine" in action_names
    assert action_names.index("Routine") < action_names.index("LLMDialog")


def test_core_database_uses_environment_path_and_imports_legacy_mapping(
    monkeypatch, tmp_path
):
    path = tmp_path / "shared.db"
    database = RobotDatabase(str(path))
    database.save_routine("迎宾", WAIT_SPEC)
    monkeypatch.setenv("KRT_ROBOT_DB", str(path))
    config = SimpleNamespace(
        adapters={"navigation": {"robot_db": "/wrong/database.db"}},
        routine_keyword_actions=[
            {
                "routine_name": "迎宾",
                "keywords": ["开始迎宾"],
                "response_text": "迎宾完成",
            }
        ],
    )

    loaded = create_routine_database(config)

    assert loaded.path == path
    assert loaded.list_routine_voice_triggers()[0]["keywords"] == ["开始迎宾"]
