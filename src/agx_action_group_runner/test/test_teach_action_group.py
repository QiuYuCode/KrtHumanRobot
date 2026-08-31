import threading
from types import SimpleNamespace

import pytest
from agx_arm_msgs.msg import AgxArmStatus
from sensor_msgs.msg import JointState

import agx_action_group_runner.runner_node as runner_module
from agx_action_group_runner.runner_node import ActionGroupRunnerNode
from agx_action_group_runner.teach_action_group_node import TeachActionGroupNode
from agx_action_group_runner.teach_action_group_node import ArmRecorder


class Recorder:
    def __init__(self):
        self.stopped = False

    def stop(self):
        self.stopped = True
        return [object()]


def test_stop_without_group_saves_unnamed_action_group():
    node = object.__new__(TeachActionGroupNode)
    recorder = Recorder()
    node.lock = threading.Lock()
    node.recorders = {"left": recorder}
    node.active_arm = "left"
    node.active_group = None
    node.database = SimpleNamespace(path="/tmp/robot.db")
    node._call_teach_mode = lambda _, enabled: (not enabled, "ok")
    saved = []
    node._write_group = lambda *args: saved.append(args)

    response = node._stop_cb(
        SimpleNamespace(arm_target="left", group_name=""), SimpleNamespace()
    )

    assert response.success
    assert response.sample_count == 1
    assert response.group_name.startswith("未命名-左臂-")
    assert recorder.stopped
    assert node.active_arm is None
    assert saved[0][0] == response.group_name


class FakeRecorderNode:
    def create_subscription(self, *_args, **_kwargs):
        return object()

    def create_client(self, *_args, **_kwargs):
        return object()

    def create_timer(self, *_args, **_kwargs):
        return object()


class RecordingArm:
    """Minimal arm boundary used to exercise runner step behavior."""

    def __init__(self, status):
        self.latest_status = status
        self.published = []

    def publish(self, step_type, msg):
        self.published.append((step_type, msg))


class RecordingGoalHandle:
    def __init__(self):
        self.feedback = []

    def publish_feedback(self, feedback):
        self.feedback.append(SimpleNamespace(state=feedback.state))


class ReplayGoalHandle(RecordingGoalHandle):
    def __init__(self):
        super().__init__()
        self.request = SimpleNamespace(
            group_name="recorded-pause",
            repeat_count=1,
            arm_target="left",
        )
        self.is_cancel_requested = False
        self.terminal_state = None

    def succeed(self):
        self.terminal_state = "succeeded"

    def abort(self):
        self.terminal_state = "aborted"

    def canceled(self):
        self.terminal_state = "canceled"


def test_arm_recorder_keeps_only_arm_joints_from_leader_feedback():
    recorder = ArmRecorder(FakeRecorderNode(), "/left", 0.01, 50.0, None)
    recorder.latest = JointState(
        name=["joint1", "joint2", "agx_gripper", "left_index_finger"],
        position=[0.2, 0.3, 0.4, 0.5],
    )
    recorder.recording = True

    recorder._sample_cb()

    assert recorder.samples == [
        {
            "name": ["joint1", "joint2"],
            "position": [0.2, 0.3],
            "velocity": [],
            "effort": [],
        }
    ]


def test_runner_build_msg_ignores_gripper_fields_and_non_arm_joints():
    node = object.__new__(ActionGroupRunnerNode)
    msg = node._build_msg(
        "joint_states",
        {
            "name": ["joint_1", "finger_1"],
            "position": [0.2, 400.0],
            "gripper": {"name": ["finger_1"], "position": [400.0]},
        },
    )

    assert msg.name == ["joint_1"]
    assert list(msg.position) == [0.2]


def test_runner_keeps_status_based_reach_wait_for_explicit_steps():
    """Explicit waits still use the arm's reached status."""
    node = object.__new__(ActionGroupRunnerNode)
    node.poll_interval_sec = 0.001
    arm = SimpleNamespace(latest_status=AgxArmStatus(arm_status=0, motion_status=0))

    assert node._wait_reach(arm, 0.01)


def test_runner_keeps_explicit_wait_reach_timeout_fatal():
    """A hand-authored reach wait must still abort when the arm does not arrive."""
    node = object.__new__(ActionGroupRunnerNode)
    node.default_step_timeout_sec = 0.0
    node.stream_step_interval_sec = 0.0
    node.poll_interval_sec = 0.001
    node.left_arm = RecordingArm(AgxArmStatus(arm_status=0, motion_status=1))
    node.right_arm = None
    goal_handle = RecordingGoalHandle()

    with pytest.raises(RuntimeError, match="step_226 did not reach target"):
        node._run_single_step(
            goal_handle=goal_handle,
            step={
                "name": ["joint1"],
                "position": [1.0],
                "wait_reach": True,
                "timeout_sec": 0.0,
                "hold_sec": 0.0,
            },
            step_index=225,
            total_steps=535,
            cycle=1,
            total_cycles=1,
            arm_target="left",
        )


def test_recorded_static_samples_replay_without_pause_reach_wait(monkeypatch):
    """Recorded dwell frames must not inject an eight-second reach wait."""
    node = object.__new__(ActionGroupRunnerNode)
    node.default_step_timeout_sec = 8.0
    node.stream_step_interval_sec = 0.02
    node.poll_interval_sec = 0.001
    node.left_arm = RecordingArm(AgxArmStatus(arm_status=0, motion_status=1))
    node.right_arm = None
    node._stop_requested = False
    node._running_goal = False
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda *_args: None,
        error=lambda *_args: None,
    )
    samples = [
        {
            "name": ["joint1"],
            "position": [1.0],
            "hold_sec": 0.02,
            "wait_reach": False,
        }
        for _ in range(26)
    ]
    node._resolve_group = lambda _name: {
        "arm_target": "left",
        "repeat_count": 1,
        "samples": samples,
    }
    goal_handle = ReplayGoalHandle()
    sleep_durations = []
    monkeypatch.setattr(runner_module.time, "sleep", sleep_durations.append)

    result = node._execute_callback(goal_handle)

    assert result.success
    assert goal_handle.terminal_state == "succeeded"
    assert len(node.left_arm.published) == 26
    assert sum(item.state == "hold" for item in goal_handle.feedback) == 26
    assert sleep_durations == [0.02] * 26
