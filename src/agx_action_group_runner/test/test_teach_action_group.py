import threading
from types import SimpleNamespace

from agx_arm_msgs.msg import AgxArmStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

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


def test_arm_recorder_captures_arm_and_hand_on_one_tick():
    recorder = ArmRecorder(FakeRecorderNode(), "/left", 0.01, 50.0, None)
    recorder.latest = JointState(name=["joint_1"], position=[0.2])
    recorder.latest_hand = JointState(
        name=["finger_1", "finger_2", "finger_3"], position=[1.0, 2.0, 3.0]
    )
    recorder.recording = True

    recorder._sample_cb()

    assert recorder.samples == [
        {
            "name": ["joint_1"],
            "position": [0.2],
            "velocity": [],
            "effort": [],
            "gripper": {
                "source": "hands_control",
                "name": ["finger_1", "finger_2", "finger_3"],
                "position": [1.0, 2.0, 3.0],
            },
        }
    ]


def test_runner_build_msg_merges_optional_gripper():
    node = object.__new__(ActionGroupRunnerNode)
    msg = node._build_msg(
        "joint_states",
        {
            "name": ["joint_1"],
            "position": [0.2],
            "gripper": {"name": ["finger_1"], "position": [400]},
        },
    )

    assert msg.name == ["joint_1", "finger_1"]
    assert list(msg.position) == [0.2, 400.0]


def test_runner_marks_sustained_near_static_samples_as_pause_start():
    """A recorded dwell must wait for the arm before replaying its duration."""
    node = object.__new__(ActionGroupRunnerNode)
    node.stream_step_interval_sec = 0.02
    node.pause_min_duration_sec = 0.5
    node.pause_joint_range_rad = 0.002
    steps = [
        {"name": ["joint1"], "position": [0.0], "hold_sec": 0.02},
        {"name": ["joint1"], "position": [0.03], "hold_sec": 0.02},
    ]
    steps.extend(
        {
            "name": ["joint1"],
            "position": [1.0 + offset],
            "hold_sec": 0.02,
        }
        for offset in [0.0, 0.0005, -0.0005, 0.0008, -0.0008] * 5
    )
    steps.append({"name": ["joint1"], "position": [1.03], "hold_sec": 0.02})

    assert node._pause_start_indices(steps) == {2}


def test_runner_does_not_treat_stale_idle_status_as_reached_pause_target():
    """A dwell may start only after the feedback joints reach its target."""
    node = object.__new__(ActionGroupRunnerNode)
    node.poll_interval_sec = 0.001
    node.pause_reach_tolerance_rad = 0.02
    arm = SimpleNamespace(
        latest_status=AgxArmStatus(arm_status=0, motion_status=0),
        latest_joint_states=JointState(name=["joint1"], position=[0.7]),
    )
    target = JointState(name=["joint1"], position=[1.0])

    assert not node._wait_reach(arm, 0.01, target)


def test_runner_requires_reach_only_at_start_of_detected_pause():
    """Only the first frame of a dwell waits; its remaining frames keep 50 Hz."""
    node = object.__new__(ActionGroupRunnerNode)
    node.stream_step_interval_sec = 0.02
    node.pause_min_duration_sec = 0.06
    node.pause_joint_range_rad = 0.002
    steps = [
        {"name": ["joint1"], "position": [0.0], "hold_sec": 0.02},
        {"name": ["joint1"], "position": [1.0], "hold_sec": 0.02},
        {"name": ["joint1"], "position": [1.0005], "hold_sec": 0.02},
        {"name": ["joint1"], "position": [0.9995], "hold_sec": 0.02},
        {"name": ["joint1"], "position": [1.0008], "hold_sec": 0.02},
        {"name": ["joint1"], "position": [1.03], "hold_sec": 0.02},
    ]

    replay_steps = node._with_pause_waits(steps)

    assert replay_steps[1]["wait_reach"] is True
    assert "wait_reach" not in replay_steps[2]
    assert "wait_reach" not in steps[1]


def test_runner_keeps_status_based_reach_wait_for_pose_steps():
    """Non-joint legacy steps must not require JointState feedback fields."""
    node = object.__new__(ActionGroupRunnerNode)
    node.poll_interval_sec = 0.001
    arm = SimpleNamespace(
        latest_status=AgxArmStatus(arm_status=0, motion_status=0),
        latest_joint_states=None,
    )

    assert node._wait_reach(arm, 0.01, PoseStamped())
