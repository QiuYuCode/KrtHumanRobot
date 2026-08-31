import threading
from types import SimpleNamespace

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
