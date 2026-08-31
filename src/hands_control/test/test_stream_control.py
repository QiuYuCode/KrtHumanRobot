import threading
from types import SimpleNamespace

from sensor_msgs.msg import JointState

from hands_control.hand_control_server import HandControlServer


class FakeHand:
    def __init__(self):
        self.calls = []

    def move_finger(self, *args):
        self.calls.append(args)


def test_stream_joint_state_maps_three_fingers_and_ignores_other_names():
    node = object.__new__(HandControlServer)
    node._stopping = False
    node.hand = FakeHand()
    node.device_id = 7
    node.comm_lock = threading.Lock()
    node.hand_name = "左手"
    node.get_parameter = lambda name: SimpleNamespace(
        value={"stream_speed": 500, "stream_force": 85, "stream_wait_time": 0}[name]
    )
    node.get_logger = lambda: SimpleNamespace(
        warning=lambda *_args, **_kwargs: None,
        error=lambda *_args, **_kwargs: None,
    )

    node._joint_state_callback(
        JointState(
            name=["finger_1", "other", "finger_3"], position=[100, 200, 900]
        )
    )

    assert [call[1:3] for call in node.hand.calls] == [(1, 100), (3, 900)]
