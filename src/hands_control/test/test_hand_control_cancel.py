import threading
from types import SimpleNamespace

from rclpy.action import CancelResponse

from hands_control.hand_control_server import HandControlServer


class FakeHand:
    def clear_error(self, _device_id):
        pass

    def move_finger(self, *_args):
        pass


class FakeGoalHandle:
    def __init__(self):
        self.request = SimpleNamespace(
            adapter_index=0, finger_id=0, position=500,
            speed=300, force=85, wait_time=10,
        )
        self.is_cancel_requested = True
        self.was_canceled = False
        self.was_succeeded = False

    def publish_feedback(self, _feedback):
        pass

    def canceled(self):
        self.was_canceled = True

    def succeed(self):
        self.was_succeeded = True

    def abort(self):
        pass


def test_hand_control_accepts_cancel_requests():
    assert HandControlServer._cancel_callback(None) == CancelResponse.ACCEPT


def test_hand_control_stops_feedback_loop_when_canceled(monkeypatch):
    monkeypatch.setattr("hands_control.hand_control_server.time.sleep", lambda _delay: None)
    server = SimpleNamespace(
        adapter_index=0, device_id=1, hand_name="左手", hand=FakeHand(),
        comm_lock=threading.Lock(),
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None,
                                           error=lambda *_args: None),
        _read_all_positions=lambda _hand, _device_id: [1, 2, 3],
    )
    goal_handle = FakeGoalHandle()

    result = HandControlServer._execute_hand_control(server, goal_handle)

    assert result.success is False
    assert result.message == "左手 控制已取消"
    assert goal_handle.was_canceled is True
    assert goal_handle.was_succeeded is False
