from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped

from ranger_nav.waypoint_manager import WaypointNode, build_parser


def test_control_cli_accepts_session_service():
    args = build_parser().parse_args([
        "--control-prefix", "/cruise/test", "control", "pause",
    ])
    assert args.operation == "pause"


def test_pause_cancels_navigation_and_waits_for_terminal_result():
    node = SimpleNamespace(
        pause_requested=True, cancel_requested=False, paused=False,
        get_logger=lambda: Mock(),
    )
    result = SimpleNamespace(status=5)
    handle = Mock()
    handle.cancel_goal_async.return_value = object()
    handle.get_result_async.return_value = object()
    node.wait_future = Mock(side_effect=[SimpleNamespace(goals_canceling=[1]), result])
    assert WaypointNode.cancel_goal(node, handle) is result
    handle.cancel_goal_async.assert_called_once()
    assert node.wait_future.call_count == 2


def test_resume_rejected_until_pause_confirmed():
    node = SimpleNamespace(pause_requested=True, paused=False, cancel_requested=False)
    response = SimpleNamespace(success=False, message="")
    WaypointNode.control_request(node, "resume", response)
    assert not response.success
    assert node.pause_requested


@pytest.mark.parametrize("result", [None, SimpleNamespace(status=2)])
def test_cancel_requires_terminal_result(result):
    node = SimpleNamespace(
        wait_future=Mock(side_effect=[SimpleNamespace(goals_canceling=[1]), result]),
    )
    with pytest.raises(RuntimeError, match="未确认目标停止"):
        WaypointNode.cancel_goal(node, Mock())


def test_routine_pause_waits_for_completion_without_cancel(monkeypatch):
    future = Mock()
    future.done.side_effect = [False, True, True]
    result = SimpleNamespace(status=4)
    future.result.return_value = result
    handle = Mock()
    handle.get_result_async.return_value = future
    node = SimpleNamespace(pause_requested=True, cancel_requested=False)
    monkeypatch.setattr("ranger_nav.waypoint_manager.rclpy.ok", lambda: True)
    monkeypatch.setattr("ranger_nav.waypoint_manager.rclpy.spin_once", lambda *a, **k: None)
    assert WaypointNode.wait_action(node, handle, pause_navigation=False) is result
    handle.cancel_goal_async.assert_not_called()


def test_navigation_acceptance_timeout_is_not_successful_cancellation():
    node = Mock(cancel_requested=True)
    pose = PoseStamped()
    node.get_clock.return_value.now.return_value.to_msg.return_value = pose.header.stamp
    node.wait_future.return_value = None
    with pytest.raises(RuntimeError, match="未确认导航目标"):
        WaypointNode.send_nav_attempt(node, pose, "入口", "完整位姿")


def test_routine_acceptance_timeout_is_not_successful_cancellation():
    node = Mock(cancel_requested=True)
    node.wait_future.return_value = None
    with pytest.raises(RuntimeError, match="未确认 routine 目标"):
        WaypointNode.run_task(node, SimpleNamespace(name="入口", routine="挥手"))


def test_shutdown_during_action_is_not_successful_cancellation(monkeypatch):
    monkeypatch.setattr("ranger_nav.waypoint_manager.rclpy.ok", lambda: False)
    handle = Mock()
    handle.get_result_async.return_value.done.return_value = False
    with pytest.raises(RuntimeError, match="未确认目标终止"):
        WaypointNode.wait_action(Mock(), handle, pause_navigation=True)
