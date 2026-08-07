import threading
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from rclpy.parameter import Parameter
from lifecycle_msgs.srv import GetState

from hands_control.hand_control_server import HandControlServer
from hands_control_interfaces.action import HandControl


class FakeHand:
    def __init__(self, *_args, **_kwargs):
        self.listen_states = []
        self.realtime_states = []

    def __getattr__(self, name):
        if name.startswith("get_"):
            return lambda *_args: 0
        return lambda *_args, **_kwargs: None

    def clear_error(self, _device_id):
        pass

    def move_finger(self, *_args):
        pass

    def get_joint_degree(self, *_args):
        return 100

    def listen(self, *, enable):
        self.listen_states.append(enable)

    def enable_realtime_response(self, *, device_id, enable):
        del device_id
        self.realtime_states.append(enable)


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


def test_hand_control_lifecycle_allocates_and_deactivates_hardware(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    monkeypatch.setattr("hands_control.hand_control_server.DexHand021S", FakeHand)
    monkeypatch.setattr("hands_control.hand_control_server.DEXHAND_AVAILABLE", True)
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    node = HandControlServer()
    try:
        assert isinstance(node, LifecycleNode)
        assert not hasattr(node, "hand")
        assert node.trigger_configure().name == "SUCCESS"
        assert hasattr(node, "hand")
        hand = node.hand
        assert node.trigger_activate().name == "SUCCESS"
        assert node.trigger_deactivate().name == "SUCCESS"
        assert node.trigger_cleanup().name == "SUCCESS"
        assert node.hand is hand
        assert node._interfaces_active is False
    finally:
        node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_hand_control_reconfigure_reuses_sdk_handle(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    instances = []

    def create_hand(*args, **kwargs):
        hand = FakeHand(*args, **kwargs)
        instances.append(hand)
        return hand

    monkeypatch.setattr(
        "hands_control.hand_control_server.DexHand021S", create_hand
    )
    monkeypatch.setattr("hands_control.hand_control_server.DEXHAND_AVAILABLE", True)
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    node = HandControlServer()
    try:
        assert node.trigger_configure().name == "SUCCESS"
        assert node.trigger_activate().name == "SUCCESS"
        assert node.trigger_deactivate().name == "SUCCESS"
        assert node.trigger_cleanup().name == "SUCCESS"
        assert node.trigger_configure().name == "SUCCESS"
        assert len(instances) == 1
    finally:
        node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_connected_sdk_rejects_adapter_change(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    monkeypatch.setattr("hands_control.hand_control_server.DexHand021S", FakeHand)
    monkeypatch.setattr("hands_control.hand_control_server.DEXHAND_AVAILABLE", True)
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    node = HandControlServer()
    try:
        assert node.trigger_configure().name == "SUCCESS"
        assert node.trigger_cleanup().name == "SUCCESS"

        result = node._handle_parameter_update([
            Parameter("adapter_index", value=1)
        ])

        assert result.successful is False
        assert result.reason == "adapter_index 需要重启节点后修改"
    finally:
        node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_failed_deactivate_keeps_control_interfaces(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    monkeypatch.setattr("hands_control.hand_control_server.DexHand021S", FakeHand)
    monkeypatch.setattr("hands_control.hand_control_server.DEXHAND_AVAILABLE", True)
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    node = HandControlServer()
    try:
        assert node.trigger_configure().name == "SUCCESS"
        assert node.trigger_activate().name == "SUCCESS"
        original_listen = node.hand.listen
        node.hand.listen = lambda *, enable: (
            (_ for _ in ()).throw(RuntimeError("disable failed"))
            if not enable else original_listen(enable=enable)
        )

        assert node.trigger_deactivate().name == "FAILURE"
        assert node._interfaces_active is True
        assert node._stopping is False
    finally:
        node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_hand_launch_supports_single_side_and_autostart():
    source = (
        Path(__file__).parents[1] / "launch" / "hand_control_launch.py"
    ).read_text(encoding="utf-8")

    assert "enable_left" in source
    assert "enable_right" in source
    assert "autostart" in source
    assert "LifecycleNode" in source
    assert 'DeclareLaunchArgument("left_hand_device_id", default_value="1")' in source
    assert 'DeclareLaunchArgument("right_hand_device_id", default_value="2")' in source
    assert 'DeclareLaunchArgument("left_hand_adapter_index", default_value="1")' in source
    assert 'DeclareLaunchArgument("right_hand_adapter_index", default_value="0")' in source


def test_hand_control_executor_can_process_cancel_during_execution():
    source = (
        Path(__file__).parents[1] / "hands_control" / "hand_control_server.py"
    ).read_text(encoding="utf-8")

    assert "ReentrantCallbackGroup()" in source
    assert "MultiThreadedExecutor(num_threads=2)" in source


def _wait_future(future, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), "ROS future timed out"
    return future.result()


def test_hand_control_lifecycle_state_service_responds(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    server_node = HandControlServer()
    client_node = Node("hand_lifecycle_state_test_client")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(client_node)
    client = client_node.create_client(
        GetState, "/hand_control_server/get_state"
    )
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert client.wait_for_service(timeout_sec=2.0)
        response = _wait_future(client.call_async(GetState.Request()))
        assert response.current_state.label == "unconfigured"
    finally:
        executor.shutdown(timeout_sec=2.0)
        server_node.destroy_node()
        client_node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_static_parameter_update_reads_humble_lifecycle_state(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    node = HandControlServer()
    try:
        result = node._handle_parameter_update([
            Parameter("device_id", value=2)
        ])
        assert result.successful is True
    finally:
        node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_inflight_hand_control_goal_is_canceled_by_executor(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-logs"))
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    server_node = Node("cancel_test_server")
    client_node = Node("cancel_test_client")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server_node)
    executor.add_node(client_node)
    fake_server = SimpleNamespace(
        adapter_index=0,
        device_id=1,
        hand_name="测试手",
        hand=FakeHand(),
        comm_lock=threading.Lock(),
        _stopping=False,
        get_logger=server_node.get_logger,
    )
    fake_server._read_all_positions = lambda hand, device_id: (
        HandControlServer._read_all_positions(fake_server, hand, device_id)
    )
    action_server = ActionServer(
        server_node,
        HandControl,
        "cancel_test_hand_control",
        lambda goal: HandControlServer._execute_hand_control(fake_server, goal),
        cancel_callback=HandControlServer._cancel_callback,
        callback_group=ReentrantCallbackGroup(),
    )
    client = ActionClient(client_node, HandControl, "cancel_test_hand_control")
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        assert client.wait_for_server(timeout_sec=2.0)
        goal = HandControl.Goal()
        goal.adapter_index = 0
        goal.finger_id = 1
        goal.position = 500
        goal.speed = 300
        goal.force = 85
        goal.wait_time = 20
        goal_handle = _wait_future(client.send_goal_async(goal))
        assert goal_handle.accepted
        cancel_response = _wait_future(goal_handle.cancel_goal_async())
        assert cancel_response.goals_canceling
        result = _wait_future(goal_handle.get_result_async())
        assert result.result.success is False
        assert result.result.message == "测试手 控制已取消"
    finally:
        action_server.destroy()
        executor.shutdown(timeout_sec=2.0)
        server_node.destroy_node()
        client_node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_hand_control_stops_feedback_loop_when_canceled(monkeypatch):
    monkeypatch.setattr("hands_control.hand_control_server.time.sleep", lambda _delay: None)
    server = SimpleNamespace(
        adapter_index=0, device_id=1, hand_name="左手", hand=FakeHand(),
        comm_lock=threading.Lock(), _stopping=False,
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
