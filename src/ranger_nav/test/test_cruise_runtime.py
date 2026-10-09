"""Isolated ROS action/service checks; never connect this test to a robot domain."""

import os
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from ranger_nav.waypoint_manager import Waypoint, WaypointNode, build_parser


pytestmark = pytest.mark.skipif(
    os.environ.get("KRT_CRUISE_RUNTIME_TEST") != "1",
    reason="requires an explicitly isolated ROS domain",
)


@pytest.mark.parametrize("cancel_while_paused", [False, True])
def test_navigation_pause_resume_preserves_rounds(tmp_path, cancel_while_paused):
    assert os.environ.get("ROS_DOMAIN_ID") == "91"
    assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    rclpy.init()
    server = Node("cruise_test_nav")
    controller = Node("cruise_test_controller")
    controller_executor = SingleThreadedExecutor()
    controller_executor.add_node(controller)
    release = threading.Event()
    received = threading.Event()
    goals = []
    canceled = []

    def execute(handle):
        x = handle.request.pose.pose.position.x
        goals.append(x)
        received.set()
        while not release.wait(0.01):
            if handle.is_cancel_requested:
                canceled.append(x)
                handle.canceled()
                return NavigateToPose.Result()
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(
        server, NavigateToPose, "/cruise_test/navigate", execute,
        cancel_callback=lambda handle: CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup(),
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)
    server_thread = threading.Thread(target=executor.spin)
    server_thread.start()
    args = build_parser().parse_args([
        "--robot-db", str(tmp_path / "robot.db"),
        "--navigate-action", "/cruise_test/navigate",
        "--control-prefix", "/cruise_test/control",
        "--accuracy-report", "", "cruise",
    ])
    worker = WaypointNode(args)
    waypoints = []
    for x in (1.0, 2.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        waypoints.append(Waypoint(str(x), pose))
    worker.select_waypoints = lambda names: waypoints
    worker.arrival_reached = lambda wp: True
    worker.record_accuracy = lambda wp: None
    outcome = []
    worker_thread = threading.Thread(
        target=lambda: outcome.append(worker.cruise([], repeat=2, loop=False)),
    )

    def call(operation):
        client = controller.create_client(Trigger, f"/cruise_test/control/{operation}")
        try:
            assert client.wait_for_service(timeout_sec=3)
            future = client.call_async(Trigger.Request())
            controller_executor.spin_until_future_complete(future, timeout_sec=3)
            assert future.done(), operation
            return future.result()
        finally:
            controller.destroy_client(client)

    def cli(operation):
        result = subprocess.run([
            sys.executable, "-m", "ranger_nav.waypoint_manager",
            "--robot-db", str(tmp_path / "robot.db"),
            "--control-prefix", "/cruise_test/control", "control", operation,
        ], capture_output=True, text=True, timeout=30, check=False)
        assert result.returncode == 0, result.stdout + result.stderr

    worker_thread.start()
    try:
        assert received.wait(5)
        cli("pause")
        deadline = time.monotonic() + 5
        while call("status").message != "paused":
            assert time.monotonic() < deadline
        assert canceled == [1.0]
        assert goals == [1.0]
        assert call("pause").success  # Repeated stop is harmless.
        if cancel_while_paused:
            assert call("cancel").success
        else:
            release.set()
            cli("resume")
        worker_thread.join(5)
        assert not worker_thread.is_alive()
        assert outcome == [0]
        assert goals == ([1.0] if cancel_while_paused else [1.0, 1.0, 2.0, 1.0, 2.0])
    finally:
        worker.cancel_requested = True
        worker.pause_requested = False
        release.set()
        worker_thread.join(5)
        executor.shutdown(timeout_sec=5)
        server_thread.join(5)
        action.destroy()
        controller_executor.shutdown()
        worker.destroy_node()
        controller.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
