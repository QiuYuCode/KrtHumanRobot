#!/usr/bin/env python3
"""Headless end-to-end planning and mock execution check."""

import math
import os
import signal
import subprocess
import time

import rclpy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import (
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def wait_future(node, future, timeout=15.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("ROS request timed out")
    return future.result()


def main():
    command = [
        "ros2",
        "launch",
        "krthumanrobot_moveit_config",
        "demo.launch.py",
        "launch_rviz:=false",
    ]
    with open(
        "/tmp/krt_moveit_smoke_launch.log", "w", encoding="utf-8"
    ) as log:
        launch = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            rclpy.init()
            node = rclpy.create_node("krt_moveit_smoke")
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, node)
            states = []
            sub = node.create_subscription(
                JointState, "/joint_states", states.append, 10
            )
            validity = node.create_client(
                GetStateValidity, "/check_state_validity"
            )
            planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
            fk = node.create_client(GetPositionFK, "/compute_fk")
            ik = node.create_client(GetPositionIK, "/compute_ik")
            executor = ActionClient(
                node, ExecuteTrajectory, "/execute_trajectory"
            )
            if not validity.wait_for_service(timeout_sec=30):
                raise TimeoutError("state validity service missing")
            if not planner.wait_for_service(timeout_sec=10):
                raise TimeoutError("planner service missing")
            if not fk.wait_for_service(
                timeout_sec=10
            ) or not ik.wait_for_service(timeout_sec=10):
                raise TimeoutError("kinematics service missing")
            if not executor.wait_for_server(timeout_sec=10):
                raise TimeoutError("execution action missing")
            deadline = time.monotonic() + 10
            while not states and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            if not states:
                raise TimeoutError("joint states missing")
            current = states[-1]
            for side in ("left", "right"):
                for index in range(1, 8):
                    name = f"{side}_arm_link{index}_joint"
                    expected = math.pi / 2.0 if index == 2 else 0.0
                    actual = current.position[current.name.index(name)]
                    assert abs(actual - expected) < 1e-6, (
                        f"{name} initial state {actual} != {expected}"
                    )
            print("home joint state verified", flush=True)
            deadline = time.monotonic() + 5
            while not tf_buffer.can_transform(
                "base_footprint", "base_link", rclpy.time.Time()
            ) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            transform = tf_buffer.lookup_transform(
                "base_footprint", "base_link", rclpy.time.Time()
            ).transform
            assert abs(transform.translation.z - 0.66769819) < 1e-6
            assert abs(transform.translation.x) < 1e-9
            assert abs(transform.translation.y) < 1e-9
            assert abs(transform.rotation.w - 1.0) < 1e-9
            print("ground TF verified: base_link z=0.66769819 m", flush=True)
            current = states[-1]
            state = RobotState()
            state.joint_state = current
            check = GetStateValidity.Request()
            check.robot_state = state
            # Include fixed links too; group-only checks missed head/chassis
            # contact and allowed RViz to show a globally colliding robot.
            check.group_name = ""
            result = wait_future(node, validity.call_async(check))
            if not result.valid:
                pairs = [
                    (c.contact_body_1, c.contact_body_2)
                    for c in result.contacts
                ]
                raise AssertionError(f"home state in collision: {pairs[:20]}")
            print("home state valid", flush=True)

            for side in ("left", "right"):
                tip = f"{side}_hand_base_link"
                fk_request = GetPositionFK.Request()
                fk_request.header.frame_id = "base_link"
                fk_request.fk_link_names = [tip]
                fk_request.robot_state = state
                fk_result = wait_future(node, fk.call_async(fk_request))
                assert fk_result.error_code.val == 1, f"{side} FK failed"
                ik_request = GetPositionIK.Request()
                ik_request.ik_request.group_name = f"{side}_arm"
                ik_request.ik_request.robot_state = state
                ik_request.ik_request.ik_link_name = tip
                ik_request.ik_request.pose_stamped = fk_result.pose_stamped[0]
                ik_request.ik_request.avoid_collisions = True
                ik_request.ik_request.timeout.sec = 2
                ik_result = wait_future(node, ik.call_async(ik_request))
                assert ik_result.error_code.val == 1, (
                    f"{side} IK failed: {ik_result.error_code.val}"
                )
                print(f"{side}_arm IK succeeded", flush=True)

            for group, sides, target in (
                ("left_arm", ("left",), "offset"),
                ("right_arm", ("right",), "offset"),
                ("both_arms", ("left", "right"), "offset"),
                ("both_arms", ("left", "right"), "zero"),
                ("both_arms", ("left", "right"), "home"),
            ):
                current = states[-1]
                request = GetMotionPlan.Request()
                motion = request.motion_plan_request
                motion.group_name = group
                motion.start_state.is_diff = True
                motion.allowed_planning_time = 5.0
                motion.num_planning_attempts = 2
                motion.max_velocity_scaling_factor = 0.2
                motion.max_acceleration_scaling_factor = 0.2
                goal = Constraints()
                goal_state = RobotState()
                goal_state.joint_state.name = list(current.name)
                goal_state.joint_state.position = list(current.position)
                targets = {}
                for side in sides:
                    for index in range(1, 8):
                        name = f"{side}_arm_link{index}_joint"
                        position = current.position[current.name.index(name)]
                        if target == "zero":
                            position = 0.0
                        elif target == "home":
                            position = math.pi / 2.0 if index == 2 else 0.0
                        elif index == 7:
                            position += 0.05
                        targets[name] = position
                for name, position in targets.items():
                    goal_state.joint_state.position[
                        current.name.index(name)
                    ] = position
                    goal.joint_constraints.append(
                        JointConstraint(
                            joint_name=name,
                            position=position,
                            tolerance_above=0.0001,
                            tolerance_below=0.0001,
                            weight=1.0,
                        )
                    )
                check.robot_state = goal_state
                result = wait_future(node, validity.call_async(check))
                if not result.valid:
                    pairs = [
                        (c.contact_body_1, c.contact_body_2)
                        for c in result.contacts
                    ]
                    raise AssertionError(
                        f"{group} goal in collision: {pairs[:20]}"
                    )
                motion.goal_constraints = [goal]
                response = wait_future(
                    node, planner.call_async(request), timeout=20
                )
                code = response.motion_plan_response.error_code.val
                if code != 1:
                    raise AssertionError(f"{group} planning failed: {code}")
                trajectory = response.motion_plan_response.trajectory
                assert trajectory.joint_trajectory.points, group
                print(f"{group}/{target} planned", flush=True)
                execute_goal = ExecuteTrajectory.Goal()
                execute_goal.trajectory = trajectory
                handle = wait_future(
                    node, executor.send_goal_async(execute_goal)
                )
                assert handle.accepted, "execution rejected"
                execution = wait_future(
                    node, handle.get_result_async(), timeout=60
                )
                assert execution.result.error_code.val == 1, execution.result
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if all(
                        abs(
                            states[-1].position[states[-1].name.index(name)]
                            - position
                        )
                        < 0.02
                        for name, position in targets.items()
                    ):
                        break
                else:
                    raise AssertionError(
                        f"{group} state did not follow trajectory"
                    )
                print(
                    f"{group}/{target} simulated execution succeeded",
                    flush=True,
                )
            node.destroy_subscription(sub)
            tf_listener.unregister()
            node.destroy_node()
            rclpy.shutdown()
        finally:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()


if __name__ == "__main__":
    main()
