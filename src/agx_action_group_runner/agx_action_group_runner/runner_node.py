import copy
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from agx_action_group_interfaces.action import RunActionGroup
from agx_arm_msgs.msg import AgxArmStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


SUPPORTED_STEP_TYPES = {"move_j", "move_p", "move_l", "move_c", "joint_states"}
SUPPORTED_ARM_TARGETS = {"left", "right", "both"}


class ArmIo:
    def __init__(self, namespace: str, node: Node):
        self.namespace = self._normalize_ns(namespace)
        self.latest_status: Optional[AgxArmStatus] = None
        self.pub_move_j = node.create_publisher(
            JointState, self._topic("control/move_j"), 10
        )
        self.pub_move_p = node.create_publisher(
            PoseStamped, self._topic("control/move_p"), 10
        )
        self.pub_move_l = node.create_publisher(
            PoseStamped, self._topic("control/move_l"), 10
        )
        self.pub_move_c = node.create_publisher(PoseArray, self._topic("control/move_c"), 10)
        self.pub_joint_states = node.create_publisher(
            JointState, self._topic("control/joint_states"), 10
        )
        self.sub_status = node.create_subscription(
            AgxArmStatus, self._topic("feedback/arm_status"), self._status_cb, 10
        )

    def _status_cb(self, msg: AgxArmStatus):
        self.latest_status = msg

    @staticmethod
    def _normalize_ns(namespace: str) -> str:
        if not namespace:
            return ""
        ns = namespace.strip()
        if not ns.startswith("/"):
            ns = "/" + ns
        return ns.rstrip("/")

    def _topic(self, suffix: str) -> str:
        if self.namespace:
            return f"{self.namespace}/{suffix}"
        return "/" + suffix

    def publish(self, step_type: str, msg):
        if step_type == "move_j":
            self.pub_move_j.publish(msg)
        elif step_type == "move_p":
            self.pub_move_p.publish(msg)
        elif step_type == "move_l":
            self.pub_move_l.publish(msg)
        elif step_type == "move_c":
            self.pub_move_c.publish(msg)
        elif step_type == "joint_states":
            self.pub_joint_states.publish(msg)
        else:
            raise ValueError(f"Unsupported step type: {step_type}")


class ActionGroupRunnerNode(Node):
    def __init__(self):
        super().__init__("agx_action_group_runner")

        self.declare_parameter("groups_file", "")
        self.declare_parameter("default_step_timeout_sec", 8.0)
        self.declare_parameter("poll_interval_sec", 0.05)
        self.declare_parameter("stream_step_interval_sec", 0.02)
        self.declare_parameter("left_namespace", "/left_arm")
        self.declare_parameter("right_namespace", "/right_arm")

        groups_file = self.get_parameter("groups_file").value
        if not groups_file:
            self.get_logger().error("Parameter 'groups_file' is required.")
            raise RuntimeError("groups_file parameter is empty")

        self.groups_file = Path(groups_file)
        self.default_step_timeout_sec = float(
            self.get_parameter("default_step_timeout_sec").value
        )
        self.poll_interval_sec = float(self.get_parameter("poll_interval_sec").value)
        self.stream_step_interval_sec = float(
            self.get_parameter("stream_step_interval_sec").value
        )

        left_namespace = str(self.get_parameter("left_namespace").value)
        right_namespace = str(self.get_parameter("right_namespace").value)

        self.left_arm = ArmIo(left_namespace, self)
        self.right_arm = ArmIo(right_namespace, self)
        self._running_goal = False

        self._action_server = ActionServer(
            self,
            RunActionGroup,
            "run_action_group",
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
        )

        self.get_logger().info(
            f"Action group runner started. groups_file={self.groups_file}, "
            f"left_ns={self.left_arm.namespace or '/'}, right_ns={self.right_arm.namespace or '/'}"
        )

    def _goal_callback(self, goal_request: RunActionGroup.Goal):
        if self._running_goal:
            self.get_logger().warn("Reject new goal because another goal is running.")
            return GoalResponse.REJECT
        if goal_request.arm_target not in SUPPORTED_ARM_TARGETS:
            self.get_logger().warn(
                f"Invalid arm_target '{goal_request.arm_target}', "
                f"allowed={sorted(SUPPORTED_ARM_TARGETS)}"
            )
            return GoalResponse.REJECT
        if not goal_request.group_name:
            self.get_logger().warn("Reject goal because group_name is empty.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    def _load_groups(self) -> dict:
        if not self.groups_file.exists():
            raise FileNotFoundError(f"groups file not found: {self.groups_file}")
        with self.groups_file.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        groups = data.get("groups")
        if not isinstance(groups, dict):
            raise ValueError("YAML must contain a mapping at top-level key 'groups'")
        return groups

    def _resolve_group(self, group_name: str) -> dict:
        groups = self._load_groups()
        if group_name not in groups:
            raise KeyError(f"group '{group_name}' not found in {self.groups_file}")
        group = groups[group_name]
        if not isinstance(group, dict):
            raise ValueError(f"group '{group_name}' must be a map")
        if "steps" not in group or not isinstance(group["steps"], list):
            raise ValueError(f"group '{group_name}' must provide list field 'steps'")
        return group

    def _read_payload(self, rel_file: str) -> dict:
        payload_path = (self.groups_file.parent / rel_file).resolve()
        with payload_path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _build_msg(self, step_type: str, payload: dict):
        if step_type == "move_j" or step_type == "joint_states":
            msg = JointState()
            msg.name = payload.get("name", [])
            msg.position = payload.get("position", [])
            msg.velocity = payload.get("velocity", [])
            msg.effort = payload.get("effort", [])
            return msg
        if step_type == "move_p" or step_type == "move_l":
            msg = PoseStamped()
            pose = payload.get("pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            msg.pose.position.x = float(pos.get("x", 0.0))
            msg.pose.position.y = float(pos.get("y", 0.0))
            msg.pose.position.z = float(pos.get("z", 0.0))
            msg.pose.orientation.x = float(ori.get("x", 0.0))
            msg.pose.orientation.y = float(ori.get("y", 0.0))
            msg.pose.orientation.z = float(ori.get("z", 0.0))
            msg.pose.orientation.w = float(ori.get("w", 1.0))
            return msg
        if step_type == "move_c":
            msg = PoseArray()
            poses = payload.get("poses", [])
            if len(poses) < 3:
                raise ValueError("move_c payload requires at least 3 poses")
            for p in poses:
                ps = PoseStamped()
                pos = p.get("position", {})
                ori = p.get("orientation", {})
                ps.pose.position.x = float(pos.get("x", 0.0))
                ps.pose.position.y = float(pos.get("y", 0.0))
                ps.pose.position.z = float(pos.get("z", 0.0))
                ps.pose.orientation.x = float(ori.get("x", 0.0))
                ps.pose.orientation.y = float(ori.get("y", 0.0))
                ps.pose.orientation.z = float(ori.get("z", 0.0))
                ps.pose.orientation.w = float(ori.get("w", 1.0))
                msg.poses.append(ps.pose)
            return msg
        raise ValueError(f"Unsupported step type: {step_type}")

    def _wait_reach(self, arm: ArmIo, timeout_sec: float) -> bool:
        waited = 0.0
        while waited < timeout_sec:
            status = arm.latest_status
            if status is not None:
                if status.arm_status != 0:
                    return False
                if status.motion_status == 0:
                    return True
            time.sleep(self.poll_interval_sec)
            waited += self.poll_interval_sec
        return False

    def _run_single_step(
        self,
        goal_handle,
        step: dict,
        step_index: int,
        total_steps: int,
        cycle: int,
        total_cycles: int,
        arm_target: str,
    ):
        step_type = step.get("type")
        if step_type not in SUPPORTED_STEP_TYPES:
            raise ValueError(f"Invalid step.type: {step_type}")

        step_name = step.get("name", f"step_{step_index + 1}")
        timeout_sec = float(step.get("timeout_sec", self.default_step_timeout_sec))
        wait_reach = bool(step.get("wait_reach", total_steps <= 50))
        hold_sec = float(
            step.get(
                "hold_sec",
                self.stream_step_interval_sec if not wait_reach else 0.0,
            )
        )

        feedback = RunActionGroup.Feedback()
        feedback.current_cycle = cycle
        feedback.total_cycles = total_cycles
        feedback.current_step = step_index + 1
        feedback.total_steps = total_steps
        feedback.arm_target = arm_target
        feedback.step_name = step_name
        feedback.state = "dispatch"
        goal_handle.publish_feedback(feedback)

        if arm_target in ("left", "right"):
            file_key = "payload_file"
            if file_key not in step:
                raise ValueError(f"{step_name} needs '{file_key}'")
            payload = self._read_payload(step[file_key])
            msg = self._build_msg(step_type, payload)
            target_arm = self.left_arm if arm_target == "left" else self.right_arm
            target_arm.publish(step_type, msg)
            if wait_reach:
                ok = self._wait_reach(target_arm, timeout_sec)
                if not ok:
                    raise RuntimeError(f"{step_name} did not reach target in {timeout_sec}s")
        else:
            left_file = step.get("left_payload_file", step.get("payload_file"))
            right_file = step.get("right_payload_file", step.get("payload_file"))
            if not left_file or not right_file:
                raise ValueError(
                    f"{step_name} for both-arm target needs payload_file or "
                    "left_payload_file/right_payload_file"
                )

            left_payload = self._read_payload(left_file)
            right_payload = self._read_payload(right_file)
            left_msg = self._build_msg(step_type, left_payload)
            right_msg = self._build_msg(step_type, right_payload)
            self.left_arm.publish(step_type, left_msg)
            self.right_arm.publish(step_type, right_msg)
            if wait_reach:
                left_ok = self._wait_reach(self.left_arm, timeout_sec)
                right_ok = self._wait_reach(self.right_arm, timeout_sec)
                if not left_ok or not right_ok:
                    raise RuntimeError(f"{step_name} did not reach target on both arms")

        if hold_sec > 0.0:
            feedback.state = "hold"
            goal_handle.publish_feedback(feedback)
            time.sleep(hold_sec)

    def _execute_callback(self, goal_handle):
        self._running_goal = True
        try:
            goal = goal_handle.request
            group = self._resolve_group(goal.group_name)
            steps = copy.deepcopy(group["steps"])
            total_steps = len(steps)
            repeat_count = int(goal.repeat_count) if goal.repeat_count > 0 else int(
                group.get("repeat_count", 1)
            )
            if repeat_count <= 0:
                raise ValueError("repeat_count must be > 0")

            for cycle in range(1, repeat_count + 1):
                for step_index, step in enumerate(steps):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result = RunActionGroup.Result()
                        result.success = False
                        result.message = "Goal canceled"
                        result.finished_cycles = cycle - 1
                        result.failed_step = step.get("name", f"step_{step_index + 1}")
                        return result

                    self._run_single_step(
                        goal_handle=goal_handle,
                        step=step,
                        step_index=step_index,
                        total_steps=total_steps,
                        cycle=cycle,
                        total_cycles=repeat_count,
                        arm_target=goal.arm_target,
                    )

            goal_handle.succeed()
            result = RunActionGroup.Result()
            result.success = True
            result.message = "Completed all cycles"
            result.finished_cycles = repeat_count
            result.failed_step = ""
            return result
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"RunActionGroup failed: {exc}")
            goal_handle.abort()
            result = RunActionGroup.Result()
            result.success = False
            result.message = str(exc)
            result.finished_cycles = 0
            result.failed_step = ""
            return result
        finally:
            self._running_goal = False


def main(args=None):
    rclpy.init(args=args)
    node = ActionGroupRunnerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
