import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from agx_action_group_interfaces.action import RunActionGroup
from agx_arm_msgs.msg import AgxArmStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


TEACH_STATUS_START_RECORD = 1
TEACH_STATUS_END_RECORD = 2
CTRL_MODE_TEACH = 2


class TeachRecordReplayNode(Node):
    def __init__(self):
        super().__init__("teach_record_replay")

        self.declare_parameter("arm_namespace", "/left_arm")
        self.declare_parameter("sample_rate_hz", 20.0)
        self.declare_parameter("min_joint_delta_rad", 0.01)
        self.declare_parameter("auto_replay_once", True)
        self.declare_parameter("replay_group_prefix", "teach_replay")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("prefer_offline_teach", True)

        self.arm_namespace = self._normalize_ns(str(self.get_parameter("arm_namespace").value))
        self.sample_period_sec = 1.0 / float(self.get_parameter("sample_rate_hz").value)
        self.min_joint_delta_rad = float(self.get_parameter("min_joint_delta_rad").value)
        self.auto_replay_once = bool(self.get_parameter("auto_replay_once").value)
        self.replay_group_prefix = str(self.get_parameter("replay_group_prefix").value)
        self.prefer_offline_teach = bool(self.get_parameter("prefer_offline_teach").value)

        output_dir = str(self.get_parameter("output_dir").value).strip()
        if not output_dir:
            output_dir = str(Path.cwd() / "teach_records")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._recording = False
        self._last_sample_time = 0.0
        self._last_positions: Optional[List[float]] = None
        self._joint_names: List[str] = []
        self._samples: List[List[float]] = []
        self._last_teach_status: Optional[int] = None

        self._action_client = ActionClient(self, RunActionGroup, "/run_action_group")

        self.create_subscription(
            AgxArmStatus, self._topic("feedback/arm_status"), self._arm_status_cb, 10
        )
        self.create_subscription(
            JointState, self._topic("feedback/joint_states"), self._joint_state_cb, 10
        )

        self.get_logger().info(
            f"Teach recorder ready on {self.arm_namespace or '/'}; output_dir={self.output_dir}"
        )
        if self.prefer_offline_teach:
            self.get_logger().warn(
                "prefer_offline_teach=true, but no offline-trajectory ROS API exists now. "
                "Fallback to sampled joint_states recording."
            )

    @staticmethod
    def _normalize_ns(namespace: str) -> str:
        ns = namespace.strip()
        if not ns:
            return ""
        if not ns.startswith("/"):
            ns = "/" + ns
        return ns.rstrip("/")

    def _topic(self, suffix: str) -> str:
        if self.arm_namespace:
            return f"{self.arm_namespace}/{suffix}"
        return "/" + suffix

    def _arm_status_cb(self, msg: AgxArmStatus):
        teach_status = int(msg.teach_status)
        if self._last_teach_status == teach_status:
            return
        self._last_teach_status = teach_status

        is_start = teach_status == TEACH_STATUS_START_RECORD or (
            msg.ctrl_mode == CTRL_MODE_TEACH and not self._recording
        )
        is_end = teach_status == TEACH_STATUS_END_RECORD and self._recording

        if is_start and not self._recording:
            self._start_record()
            return
        if is_end:
            self._stop_record_and_replay()

    def _start_record(self):
        self._recording = True
        self._samples = []
        self._last_positions = None
        self._last_sample_time = 0.0
        self.get_logger().info("Teach record started.")

    def _stop_record_and_replay(self):
        self._recording = False
        self.get_logger().info(f"Teach record stopped. sample_count={len(self._samples)}")
        if not self._samples or not self._joint_names:
            self.get_logger().warn("No valid samples captured, skip replay.")
            return

        generated = self._generate_group_files()
        if generated is None:
            self.get_logger().error("Failed to generate replay files.")
            return

        group_name, group_file = generated
        self.get_logger().info(f"Generated group '{group_name}' at {group_file}")
        if self.auto_replay_once:
            self._send_replay_goal(group_name)

    def _joint_state_cb(self, msg: JointState):
        if not self._recording:
            return
        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            return

        now = time.time()
        if now - self._last_sample_time < self.sample_period_sec:
            return
        self._last_sample_time = now

        positions = [float(x) for x in msg.position]
        if self._last_positions is not None:
            max_delta = max(abs(a - b) for a, b in zip(positions, self._last_positions))
            if max_delta < self.min_joint_delta_rad:
                return

        self._joint_names = list(msg.name)
        self._samples.append(positions)
        self._last_positions = positions

    def _generate_group_files(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = self.output_dir / f"{self.replay_group_prefix}_{stamp}"
        payload_dir = run_dir / "steps"
        payload_dir.mkdir(parents=True, exist_ok=True)

        steps = []
        for idx, positions in enumerate(self._samples, start=1):
            payload_name = f"step_{idx:04d}.yaml"
            payload_path = payload_dir / payload_name
            payload_data = {
                "name": self._joint_names,
                "position": positions,
                "velocity": [],
                "effort": [],
            }
            with payload_path.open("w", encoding="utf-8") as f:
                yaml.safe_dump(payload_data, f, sort_keys=False, allow_unicode=False)
            steps.append(
                {
                    "name": f"teach_step_{idx:04d}",
                    "type": "move_j",
                    "payload_file": f"steps/{payload_name}",
                    "timeout_sec": 6.0,
                }
            )

        group_name = f"{self.replay_group_prefix}_{stamp}"
        group_data = {
            "groups": {
                group_name: {
                    "repeat_count": 1,
                    "steps": steps,
                }
            }
        }
        group_file = run_dir / "action_groups.yaml"
        with group_file.open("w", encoding="utf-8") as f:
            yaml.safe_dump(group_data, f, sort_keys=False, allow_unicode=False)

        latest_link = self.output_dir / "latest_action_groups.yaml"
        with latest_link.open("w", encoding="utf-8") as f:
            yaml.safe_dump(group_data, f, sort_keys=False, allow_unicode=False)

        return group_name, group_file

    def _send_replay_goal(self, group_name: str):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("/run_action_group action server not available.")
            return

        goal = RunActionGroup.Goal()
        goal.group_name = group_name
        goal.repeat_count = 1
        goal.arm_target = "left" if "left" in self.arm_namespace else "right"

        self.get_logger().info(
            f"Send replay goal: group={goal.group_name}, arm_target={goal.arm_target}"
        )
        future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[Replay] cycle {fb.current_cycle}/{fb.total_cycles}, "
            f"step {fb.current_step}/{fb.total_steps}, state={fb.state}, step={fb.step_name}"
        )

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Replay goal rejected.")
            return
        self.get_logger().info("Replay goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_replay_result)

    def _on_replay_result(self, future):
        result_msg = future.result()
        if result_msg is None:
            self.get_logger().error("Replay result future failed.")
            return
        result = result_msg.result
        self.get_logger().info(
            f"Replay done. success={result.success}, cycles={result.finished_cycles}, "
            f"msg={result.message}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TeachRecordReplayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

