import threading
from datetime import datetime
from typing import Optional

import rclpy
from agx_action_group_interfaces.srv import StartTeach, StopTeach
from krt_task.robot_db import RobotDatabase
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class ArmRecorder:
    def __init__(
        self,
        node: Node,
        namespace: str,
        min_joint_delta_rad: float,
        callback_group: ReentrantCallbackGroup,
    ):
        self.node = node
        self.namespace = self._normalize_ns(namespace)
        self.min_joint_delta_rad = max(0.0, float(min_joint_delta_rad))
        self.latest: Optional[JointState] = None
        self.samples: list[JointState] = []
        self.recording = False
        self.last_positions: list[float] | None = None
        self.lock = threading.Lock()
        self.sub = node.create_subscription(
            JointState,
            self._topic("feedback/leader_joint_states"),
            self._joint_cb,
            10,
            callback_group=callback_group,
        )
        self.client = node.create_client(
            SetBool,
            self._topic("set_teach_mode"),
            callback_group=callback_group,
        )

    @staticmethod
    def _normalize_ns(namespace: str) -> str:
        ns = (namespace or "").strip()
        if not ns:
            return ""
        if not ns.startswith("/"):
            ns = "/" + ns
        return ns.rstrip("/")

    def _topic(self, suffix: str) -> str:
        return f"{self.namespace}/{suffix}" if self.namespace else f"/{suffix}"

    def _joint_cb(self, msg: JointState):
        self.latest = msg
        if not self.recording or not msg.name or not msg.position:
            return
        positions = list(msg.position)
        with self.lock:
            if self.last_positions is not None and self.min_joint_delta_rad > 0.0:
                if len(self.last_positions) == len(positions):
                    delta = max(abs(a - b) for a, b in zip(self.last_positions, positions))
                    if delta < self.min_joint_delta_rad:
                        return
            self.samples.append(msg)
            self.last_positions = positions

    def start(self):
        with self.lock:
            self.samples = []
            self.last_positions = None
            self.recording = True

    def stop(self) -> list[JointState]:
        with self.lock:
            self.recording = False
            return list(self.samples)


class TeachActionGroupNode(Node):
    def __init__(self):
        super().__init__("teach_action_group")
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter("robot_db", "~/maps/krt_robot.db")
        self.declare_parameter("legacy_groups_file", "")
        self.declare_parameter("left_namespace", "/left")
        self.declare_parameter("right_namespace", "/right")
        self.declare_parameter("step_timeout_sec", 8.0)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("min_joint_delta_rad", 0.01)
        self.declare_parameter("playback_step_interval_sec", 0.02)

        self.database = RobotDatabase(str(self.get_parameter("robot_db").value))
        legacy_groups_file = str(self.get_parameter("legacy_groups_file").value)
        if legacy_groups_file:
            self.database.migrate_action_groups(legacy_groups_file)
        self.step_timeout_sec = float(self.get_parameter("step_timeout_sec").value)
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.playback_step_interval_sec = float(
            self.get_parameter("playback_step_interval_sec").value
        )
        min_joint_delta_rad = float(self.get_parameter("min_joint_delta_rad").value)
        self.recorders = {
            "left": ArmRecorder(
                self,
                str(self.get_parameter("left_namespace").value),
                min_joint_delta_rad,
                self.cb_group,
            ),
            "right": ArmRecorder(
                self,
                str(self.get_parameter("right_namespace").value),
                min_joint_delta_rad,
                self.cb_group,
            ),
        }
        self.active_arm: str | None = None
        self.active_group: str | None = None
        self.lock = threading.Lock()

        self.create_service(
            StartTeach,
            "start_teach",
            self._start_cb,
            callback_group=self.cb_group,
        )
        self.create_service(
            StopTeach,
            "stop_teach",
            self._stop_cb,
            callback_group=self.cb_group,
        )
        self.get_logger().info("Teach action group node started.")

    def _call_teach_mode(self, recorder: ArmRecorder, enabled: bool) -> tuple[bool, str]:
        if not recorder.client.wait_for_service(timeout_sec=self.service_timeout_sec):
            return False, f"service not available: {recorder._topic('set_teach_mode')}"
        req = SetBool.Request()
        req.data = enabled
        done = threading.Event()
        future = recorder.client.call_async(req)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=self.service_timeout_sec):
            return False, "set_teach_mode timeout"
        result = future.result()
        if result is None:
            return False, "set_teach_mode failed"
        return bool(result.success), str(result.message)

    def _start_cb(self, request, response):
        arm = str(request.arm_target).strip().lower()
        if arm not in self.recorders:
            response.success = False
            response.message = "arm_target must be left or right"
            return response
        with self.lock:
            if self.active_arm is not None:
                response.success = False
                response.message = f"{self.active_arm} is already recording"
                return response
            ok, message = self._call_teach_mode(self.recorders[arm], True)
            if not ok:
                response.success = False
                response.message = message
                return response
            self.recorders[arm].start()
            self.active_arm = arm
            self.active_group = str(request.group_name).strip() or None
        response.success = True
        response.message = f"{arm} entered teach mode"
        return response

    def _stop_cb(self, request, response):
        with self.lock:
            arm = str(request.arm_target).strip().lower() or self.active_arm
            if arm not in self.recorders or arm != self.active_arm:
                response.success = False
                response.message = "no matching active recording"
                return response
            group_name = str(request.group_name).strip() or self.active_group
            if not group_name:
                side_name = "左臂" if arm == "left" else "右臂"
                group_name = f"未命名-{side_name}-{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}"
            samples = self.recorders[arm].stop()
            ok, message = self._call_teach_mode(self.recorders[arm], False)
            self.active_arm = None
            self.active_group = None
        if not ok:
            response.success = False
            response.message = message
            return response
        if not samples:
            response.success = True
            response.message = "exited teach mode; no joint samples recorded"
            response.group_name = group_name
            response.sample_count = 0
            response.groups_file = str(self.database.path)
            return response
        self._write_group(group_name, arm, samples)
        response.success = True
        response.message = f"saved action group {group_name}"
        response.group_name = group_name
        response.sample_count = len(samples)
        response.groups_file = str(self.database.path)
        return response

    def _write_group(self, group_name: str, arm: str, samples: list[JointState]):
        self.database.save_action_group(group_name, arm, [{
            "name": list(msg.name),
            "position": [float(value) for value in msg.position],
            "velocity": [], "effort": [],
            "timeout_sec": self.step_timeout_sec,
            "wait_reach": False,
            "hold_sec": self.playback_step_interval_sec,
        } for msg in samples])


def main(args=None):
    rclpy.init(args=args)
    node = TeachActionGroupNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
