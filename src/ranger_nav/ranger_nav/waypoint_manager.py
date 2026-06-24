"""Waypoint storage and cruise CLI for ranger_nav."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_WAYPOINTS_FILE = "~/maps/waypoints.yaml"
DEFAULT_INPUT_TOPIC = "/input_at_waypoint/input"
DEFAULT_IMAGE_TOPIC = "/camera/color/image_raw"
DEFAULT_IMAGE_DIR = "~/maps/waypoint_images"
DEFAULT_WAIT_MS = 200
DEFAULT_NAVIGATE_ACTION = "/navigate_to_pose"
DEFAULT_TTS_SERVICE = "/voice/tts/synthesize"
DEFAULT_ARM_ACTION = "/run_action_group"


@dataclass
class Waypoint:
    name: str
    pose: PoseStamped
    task: str = "wait"
    args: dict[str, Any] | None = None


class WaypointStore:
    """YAML-backed waypoint storage."""

    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser()

    def load(self) -> list[Waypoint]:
        if not self.path.exists():
            return []
        with self.path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        items = data.get("waypoints", [])
        if not isinstance(items, list):
            raise ValueError(f"{self.path} 中 waypoints 必须是列表")
        return [self._from_dict(item) for item in items]

    def save(self, waypoints: list[Waypoint]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "waypoints": [self._to_dict(wp) for wp in waypoints],
        }
        with self.path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    def next_name(self, waypoints: list[Waypoint]) -> str:
        used = {wp.name for wp in waypoints}
        idx = 1
        while True:
            name = f"wp_{idx:03d}"
            if name not in used:
                return name
            idx += 1

    @staticmethod
    def _from_dict(data: dict[str, Any]) -> Waypoint:
        pose = PoseStamped()
        pose.header.frame_id = str(data.get("frame_id", "map"))
        pose.header.stamp = Time()
        position = data.get("position", {}) or {}
        orientation = data.get("orientation", {}) or {}
        pose.pose.position.x = float(position.get("x", 0.0))
        pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.0))
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return Waypoint(
            name=str(data["name"]),
            pose=pose,
            task=str(data.get("task", "wait") or "wait"),
            args=dict(data.get("args", {}) or {}),
        )

    @staticmethod
    def _to_dict(wp: Waypoint) -> dict[str, Any]:
        pose = wp.pose.pose
        return {
            "name": wp.name,
            "frame_id": wp.pose.header.frame_id or "map",
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
            "task": wp.task,
            "args": wp.args or {},
        }


class WaypointNode(Node):
    """Executes waypoint operations against TF and Nav2."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("waypoint_manager")
        self.args = args
        self.store = WaypointStore(args.file)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, args.navigate_action)
        self.input_seen = False
        self.input_sub = None

    def mark(self, name: str | None, task: str, task_args: dict[str, Any]) -> int:
        waypoints = self.store.load()
        if not name:
            name = self.store.next_name(waypoints)
        if any(wp.name == name for wp in waypoints):
            self.get_logger().error(f"点位已存在: {name}")
            return 1

        pose = self.current_pose()
        waypoints.append(Waypoint(name=name, pose=pose, task=task, args=task_args))
        self.store.save(waypoints)
        self.get_logger().info(
            f"已保存点位 {name}: x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, task={task}"
        )
        return 0

    def list_waypoints(self) -> int:
        waypoints = self.store.load()
        if not waypoints:
            print("没有保存的点位。")
            return 0
        for idx, wp in enumerate(waypoints, start=1):
            pose = wp.pose.pose
            print(
                f"{idx}. {wp.name} "
                f"x={pose.position.x:.3f} y={pose.position.y:.3f} "
                f"task={wp.task} args={json.dumps(wp.args or {}, ensure_ascii=False)}"
            )
        return 0

    def remove(self, name: str) -> int:
        waypoints = self.store.load()
        kept = [wp for wp in waypoints if wp.name != name]
        if len(kept) == len(waypoints):
            self.get_logger().error(f"点位不存在: {name}")
            return 1
        self.store.save(kept)
        self.get_logger().info(f"已删除点位: {name}")
        return 0

    def clear(self) -> int:
        self.store.save([])
        self.get_logger().info("已清空所有点位。")
        return 0

    def continue_input(self) -> int:
        pub = self.create_publisher(Empty, self.args.input_topic, 10)
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            pub.publish(Empty())
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(f"已发布继续信号: {self.args.input_topic}")
        return 0

    def cruise(self, names: list[str], repeat: int | None, loop: bool) -> int:
        selected = self.select_waypoints(names)
        if not selected:
            self.get_logger().error("没有可巡航的点位。")
            return 1
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Nav2 action 不可用: {self.args.navigate_action}")
            return 1

        rounds_done = 0
        while loop or rounds_done < (repeat or 1):
            rounds_done += 1
            self.get_logger().info(f"开始第 {rounds_done} 轮巡航。")
            for wp in selected:
                if not self.navigate_to(wp):
                    return 1
                if not self.run_task(wp):
                    return 1
            if loop:
                continue
        self.get_logger().info("巡航完成。")
        return 0

    def select_waypoints(self, names: list[str]) -> list[Waypoint]:
        waypoints = self.store.load()
        if not names:
            return waypoints
        by_name = {wp.name: wp for wp in waypoints}
        missing = [name for name in names if name not in by_name]
        if missing:
            raise ValueError(f"点位不存在: {', '.join(missing)}")
        return [by_name[name] for name in names]

    def current_pose(self) -> PoseStamped:
        deadline = time.monotonic() + 5.0
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
                transform = self.tf_buffer.lookup_transform(
                    self.args.map_frame,
                    self.args.base_frame,
                    RclpyTime(),
                    timeout=Duration(seconds=0.0),
                )
                pose = PoseStamped()
                pose.header.frame_id = self.args.map_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                return pose
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"读取 TF 失败: {self.args.map_frame} -> {self.args.base_frame}: {last_error}"
        )

    def navigate_to(self, wp: Waypoint) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = wp.pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"导航到点位: {wp.name}")
        future = self.nav_client.send_goal_async(goal)
        goal_handle = self.wait_future(future, "发送导航目标超时")
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"导航目标被拒绝: {wp.name}")
            return False
        result_future = goal_handle.get_result_async()
        result = self.wait_future(result_future, "等待导航结果超时", timeout=None)
        if result is None:
            return False
        status = int(result.status)
        if status != 4:
            self.get_logger().error(f"导航失败: {wp.name}, status={status}")
            return False
        self.get_logger().info(f"到达点位: {wp.name}")
        return True

    def run_task(self, wp: Waypoint) -> bool:
        task = (wp.task or "wait").strip()
        args = wp.args or {}
        if task == "wait":
            wait_ms = int(args.get("wait_ms", self.args.default_wait_ms))
            time.sleep(max(0.0, wait_ms / 1000.0))
            return True
        if task == "photo":
            return self.take_photo(wp, args)
        if task == "input":
            return self.wait_input(wp, args)
        if task == "speak":
            return self.speak(args)
        if task == "arm_group":
            return self.run_arm_group(args)
        self.get_logger().error(f"未知点位任务: {task}")
        return False

    def take_photo(self, wp: Waypoint, args: dict[str, Any]) -> bool:
        try:
            import cv2
            from cv_bridge import CvBridge
        except ImportError as exc:
            self.get_logger().error(f"photo 任务依赖缺失: {exc}")
            return False

        image_topic = str(args.get("topic", self.args.image_topic))
        timeout_s = float(args.get("timeout_s", 5.0))
        image_dir = Path(str(args.get("dir", self.args.image_dir))).expanduser()
        image_dir.mkdir(parents=True, exist_ok=True)
        bridge = CvBridge()
        future: Future = Future()

        def callback(msg: Image) -> None:
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(Image, image_topic, callback, 10)
        msg = self.wait_future(future, "等待相机图像超时", timeout=timeout_s)
        self.destroy_subscription(sub)
        if msg is None:
            return False
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        filename = image_dir / f"{wp.name}_{stamp}.jpg"
        if not cv2.imwrite(str(filename), frame):
            self.get_logger().error(f"保存照片失败: {filename}")
            return False
        self.get_logger().info(f"已保存照片: {filename}")
        return True

    def wait_input(self, wp: Waypoint, args: dict[str, Any]) -> bool:
        timeout_s = float(args.get("timeout_s", 0.0))
        self.input_seen = False

        def callback(_msg: Empty) -> None:
            self.input_seen = True

        self.input_sub = self.create_subscription(
            Empty, self.args.input_topic, callback, 10
        )
        self.get_logger().info(f"点位 {wp.name} 等待继续信号: {self.args.input_topic}")
        deadline = None if timeout_s <= 0 else time.monotonic() + timeout_s
        while not self.input_seen:
            if deadline is not None and time.monotonic() > deadline:
                self.destroy_subscription(self.input_sub)
                self.input_sub = None
                self.get_logger().error(f"等待继续信号超时: {wp.name}")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(self.input_sub)
        self.input_sub = None
        return True

    def speak(self, args: dict[str, Any]) -> bool:
        try:
            from voice_interfaces.srv import SynthesizeSpeech
        except ImportError as exc:
            self.get_logger().error(f"speak 任务依赖缺失: {exc}")
            return False
        text = str(args.get("text", "")).strip()
        if not text:
            self.get_logger().error("speak 任务缺少 args.text")
            return False
        client = self.create_client(SynthesizeSpeech, self.args.tts_service)
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"TTS 服务不可用: {self.args.tts_service}")
            return False
        request = SynthesizeSpeech.Request()
        request.text = text
        request.language = str(args.get("language", "zh"))
        request.style = str(args.get("style", ""))
        request.priority = int(args.get("priority", 5))
        response = self.wait_future(client.call_async(request), "TTS 调用超时", 10.0)
        if response is None or not response.accepted:
            detail = getattr(response, "error_message", "") if response else ""
            self.get_logger().error(f"TTS 播报失败: {detail}")
            return False
        return True

    def run_arm_group(self, args: dict[str, Any]) -> bool:
        try:
            from agx_action_group_interfaces.action import RunActionGroup
        except ImportError as exc:
            self.get_logger().error(f"arm_group 任务依赖缺失: {exc}")
            return False
        group_name = str(args.get("group_name", "")).strip()
        if not group_name:
            self.get_logger().error("arm_group 任务缺少 args.group_name")
            return False
        client = ActionClient(self, RunActionGroup, self.args.arm_action)
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"动作组 action 不可用: {self.args.arm_action}")
            return False
        goal = RunActionGroup.Goal()
        goal.group_name = group_name
        goal.repeat_count = int(args.get("repeat_count", 1))
        goal.arm_target = str(args.get("arm_target", "left"))
        goal_handle = self.wait_future(client.send_goal_async(goal), "发送动作组超时")
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"动作组目标被拒绝: {group_name}")
            return False
        result = self.wait_future(goal_handle.get_result_async(), "等待动作组结果超时", None)
        return bool(result and result.result.success)

    def wait_future(
        self,
        future: Future,
        timeout_message: str,
        timeout: float | None = 15.0,
    ) -> Any:
        deadline = None if timeout is None else time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if deadline is not None and time.monotonic() > deadline:
                self.get_logger().error(timeout_message)
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result() if future.done() else None


def _parse_task_args(raw: str) -> dict[str, Any]:
    if not raw:
        return {}
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError("--args 必须是 JSON object")
    return data


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须是正整数")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ranger_nav waypoint manager")
    parser.add_argument("--file", default=DEFAULT_WAYPOINTS_FILE)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--navigate-action", default=DEFAULT_NAVIGATE_ACTION)
    parser.add_argument("--input-topic", default=DEFAULT_INPUT_TOPIC)
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--image-dir", default=DEFAULT_IMAGE_DIR)
    parser.add_argument("--default-wait-ms", type=int, default=DEFAULT_WAIT_MS)
    parser.add_argument("--tts-service", default=DEFAULT_TTS_SERVICE)
    parser.add_argument("--arm-action", default=DEFAULT_ARM_ACTION)

    subparsers = parser.add_subparsers(dest="command", required=True)
    mark = subparsers.add_parser("mark")
    mark.add_argument("name", nargs="?")
    mark.add_argument("--task", default="wait")
    mark.add_argument("--args", default="{}")

    subparsers.add_parser("list")
    remove = subparsers.add_parser("remove")
    remove.add_argument("name")
    subparsers.add_parser("clear")

    cruise = subparsers.add_parser("cruise")
    cruise.add_argument("names", nargs="*")
    mode = cruise.add_mutually_exclusive_group()
    mode.add_argument("--repeat", type=_positive_int)
    mode.add_argument("--loop", action="store_true")

    subparsers.add_parser("continue_input")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    rclpy.init()
    node = WaypointNode(args)
    try:
        if args.command == "mark":
            exit_code = node.mark(args.name, args.task, _parse_task_args(args.args))
        elif args.command == "list":
            exit_code = node.list_waypoints()
        elif args.command == "remove":
            exit_code = node.remove(args.name)
        elif args.command == "clear":
            exit_code = node.clear()
        elif args.command == "cruise":
            exit_code = node.cruise(args.names, args.repeat, args.loop)
        elif args.command == "continue_input":
            exit_code = node.continue_input()
        else:
            parser.error(f"未知命令: {args.command}")
            exit_code = 2
    except (RuntimeError, ValueError, yaml.YAMLError, json.JSONDecodeError) as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
