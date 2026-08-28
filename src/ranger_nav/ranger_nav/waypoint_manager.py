"""Waypoint storage and cruise CLI for ranger_nav."""

from __future__ import annotations

import argparse
import copy
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from krt_task.robot_db import RobotDatabase, WaypointRecord
from krt_task_interfaces.action import RunRoutine
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time as RclpyTime
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener

DEFAULT_ROBOT_DB = "~/maps/krt_robot.db"
DEFAULT_INPUT_TOPIC = "/input_at_waypoint/input"
DEFAULT_WAIT_MS = 200
DEFAULT_NAVIGATE_ACTION = "/navigate_to_pose"
DEFAULT_ROUTINE_ACTION = "/krt_task/run_routine"
DEFAULT_ARRIVAL_TOLERANCE_M = 0.25
DEFAULT_ARRIVAL_RETRIES = 1
DEFAULT_ACCURACY_REPORT = "~/maps/waypoint_accuracy.yaml"
DEFAULT_APPROACH_DISTANCE_M = 0.0


@dataclass
class Waypoint:
    name: str
    pose: PoseStamped
    routine: str = ""
    map_id: str | None = None


@dataclass
class AccuracySample:
    waypoint: str
    target_x: float
    target_y: float
    target_yaw: float
    actual_x: float
    actual_y: float
    actual_yaw: float
    xy_error: float
    yaw_error: float


class WaypointStore:
    """ROS pose adapter for the shared robot database."""

    def __init__(self, path: str) -> None:
        self.database = RobotDatabase(path)

    def resolve_map_id(
        self, map_id: str | None = None, *, required: bool = False
    ) -> str | None:
        if map_id:
            record = self.database.get_map(map_id)
            if record.status != "ready":
                raise ValueError("只能使用已就绪的地图")
            return record.id
        selected = self.database.get_selected_map()
        if selected is not None:
            return selected.id
        if required or self.database.list_maps():
            raise ValueError("请先选择地图")
        return None

    def load(self, map_id: str | None = None) -> list[Waypoint]:
        resolved = self.resolve_map_id(map_id)
        return [
            self._from_record(item) for item in self.database.list_waypoints(resolved)
        ]

    def save(self, waypoint: Waypoint, map_id: str | None = None) -> None:
        pose = waypoint.pose.pose
        resolved = self.resolve_map_id(map_id or waypoint.map_id, required=True)
        self.database.save_waypoint(
            WaypointRecord(
                name=waypoint.name,
                frame_id=waypoint.pose.header.frame_id or "map",
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
                qx=pose.orientation.x,
                qy=pose.orientation.y,
                qz=pose.orientation.z,
                qw=pose.orientation.w,
                routine=waypoint.routine,
                map_id=resolved,
            )
        )

    def next_name(self, waypoints: list[Waypoint]) -> str:
        used = {wp.name for wp in waypoints}
        idx = 1
        while True:
            name = f"wp_{idx:03d}"
            if name not in used:
                return name
            idx += 1

    @staticmethod
    def _from_record(data: WaypointRecord) -> Waypoint:
        pose = PoseStamped()
        pose.header.frame_id = data.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            data.x,
            data.y,
            data.z,
        )
        pose.pose.orientation.x, pose.pose.orientation.y = data.qx, data.qy
        pose.pose.orientation.z, pose.pose.orientation.w = data.qz, data.qw
        return Waypoint(
            name=data.name, pose=pose, routine=data.routine, map_id=data.map_id
        )


class WaypointNode(Node):
    """Executes waypoint operations against TF and Nav2."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("waypoint_manager")
        self.args = args
        self.store = WaypointStore(args.robot_db)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, args.navigate_action)
        self.routine_client = ActionClient(self, RunRoutine, args.routine_action)
        self.input_seen = False
        self.input_sub = None
        self.accuracy_samples: list[AccuracySample] = []

    def mark(self, name: str | None, routine: str) -> int:
        map_id = self.store.resolve_map_id(self.args.map_id, required=True)
        waypoints = self.store.load(map_id)
        if not name:
            name = self.store.next_name(waypoints)
        if any(wp.name == name for wp in waypoints):
            self.get_logger().error(f"点位已存在: {name}")
            return 1
        pose = self.current_pose()
        self.store.save(
            Waypoint(name=name, pose=pose, routine=routine, map_id=map_id), map_id
        )
        self.get_logger().info(
            f"已保存点位 {name}: x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, routine={routine}"
        )
        return 0

    def list_waypoints(self) -> int:
        waypoints = self.store.load(self.args.map_id)
        if not waypoints:
            print("没有保存的点位。")
            return 0
        for idx, wp in enumerate(waypoints, start=1):
            pose = wp.pose.pose
            print(
                f"{idx}. {wp.name} "
                f"x={pose.position.x:.3f} y={pose.position.y:.3f} "
                f"routine={wp.routine or '-'}"
            )
        return 0

    def remove(self, name: str) -> int:
        try:
            self.store.database.delete_waypoint(name)
        except KeyError:
            self.get_logger().error(f"点位不存在: {name}")
            return 1
        self.get_logger().info(f"已删除点位: {name}")
        return 0

    def bind(self, name: str, routine: str) -> int:
        self.store.database.bind_waypoint(name, routine)
        self.get_logger().info(f"点位已绑定: {name} -> {routine or '-'}")
        return 0

    def clear(self) -> int:
        for waypoint in self.store.load(self.args.map_id):
            self.store.database.delete_waypoint(waypoint.name)
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
                    self.report_accuracy()
                    return 1
                if not self.run_task(wp):
                    self.report_accuracy()
                    return 1
            if loop:
                continue
        self.get_logger().info("巡航完成。")
        self.report_accuracy()
        return 0

    def select_waypoints(self, names: list[str]) -> list[Waypoint]:
        waypoints = self.store.load(self.args.map_id)
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
                    timeout=Duration(seconds=0),
                )
                pose = PoseStamped()
                pose.header.frame_id = self.args.map_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                return pose
            except Exception as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(
            f"读取 TF 失败: {self.args.map_frame} -> {self.args.base_frame}: {last_error}"
        )

    def navigate_to(self, wp: Waypoint) -> bool:
        for attempt in range(self.args.arrival_retries + 1):
            approach = self.approach_pose(wp)
            if approach is not None and not self.send_nav_goal(
                approach, wp.name, "预进场"
            ):
                return False
            if not self.send_nav_goal(wp.pose, wp.name, "完整位姿"):
                return False
            if not self.arrival_reached(wp):
                if attempt < self.args.arrival_retries:
                    self.get_logger().warning("XY 未到达，重试接近点位")
                    continue
                self.record_accuracy(wp)
                self.get_logger().warning(f"未正确接近点位，继续巡航: {wp.name}")
                return True
            self.record_accuracy(wp)
            self.get_logger().info(f"到达点位: {wp.name}")
            return True
        self.record_accuracy(wp)
        self.get_logger().warning(f"未正确接近点位，继续巡航: {wp.name}")
        return True

    def approach_pose(self, wp: Waypoint) -> PoseStamped | None:
        distance = float(self.args.approach_distance)
        if distance <= 0.0:
            return None
        yaw = _yaw_from_pose(wp.pose)
        pose = copy.deepcopy(wp.pose)
        pose.pose.position.x -= math.cos(yaw) * distance
        pose.pose.position.y -= math.sin(yaw) * distance
        return pose

    def send_nav_goal(self, pose: PoseStamped, name: str, phase: str) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(pose)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"导航到点位: {name} ({phase})")
        future = self.nav_client.send_goal_async(goal)
        goal_handle = self.wait_future(future, "发送导航目标超时")
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"导航目标被拒绝: {name}")
            return False
        result = self.wait_future(
            goal_handle.get_result_async(), "等待导航结果超时", timeout=None
        )
        if result is None:
            return False
        status = int(result.status)
        if status != 4:
            self.get_logger().error(f"导航失败: {name}, status={status}")
            return False
        return True

    def arrival_reached(self, wp: Waypoint) -> bool:
        current = self.current_pose()
        target = wp.pose.pose.position
        actual = current.pose.position
        distance = math.hypot(actual.x - target.x, actual.y - target.y)
        tolerance = float(self.args.arrival_tolerance)
        self.get_logger().info(
            f"点位验收 {wp.name}: target=({target.x:.3f}, {target.y:.3f}) "
            f"actual=({actual.x:.3f}, {actual.y:.3f}) "
            f"distance={distance:.3f}m tolerance={tolerance:.3f}m"
        )
        return distance <= tolerance

    def record_accuracy(self, wp: Waypoint) -> None:
        current = self.current_pose()
        target = wp.pose.pose
        actual = current.pose
        target_yaw = _yaw_from_pose(wp.pose)
        actual_yaw = _yaw_from_pose(current)
        sample = AccuracySample(
            waypoint=wp.name,
            target_x=float(target.position.x),
            target_y=float(target.position.y),
            target_yaw=target_yaw,
            actual_x=float(actual.position.x),
            actual_y=float(actual.position.y),
            actual_yaw=actual_yaw,
            xy_error=math.hypot(
                actual.position.x - target.position.x,
                actual.position.y - target.position.y,
            ),
            yaw_error=abs(_angle_diff(actual_yaw, target_yaw)),
        )
        self.accuracy_samples.append(sample)
        self.get_logger().info(
            f"定位精度 {wp.name}: xy_error={sample.xy_error:.3f}m "
            f"yaw_error={sample.yaw_error:.3f}rad"
        )

    def report_accuracy(self) -> None:
        if not self.accuracy_samples:
            return
        xy_errors = [s.xy_error for s in self.accuracy_samples]
        yaw_errors = [s.yaw_error for s in self.accuracy_samples]
        xy_rms = math.sqrt(sum(e * e for e in xy_errors) / len(xy_errors))
        yaw_rms = math.sqrt(sum(e * e for e in yaw_errors) / len(yaw_errors))
        self.get_logger().info(
            "巡航定位精度统计: "
            f"count={len(xy_errors)} "
            f"xy_avg={sum(xy_errors) / len(xy_errors):.3f}m "
            f"xy_rms={xy_rms:.3f}m "
            f"xy_max={max(xy_errors):.3f}m "
            f"yaw_avg={sum(yaw_errors) / len(yaw_errors):.3f}rad "
            f"yaw_rms={yaw_rms:.3f}rad "
            f"yaw_max={max(yaw_errors):.3f}rad"
        )
        if not self.args.accuracy_report:
            return
        path = Path(self.args.accuracy_report).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "map_frame": self.args.map_frame,
            "base_frame": self.args.base_frame,
            "summary": {
                "count": len(xy_errors),
                "xy_avg": sum(xy_errors) / len(xy_errors),
                "xy_rms": xy_rms,
                "xy_max": max(xy_errors),
                "yaw_avg": sum(yaw_errors) / len(yaw_errors),
                "yaw_rms": yaw_rms,
                "yaw_max": max(yaw_errors),
            },
            "samples": [sample.__dict__ for sample in self.accuracy_samples],
        }
        with path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        self.get_logger().info(f"已写入定位精度报告: {path}")

    def run_task(self, wp: Waypoint) -> bool:
        routine = wp.routine.strip()
        if not routine:
            self.get_logger().info(f"点位 {wp.name} 未绑定 routine，跳过动作。")
            return True
        if not self.routine_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"routine action 不可用: {self.args.routine_action}"
            )
            return False
        goal = RunRoutine.Goal()
        goal.routine_name = routine
        self.get_logger().info(
            f"执行点位 routine: waypoint={wp.name}, routine={routine}"
        )
        goal_handle = self.wait_future(
            self.routine_client.send_goal_async(goal),
            "发送 routine 目标超时",
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"routine 目标被拒绝: {routine}")
            return False
        result = self.wait_future(
            goal_handle.get_result_async(),
            "等待 routine 结果超时",
            timeout=None,
        )
        if result is None:
            return False
        if not result.result.success:
            self.get_logger().error(f"routine 执行失败: {result.result.message}")
            return False
        return True

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


def _yaw_from_pose(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须是正整数")
    return parsed


def _non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("必须是非负整数")
    return parsed


def _positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ranger_nav waypoint manager")
    parser.add_argument("--robot-db", default=DEFAULT_ROBOT_DB)
    parser.add_argument(
        "--map-id",
        default=None,
        help="地图数据库 ID；省略时使用当前选中的地图",
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--navigate-action", default=DEFAULT_NAVIGATE_ACTION)
    parser.add_argument("--routine-action", default=DEFAULT_ROUTINE_ACTION)
    parser.add_argument("--input-topic", default=DEFAULT_INPUT_TOPIC)
    parser.add_argument("--default-wait-ms", type=int, default=DEFAULT_WAIT_MS)
    parser.add_argument(
        "--accuracy-report",
        default=DEFAULT_ACCURACY_REPORT,
        help="巡航定位精度报告 yaml 路径；传空字符串则只打印不写文件",
    )
    parser.add_argument(
        "--arrival-tolerance",
        type=_positive_float,
        default=DEFAULT_ARRIVAL_TOLERANCE_M,
        help="巡航点到达后的 2D 距离验收阈值（米）",
    )
    parser.add_argument(
        "--arrival-retries",
        type=_non_negative_int,
        default=DEFAULT_ARRIVAL_RETRIES,
        help="Nav2 成功但距离验收失败后的重试次数",
    )
    parser.add_argument(
        "--approach-distance",
        type=float,
        default=DEFAULT_APPROACH_DISTANCE_M,
        help="终点前沿目标 yaw 反方向插入预进场点距离（米）；0 表示关闭",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    mark = subparsers.add_parser("mark")
    mark.add_argument("name", nargs="?")
    mark.add_argument("--routine", default="")

    subparsers.add_parser("list")
    remove = subparsers.add_parser("remove")
    remove.add_argument("name")
    bind = subparsers.add_parser("bind")
    bind.add_argument("name")
    bind.add_argument("--routine", required=True)
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
            exit_code = node.mark(args.name, args.routine)
        elif args.command == "list":
            exit_code = node.list_waypoints()
        elif args.command == "remove":
            exit_code = node.remove(args.name)
        elif args.command == "bind":
            exit_code = node.bind(args.name, args.routine)
        elif args.command == "clear":
            exit_code = node.clear()
        elif args.command == "cruise":
            exit_code = node.cruise(args.names, args.repeat, args.loop)
        elif args.command == "continue_input":
            exit_code = node.continue_input()
        else:
            parser.error(f"未知命令: {args.command}")
            exit_code = 2
    except (RuntimeError, ValueError, KeyError) as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
