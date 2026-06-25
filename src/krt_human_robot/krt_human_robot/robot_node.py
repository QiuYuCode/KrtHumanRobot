"""KrtHumanRobot core ROS node."""

from __future__ import annotations

import os
import sys
import time

import rclpy
from loguru import logger
from py_trees.common import Duration, Status
from py_trees_ros.trees import BehaviourTree
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from voice_interfaces.action import PlayAudio
from voice_interfaces.srv import DescribeScene, SynthesizeSpeech

from krt_human_robot.tree_factory import create_tree
from krt_human_robot.config import load_config
from krt_human_robot.behaviors.core.actions.vision import execute_describe_scene


class KrtHumanRobotNode(Node):
    """Core behavior tree entry for the humanoid robot."""

    def __init__(self) -> None:
        super().__init__("krtHumanRobot")

        self.declare_parameter("config_file", "")
        self.declare_parameter("tick_interval_ms", 100)
        self.declare_parameter("enable_monitor", False)
        self.declare_parameter("snapshot_period_s", 2.0)
        self.declare_parameter("snapshot_blackboard_data", True)
        self.declare_parameter("snapshot_blackboard_activity", False)

        config_path = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_path:
            config_path = os.environ.get("KRT_HUMAN_ROBOT_CONFIG", "").strip()

        tick_ms = self.get_parameter("tick_interval_ms").get_parameter_value().integer_value
        env_tick = os.environ.get("KRT_HUMAN_ROBOT_TICK_INTERVAL_MS", "").strip()
        if env_tick:
            tick_ms = int(env_tick)

        monitor_enabled = (
            self.get_parameter("enable_monitor").get_parameter_value().bool_value
        )
        env_monitor = os.environ.get("KRT_HUMAN_ROBOT_ENABLE_MONITOR", "").strip().lower()
        if env_monitor in {"1", "true", "yes", "on"}:
            monitor_enabled = True
        elif env_monitor in {"0", "false", "no", "off"}:
            monitor_enabled = False

        snapshot_period_s = (
            self.get_parameter("snapshot_period_s").get_parameter_value().double_value
        )
        env_snapshot_period = os.environ.get("KRT_HUMAN_ROBOT_SNAPSHOT_PERIOD_S", "").strip()
        if env_snapshot_period:
            snapshot_period_s = float(env_snapshot_period)

        snapshot_blackboard_data = (
            self.get_parameter("snapshot_blackboard_data")
            .get_parameter_value()
            .bool_value
        )
        env_bb_data = os.environ.get("KRT_HUMAN_ROBOT_SNAPSHOT_BLACKBOARD_DATA", "").strip().lower()
        if env_bb_data in {"1", "true", "yes", "on"}:
            snapshot_blackboard_data = True
        elif env_bb_data in {"0", "false", "no", "off"}:
            snapshot_blackboard_data = False

        snapshot_blackboard_activity = (
            self.get_parameter("snapshot_blackboard_activity")
            .get_parameter_value()
            .bool_value
        )
        env_bb_activity = os.environ.get(
            "KRT_HUMAN_ROBOT_SNAPSHOT_BLACKBOARD_ACTIVITY", ""
        ).strip().lower()
        if env_bb_activity in {"1", "true", "yes", "on"}:
            snapshot_blackboard_activity = True
        elif env_bb_activity in {"0", "false", "no", "off"}:
            snapshot_blackboard_activity = False

        self._config = load_config(config_path) if config_path else load_config()
        self._config.enable_monitor = monitor_enabled

        self._tts_client = self.create_client(
            SynthesizeSpeech, "/voice/tts/synthesize"
        )
        self._describe_scene_service = self.create_service(
            DescribeScene,
            "/krt_human_robot/vision/describe_scene",
            self._handle_describe_scene,
        )
        root = create_tree(self._config)
        self._tree = BehaviourTree(root, unicode_tree_debug=False)
        self._tree.setup(node=self, timeout=30.0)
        self.set_parameters([
            Parameter(
                "default_snapshot_stream",
                Parameter.Type.BOOL,
                monitor_enabled,
            ),
            Parameter(
                "default_snapshot_period",
                Parameter.Type.DOUBLE,
                max(0.1, snapshot_period_s) if monitor_enabled else Duration.INFINITE.value,
            ),
            Parameter(
                "default_snapshot_blackboard_data",
                Parameter.Type.BOOL,
                snapshot_blackboard_data,
            ),
            Parameter(
                "default_snapshot_blackboard_activity",
                Parameter.Type.BOOL,
                snapshot_blackboard_activity,
            ),
        ])

        self._tick_ms = max(10, int(tick_ms))
        self.get_logger().info("krtHumanRobot core behavior tree started")
        self.get_logger().info(f"  config: {config_path or 'default'}")
        self.get_logger().info(f"  monitor: {'enabled' if monitor_enabled else 'disabled'}")

        if self._config.startup_sound_enabled:
            self._speak_startup(self._config.startup_sound_text)

        self._tree.tick_tock(
            period_ms=self._tick_ms,
            pre_tick_handler=_pre_tick_healthcheck,
            post_tick_handler=_post_tick_reset_root,
        )

    def destroy_node(self) -> bool:
        try:
            self._tree.shutdown()
        except Exception as exc:
            self.get_logger().warning(f"behavior tree shutdown failed: {exc}")
        return super().destroy_node()

    def _speak_startup(self, text: str) -> None:
        play_client = ActionClient(self, PlayAudio, "/voice/playback/play")
        req = SynthesizeSpeech.Request()
        req.text = text
        req.language = "zh-CN"
        req.style = "default"
        req.priority = 2

        deadline = time.time() + 30.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._tts_client.service_is_ready() and play_client.server_is_ready():
                break
        else:
            self.get_logger().warning("startup sound skipped: voice services not ready")
            return

        future = self._tts_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
            resp = future.result()
            if resp is not None and resp.accepted:
                time.sleep(max(0.1, float(resp.estimated_duration_sec)) + 0.15)
                return
            error_message = getattr(resp, "error_message", "") if resp else ""
            self.get_logger().warning(f"startup sound failed: {error_message or 'unknown'}")
        except Exception:
            self.get_logger().warning("startup sound request failed")

    def _handle_describe_scene(
        self,
        request: DescribeScene.Request,
        response: DescribeScene.Response,
    ) -> DescribeScene.Response:
        camera_id = (request.camera_id or self._config.default_camera).strip()
        question = (request.question or "请描述你看到的场景").strip()
        try:
            response.description = execute_describe_scene(
                self._config,
                question=question,
                camera_id=camera_id,
            )
            response.success = True
        except Exception as exc:
            response.success = False
            response.error_message = str(exc)
        return response


def _post_tick_reset_root(tree: BehaviourTree) -> None:
    root = tree.root
    if root.status in (Status.SUCCESS, Status.FAILURE):
        logger.debug("dialog turn finished, returning to standby")
        root.stop(Status.INVALID)
        time.sleep(0.1)


def _pre_tick_healthcheck(tree: BehaviourTree) -> None:
    if tree.root is None:
        raise RuntimeError("behavior tree root is empty")


def main(args: list[str] | None = None) -> None:
    logger.remove()
    logger.add(
        sys.stderr,
        format=(
            "<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | "
            "<cyan>{name}</cyan> - <level>{message}</level>"
        ),
        level="INFO",
        colorize=True,
    )

    rclpy.init(args=args)
    node = KrtHumanRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
