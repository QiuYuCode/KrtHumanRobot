"""语音助手 ROS2 节点：py_trees_ros 驱动行为树 + VoiceEngine。"""

from __future__ import annotations

import os
import sys
import time

import rclpy
from loguru import logger
from py_trees.common import Duration, Status
from py_trees_ros.trees import BehaviourTree
from rclpy.node import Node
from rclpy.parameter import Parameter

from voice_assistant.config import RobotConfig, load_config
from voice_assistant.engine import VoiceEngine
from voice_assistant.tree_factory import create_tree


class VoiceAssistantNode(Node):
    """语音助手主节点。"""

    def __init__(self) -> None:
        super().__init__("voice_assistant")

        self.declare_parameter("config_file", "")
        self.declare_parameter("tick_interval_ms", 100)
        self.declare_parameter("enable_monitor", False)
        self.declare_parameter("snapshot_period_s", 2.0)
        self.declare_parameter("snapshot_blackboard_data", True)
        self.declare_parameter("snapshot_blackboard_activity", False)

        config_path = self.get_parameter("config_file").get_parameter_value().string_value
        if not config_path:
            config_path = os.environ.get("VOICE_ASSISTANT_CONFIG", "").strip()

        tick_ms = self.get_parameter("tick_interval_ms").get_parameter_value().integer_value
        env_tick = os.environ.get("VOICE_TICK_INTERVAL_MS", "").strip()
        if env_tick:
            tick_ms = int(env_tick)
        monitor_enabled = (
            self.get_parameter("enable_monitor").get_parameter_value().bool_value
        )
        env_monitor = os.environ.get("VOICE_ENABLE_MONITOR", "").strip().lower()
        if env_monitor in {"1", "true", "yes", "on"}:
            monitor_enabled = True
        elif env_monitor in {"0", "false", "no", "off"}:
            monitor_enabled = False
        snapshot_period_s = (
            self.get_parameter("snapshot_period_s").get_parameter_value().double_value
        )
        env_snapshot_period = os.environ.get("VOICE_SNAPSHOT_PERIOD_S", "").strip()
        if env_snapshot_period:
            snapshot_period_s = float(env_snapshot_period)
        snapshot_blackboard_data = (
            self.get_parameter("snapshot_blackboard_data")
            .get_parameter_value()
            .bool_value
        )
        env_bb_data = os.environ.get("VOICE_SNAPSHOT_BLACKBOARD_DATA", "").strip().lower()
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
            "VOICE_SNAPSHOT_BLACKBOARD_ACTIVITY", ""
        ).strip().lower()
        if env_bb_activity in {"1", "true", "yes", "on"}:
            snapshot_blackboard_activity = True
        elif env_bb_activity in {"0", "false", "no", "off"}:
            snapshot_blackboard_activity = False

        self._config: RobotConfig = (
            load_config(config_path) if config_path else load_config()
        )
        self._config.enable_monitor = monitor_enabled

        self._engine = VoiceEngine(self._config)
        self._engine.start()

        root = create_tree(self._engine, self._config)
        self._tree = BehaviourTree(root, unicode_tree_debug=monitor_enabled)
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
        self._tree.tick_tock(
            period_ms=self._tick_ms,
            pre_tick_handler=_pre_tick_healthcheck,
            post_tick_handler=_post_tick_reset_root,
        )

        self.get_logger().info("语音助手已启动，等待唤醒词...")
        self.get_logger().info(f"  配置文件: {config_path or '默认'}")
        self.get_logger().info(f"  监控发布: {'启用' if monitor_enabled else '关闭'}")
        self.get_logger().info(
            f"  意图模式: {'LLM 规划' if self._config.use_llm_planner else '关键词匹配'}"
        )

        if self._config.startup_sound_enabled:
            self._engine.speak_blocking(self._config.startup_sound_text)

    def destroy_node(self) -> bool:
        try:
            self._tree.shutdown()
        except Exception as exc:
            self.get_logger().warning(f"行为树 shutdown 异常: {exc}")
        try:
            self._engine.stop()
        except Exception as exc:
            self.get_logger().warning(f"VoiceEngine stop 异常: {exc}")
        return super().destroy_node()


def _post_tick_reset_root(tree: BehaviourTree) -> None:
    """回合结束后重置行为树，回到唤醒待机。"""
    root = tree.root
    if root.status in (Status.SUCCESS, Status.FAILURE):
        logger.debug("回合结束，回到待机")
        root.stop(Status.INVALID)
        time.sleep(0.1)


def _pre_tick_healthcheck(tree: BehaviourTree) -> None:
    """树 tick 前的最小健康检查。"""
    if tree.root is None:
        raise RuntimeError("行为树 root 为空，无法执行 tick")


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
    node = VoiceAssistantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
