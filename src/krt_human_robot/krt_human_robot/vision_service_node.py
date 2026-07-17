"""Standalone scene-description service for Web-managed routines."""

from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from voice_interfaces.srv import DescribeScene

from krt_human_robot.behaviors.core.actions.vision import execute_describe_scene
from krt_human_robot.config import load_config


class VisionServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_service")
        self.declare_parameter("config_file", "")
        config_file = str(self.get_parameter("config_file").value).strip()
        config_file = config_file or os.environ.get("KRT_HUMAN_ROBOT_CONFIG", "")
        self.config = load_config(config_file or None)
        self.service = self.create_service(
            DescribeScene,
            "/krt_human_robot/vision/describe_scene",
            self._describe,
        )

    def _describe(self, request, response):
        try:
            response.description = execute_describe_scene(
                self.config,
                question=str(request.question or "请描述你看到的场景"),
                camera_id=str(request.camera_id or self.config.default_camera),
            )
            response.success = True
        except Exception as exc:
            response.success = False
            response.error_message = str(exc)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
