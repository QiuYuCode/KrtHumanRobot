from __future__ import annotations

from typing import Any

import rclpy


def call_service(node, client, request: Any, timeout_sec: float) -> Any:
    """Wait for a service, call it, and return its response."""
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise TimeoutError(f"服务不可用: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError(f"服务响应超时: {client.srv_name}")
    exception = future.exception()
    if exception is not None:
        raise RuntimeError(str(exception))
    return future.result()


def shutdown_node(node) -> None:
    """Destroy a node and shut down rclpy safely."""
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
