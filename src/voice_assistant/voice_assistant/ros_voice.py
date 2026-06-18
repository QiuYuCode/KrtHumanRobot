"""ROS /voice 接口辅助函数（供 bt_manager 行为树节点调用）。"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from voice_interfaces.srv import SynthesizeSpeech


def speak_blocking(
    node: Node,
    tts_client,
    text: str,
    *,
    language: str = "zh-CN",
    style: str = "default",
    priority: int = 2,
    timeout_sec: float = 3.0,
) -> bool:
    """通过 /voice/tts/synthesize 阻塞播报短文本。"""
    if not text.strip() or tts_client is None:
        return False
    if not tts_client.wait_for_service(timeout_sec=0.2):
        return False
    req = SynthesizeSpeech.Request()
    req.text = text
    req.language = language
    req.style = style
    req.priority = priority
    future = tts_client.call_async(req)
    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        resp = future.result()
        return resp is not None and resp.accepted
    except Exception:
        return False
