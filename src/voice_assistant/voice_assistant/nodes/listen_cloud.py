"""云端监听节点（兼容类）：复用统一 ListenCommand。"""

from voice_assistant.nodes.listen import ListenCommand


class ListenCloudCommand(ListenCommand):
    """兼容旧类名，内部复用统一 ROS ASR 客户端实现。"""
